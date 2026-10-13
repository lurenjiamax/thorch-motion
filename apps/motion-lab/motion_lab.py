#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Native, user-local Thor motion inspector. No firmware calibration writes."""
from __future__ import annotations

import argparse
import ctypes as C
import ctypes.util
from collections import Counter, deque
import json
import math
import os
from pathlib import Path
import re
import statistics
import tempfile
import time
from typing import Callable

Vector = tuple[float, float, float]
GRAVITY = 9.80665
FACES = ("+X", "-X", "+Y", "-Y", "+Z", "-Z")
POSES = ("屏幕朝上", "屏幕朝下", "上边朝上", "下边朝上", "左边朝上", "右边朝上")
POSE_GUIDANCE = (
    "按键和下屏朝向天花板，平放在稳固支撑上。",
    "按键和下屏朝向地面，用软垫支撑，避免挤压按键。",
    "让下半机身靠近转轴的上边朝向天花板，立稳。",
    "让下半机身远离转轴的下边朝向天花板，立稳。",
    "让左侧方向键这一边朝向天花板，侧立并支撑。",
    "让右侧操作键这一边朝向天花板，侧立并支撑。",
)
Sample = tuple[float, float, Vector]
SAMPLE = re.compile(r"sensor measurement: X=([-+\deE.]+) Y=([-+\deE.]+) Z=([-+\deE.]+).*?t_us=(\d+)")


def pose_face_hint(history: deque[Sample], now: float) -> tuple[str | None, str]:
    """Detect a reported channel from measured data; never infer physical mounting."""
    rows = [row for row in history if now - 1.2 <= row[1] <= now]
    if len(rows) < 50 or rows[-1][1] - rows[0][1] < 1 or now - rows[-1][1] > .15:
        return None, "请放稳下半机身，等待至少 1 秒新数据。"
    if any(abs(row[1] - row[0]) > .5 for row in rows) or any(b[1] - a[1] > .3 or not 0 < b[0] - a[0] <= .3 for a, b in zip(rows, rows[1:])):
        return None, "数据有延迟或中断，请等待恢复后再采集。"
    vectors = [row[2] for row in rows]
    if max(statistics.pstdev(v[i] for v in vectors) for i in range(3)) > .12:
        return None, "机身还在晃动，请支撑稳后再采集。"
    if any(not 6.0 < math.sqrt(sum(x*x for x in v)) < 14.0 for v in vectors):
        return None, "重力读数未通过，请放稳后重试；持续异常请查看诊断。"
    mean = tuple(statistics.mean(v[i] for v in vectors) for i in range(3))
    axis = max(range(3), key=lambda i: abs(mean[i]))
    sign = 1 if mean[axis] >= 0 else -1
    if abs(mean[axis]) < .65 * math.sqrt(sum(v*v for v in mean)):
        return None, (
            "方向不明确，请按当前图示摆放。无需让横向读数归零。"
            "\n平均读数：{:+.3f} / {:+.3f} / {:+.3f} m/s²".format(*mean)
        )
    return ("+" if sign > 0 else "-") + "XYZ"[axis], "已放稳，请确认姿势后点击采集。"


def pose_mapping_problem(mapping: dict[str, str], pose: str, face: str) -> str | None:
    if any(other != pose and value == face for other, value in mapping.items()):
        return "姿势与已采集重复，请重新摆放。"
    index = POSES.index(pose)
    opposite = mapping.get(POSES[index ^ 1])
    if opposite is not None and opposite != ("-" if face[0] == "+" else "+") + face[1]:
        return "与相反姿势的读数不一致，请确认下半机身已翻转；必要时重置六个姿势。"
    return None


def checked_vector(value: object, limit: float) -> Vector:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("Calibration vector must have three numeric channels")
    if any(type(v) not in (int, float) or abs(v) > limit or not math.isfinite(v) for v in value):
        raise ValueError("Calibration vector is non-finite or outside safe bounds")
    return (float(value[0]), float(value[1]), float(value[2]))


def sphere_parameters(points: list[Vector]) -> tuple[Vector, float]:
    """Solve the four-parameter isotropic sphere without optional dependencies."""
    rows = [[2*v for v in point] + [1.0] for point in points]
    rhs = [sum(v*v for v in point) for point in points]
    matrix = [[sum(row[i]*row[j] for row in rows) for j in range(4)] +
              [sum(row[i]*value for row, value in zip(rows, rhs))] for i in range(4)]
    for column in range(4):
        pivot = max(range(column, 4), key=lambda i: abs(matrix[i][column]))
        if abs(matrix[pivot][column]) < 1e-8:
            raise ValueError("姿势方向覆盖不足，请重新采集六种不同方向。")
        matrix[column], matrix[pivot] = matrix[pivot], matrix[column]
        divisor = matrix[column][column]
        matrix[column] = [v/divisor for v in matrix[column]]
        for i in range(4):
            if i != column:
                factor = matrix[i][column]
                matrix[i] = [a-factor*b for a, b in zip(matrix[i], matrix[column])]
    bias = tuple(matrix[i][4] for i in range(3))
    radius_squared = matrix[3][4] + sum(v*v for v in bias)
    if not math.isfinite(radius_squared) or radius_squared <= 0:
        raise ValueError("球面拟合无效，请重新采集。")
    return checked_vector(bias, 3.0), math.sqrt(radius_squared)


def fit_sphere(points: list[Vector]) -> dict[str, object]:
    if len(points) != 6:
        raise ValueError("请先完成六种姿势。")
    points = [checked_vector(point, 20.0) for point in points]
    bias, radius = sphere_parameters(points)
    scale = GRAVITY/radius
    if not .9 <= scale <= 1.1 or math.sqrt(sum(v*v for v in bias)) > 3.0:
        raise ValueError("偏移或比例超出球面模型范围，未生成校准。")
    directions = [tuple((point[i]-bias[i])/radius for i in range(3)) for point in points]
    if any(sum(a*b for a, b in zip(directions[i], directions[j])) > .8
           for i in range(6) for j in range(i)):
        raise ValueError("采集方向有重复，请检查六种姿势。")
    if any(not any(sum(a*b for a, b in zip(direction, other)) < -.8
                   for other in directions) for direction in directions):
        raise ValueError("缺少相反方向，请检查六种姿势。")
    residuals = [abs(math.sqrt(sum((p[i]-bias[i])**2 for i in range(3)))-radius)*scale for p in points]
    held_out = []
    for i, point in enumerate(points):
        local_bias, local_radius = sphere_parameters(points[:i]+points[i+1:])
        held_out.append(abs(math.sqrt(sum((point[j]-local_bias[j])**2 for j in range(3)))-local_radius)*GRAVITY/local_radius)
    if max(residuals) > .15 or max(held_out) > .25:
        raise ValueError("球面残差或留一姿势误差过大，请重新支撑并采集。")
    return {"model": "sphere", "units": "m/s²", "frame": "unknown",
            "offset": list(bias), "scale": [scale]*3, "gravity_m_s2": GRAVITY,
            "radius_m_s2": radius, "residuals_m_s2": residuals,
            "leave_one_pose_out_m_s2": held_out,
            "validation": "sphere-fit-and-held-out-poses-independent-validation-required",
            "physical_mount_verified": False,
            "operation": "corrected[i] = (raw[i] - offset[i]) * scale[i]"}


def read_calibration(path: Path) -> dict[str, object]:
    """Refuse malformed/incompatible metadata instead of overwriting it."""
    try:
        with path.open("rb") as file:
            raw = file.read(1_048_577)
    except FileNotFoundError:
        return {}
    if len(raw) > 1_048_576:
        raise ValueError("Calibration JSON exceeds 1 MiB")
    text = raw.decode("utf-8")

    def reject_constant(value: str) -> None:
        raise ValueError(f"Invalid JSON numeric constant: {value}")

    def unique_keys(pairs: list[tuple[str, object]]) -> dict[str, object]:
        result: dict[str, object] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"Duplicate calibration JSON key: {key}")
            result[key] = value
        return result

    payload = json.loads(text, parse_constant=reject_constant, object_pairs_hook=unique_keys)
    if not isinstance(payload, dict):
        raise ValueError("Calibration JSON must be an object")
    if "schema" in payload and (type(payload["schema"]) is not int or payload["schema"] != 1):
        raise ValueError("Unsupported calibration schema")
    if payload.get("source", "thor-body-private-ssc") != "thor-body-private-ssc" or payload.get("frame", "unknown") != "unknown":
        raise ValueError("Calibration belongs to a different source/frame")
    if "gyro_bias" in payload:
        bias = checked_vector(payload["gyro_bias"], .15)
        if math.sqrt(sum(v*v for v in bias)) > .15:
            raise ValueError("Gyro bias norm exceeds 0.15 rad/s")
        if payload.get("units", "rad/s") != "rad/s":
            raise ValueError("Unsupported gyro units")
    if "accelerometer" in payload:
        accel = payload["accelerometer"]
        if not isinstance(accel, dict) or accel.get("model") not in ("diagonal", "sphere") or accel.get("units") != "m/s²" or accel.get("frame") != "unknown":
            raise ValueError("Unsupported accelerometer metadata")
        offset = checked_vector(accel.get("offset"), 3.0 if accel.get("model") == "sphere" else .6)
        if accel.get("model") == "sphere" and math.sqrt(sum(v*v for v in offset)) > 3.0:
            raise ValueError("Sphere bias norm exceeds 3 m/s²")
        scales = checked_vector(accel.get("scale"), 1.1)
        if any(value < .9 for value in scales):
            raise ValueError("Accelerometer scales outside 0.9..1.1")
        if accel.get("model") == "sphere" and max(scales)-min(scales) > 1e-9:
            raise ValueError("Sphere model requires one common scale")
    # Also detect overflowing JSON numbers in unrelated fields, preserving only valid JSON.
    json.dumps(payload, allow_nan=False)
    return payload


class SDL:
    """Real SDL2 GameController API (sensor functions require 2.0.14)."""

    def __init__(self) -> None:
        self.lib = C.CDLL(ctypes.util.find_library("SDL2") or "libSDL2-2.0.so.0")
        signatures = {
            "SDL_InitSubSystem": (C.c_int, [C.c_uint32]),
            "SDL_QuitSubSystem": (None, [C.c_uint32]),
            "SDL_NumJoysticks": (C.c_int, []),
            "SDL_IsGameController": (C.c_int, [C.c_int]),
            "SDL_GameControllerOpen": (C.c_void_p, [C.c_int]),
            "SDL_GameControllerClose": (None, [C.c_void_p]),
            "SDL_GameControllerName": (C.c_char_p, [C.c_void_p]),
            "SDL_GameControllerGetAttached": (C.c_int, [C.c_void_p]),
            "SDL_GameControllerHasSensor": (C.c_int, [C.c_void_p, C.c_int]),
            "SDL_GameControllerSetSensorEnabled": (C.c_int, [C.c_void_p, C.c_int, C.c_int]),
            "SDL_GameControllerGetSensorData": (C.c_int, [C.c_void_p, C.c_int, C.POINTER(C.c_float), C.c_int]),
            "SDL_GameControllerUpdate": (None, []),
            "SDL_GetError": (C.c_char_p, []),
        }
        for name, (result, args) in signatures.items():
            function = getattr(self.lib, name)
            function.restype, function.argtypes = result, args
        if self.lib.SDL_InitSubSystem(0x2000) != 0:
            raise RuntimeError(self.error())
        self.controllers: list[tuple[int, str, list[int]]] = []

    def error(self) -> str:
        return (self.lib.SDL_GetError() or b"SDL error").decode(errors="replace")

    def scan(self) -> str:
        self.close_handles()
        names = []
        for index in range(max(0, self.lib.SDL_NumJoysticks())):
            if not self.lib.SDL_IsGameController(index):
                continue
            handle = self.lib.SDL_GameControllerOpen(index)
            if not handle:
                continue
            name = (self.lib.SDL_GameControllerName(handle) or b"Unnamed controller").decode(errors="replace")
            enabled = []
            details = []
            for sensor, label in ((1, "accel"), (2, "gyro")):
                if not self.lib.SDL_GameControllerHasSensor(handle, sensor):
                    details.append(f"{label}: hardware reports no sensor")
                elif self.lib.SDL_GameControllerSetSensorEnabled(handle, sensor, 1) != 0:
                    details.append(f"{label}: enable failed: {self.error()}")
                else:
                    enabled.append(sensor)
                    details.append(f"{label}: enabled")
            self.controllers.append((handle, name, enabled))
            names.append(name + " — " + "; ".join(details))
        return "\n".join(names) or "No SDL game controllers detected. Thor body sensors are independent."

    def poll(self) -> list[tuple[str, str, Vector]]:
        self.lib.SDL_GameControllerUpdate()
        result = []
        for handle, name, sensors in self.controllers:
            if not self.lib.SDL_GameControllerGetAttached(handle):
                continue
            for sensor in sensors:
                values = (C.c_float * 3)()
                if self.lib.SDL_GameControllerGetSensorData(handle, sensor, values, 3) != 0:
                    raise RuntimeError(f"{name}: {self.error()}")
                result.append((name, "accel" if sensor == 1 else "gyro", tuple(values)))
        return result

    def close_handles(self) -> None:
        for handle, _, sensors in self.controllers:
            for sensor in sensors:
                self.lib.SDL_GameControllerSetSensorEnabled(handle, sensor, 0)
            self.lib.SDL_GameControllerClose(handle)
        self.controllers.clear()

    def close(self) -> None:
        self.close_handles()
        self.lib.SDL_QuitSubSystem(0x2000)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", default="/usr/bin/thorch-motion-capture", help="private corrected CLI wrapper")
    options = parser.parse_args()
    # Lazy imports keep --help usable without GUI dependencies/display.
    import gi
    gi.require_version("Gtk", "3.0")
    gi.require_version("Gdk", "3.0")
    from gi.repository import Gdk, Gio, GLib, Gtk

    def text(label: str = "") -> Gtk.Label:
        widget = Gtk.Label(label=label, xalign=0)
        widget.set_line_wrap(True)
        widget.set_max_width_chars(60)
        return widget

    def details(title: str, child: Gtk.Widget) -> Gtk.Expander:
        widget = Gtk.Expander(label=title)
        widget.add(child)
        return widget

    class StatusLabel(Gtk.Box):
        """Keep complete diagnostics while presenting a short visible summary."""
        def __init__(self, scheme: bool = False) -> None:
            super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=6)
            self.scheme = scheme
            self.summary = text()
            self.full = text()
            self.full.set_selectable(True)
            self.pack_start(self.summary, False, False, 0)
            self.pack_start(details("诊断详情", self.full), False, False, 0)

        def set_text(self, value: str) -> None:
            self.full.set_text(value)
            if self.scheme:
                if "Observed desired" in value:
                    summary = "已观察到目标方案"
                elif value.startswith("Current:"):
                    current = value.split("Current: ", 1)[1].split(" (")[0]
                    summary = "当前方案：" + {"Xbox compatibility": "Xbox 兼容", "DualSense motion": "DualSense 体感", "custom": "自定义方案"}.get(current, current)
                    if "switching disabled" in value:
                        summary += " · 无法安全切换"
                elif "error" in value.lower() or "unavailable" in value.lower() or value.startswith("InputPlumber:") or "timed out" in value or "expired" in value:
                    summary = "方案状态异常，请刷新设备；详细原因见诊断详情"
                elif "changed" in value.lower():
                    summary = "设备状态已变化，请刷新后应用"
                elif "enqueued" in value or "Requesting" in value:
                    summary = "等待方案更新"
                elif "Reading" in value or "Refreshing" in value:
                    summary = "正在核对设备"
                elif value.startswith("Connecting") or value.startswith("Loading"):
                    summary = "正在读取设备与方案"
                elif "No composites" in value:
                    summary = "未找到可用设备"
                elif "Neither" in value:
                    summary = "当前服务未提供可用方案"
                else:
                    summary = "选择设备和方案，然后应用"
            elif "rejected" in value.lower() or "failed" in value.lower() or "unavailable" in value.lower() or "unable" in value.lower() or "error" in value.lower() or "unsuccessfully" in value:
                if "Save failed" in value:
                    summary = "保存失败或结果待确认，请查看诊断详情"
                elif "insufficient continuous" in value:
                    summary = "采集未通过：数据不足或中断，请重试"
                elif "motion or invalid gravity" in value:
                    summary = "采集未通过：机身移动或重力异常"
                elif "selected signed channel" in value:
                    summary = "摆放角度未通过，请放正机身后重试"
                elif "fit rejected" in value:
                    summary = "六面拟合未通过，请查看详情并重采"
                else:
                    summary = "采集异常，请查看诊断详情"
            elif value.startswith("Saved user-local"):
                summary = "校准已保存到本机"
            elif value.startswith("Face ") and " accepted " in value:
                count = re.search(r"\((\d+/6)\)", value)
                summary = "此面通过 · 已完成 " + (count[1] if count else "此面") + " · 请继续采集"
            elif "accepted" in value.lower():
                summary = "采集检查通过，结果待保存"
            elif value.startswith("Stopped") or "stopped" in value:
                summary = "已停止采集"
            elif value.startswith("Inspecting"):
                summary = "正在采集机身体感"
            elif value.startswith("Keep Thor"):
                summary = "保持机身静止 · 剩余 " + value.split(" · ")[-1].replace("s remaining", " 秒")
            elif value.startswith("Start inspection"):
                summary = "请先开始采集，再放稳机身"
            elif value.startswith("Six-face session reset"):
                summary = "六面会话已重置，已保存系数保持不变"
            elif "already running" in value:
                summary = "正在采集，请等待完成"
            elif value.startswith("Stop or finish"):
                summary = "请先停止或完成当前采集"
            else:
                summary = value
            self.summary.set_text(summary)

        def get_text(self) -> str:
            return self.full.get_text()

    class SchemePanel(Gtk.Box):
        """Local presets over original InputPlumber target APIs; no backend extension."""

        SERVICE = "org.shadowblip.InputPlumber"
        ROOT = "/org/shadowblip/InputPlumber"
        MANAGER = "org.shadowblip.InputManager"
        COMPOSITE = "org.shadowblip.Input.CompositeDevice"
        TARGET = "org.shadowblip.Input.Target"
        GAMEPADS = {"deck", "deck-uhid", "ds5", "ds5-edge", "hori-steam", "8bitdo-u2", "gamepad", "xb360", "xbox-elite", "xbox-series", "unified-gamepad"}
        NON_GAMEPADS = {"null", "dbus", "debug", "keyboard", "mouse", "touchpad", "touchscreen"}

        def __init__(self, rescan: Callable[[], None]) -> None:
            super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=6)
            self.rescan = rescan
            self.closed = False
            self.generation = 0
            self.bus: Gio.DBusConnection | None = None
            self.cancel = Gio.Cancellable()
            self.subscriptions: list[int] = []
            self.schemes: dict[str, tuple[str, str, bool]] = {}
            self.composites: dict[str, dict[str, object]] = {}
            self.supported: set[str] = set()
            self.target_types: list[str] = []
            self.busy = True
            self.current_known = False
            self.current = "unknown"
            self.controls = Gtk.Box(spacing=6)
            self.pack_start(self.controls, False, False, 0)
            self.device_choice = Gtk.ComboBoxText()
            self.device_choice.set_no_show_all(True)
            self.device_choice.set_tooltip_text("Choose the composite explicitly; identity and complete source paths are checked before switching.")
            self.device_choice.connect("changed", self.selected_device)
            self.pack_start(self.device_choice, False, False, 0)
            self.scheme_choice = Gtk.ComboBoxText()
            self.scheme_choice.connect("changed", self.selected_scheme)
            self.controls.pack_start(self.scheme_choice, True, True, 0)
            self.load_button = Gtk.Button(label="刷新设备")
            self.load_button.connect("clicked", self.load)
            self.apply_button = Gtk.Button(label="应用")
            self.apply_button.get_style_context().add_class("suggested-action")
            self.apply_button.connect("clicked", self.apply)
            self.controls.pack_start(self.apply_button, False, False, 0)
            self.message = StatusLabel(scheme=True)
            self.message.set_text("Connecting to system DBus")
            self.compact_status = text("正在读取方案 · 切换会短暂中断输入")
            self.message.summary.connect("notify::label", lambda label, *_: self.compact_status.set_text(
                ("当前：" + {"xbox-compat": "Xbox 兼容", "dualsense-motion": "DualSense 体感"}.get(self.current, "自定义方案") + " · " if self.current_known else "")
                + (label.get_text() + " · " if not label.get_text().startswith("当前方案：") else "")
                + "切换会短暂中断输入"))
            self.pack_start(self.compact_status, False, False, 0)
            self.diagnostics = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
            self.diagnostics.pack_start(self.load_button, False, False, 0)
            self.diagnostics.pack_start(self.message, False, False, 0)
            self.identity_label = Gtk.Label(label="Composite/profile unknown until live discovery.", xalign=0)
            self.identity_label.set_line_wrap(True)
            self.identity_label.set_selectable(True)
            self.diagnostics.pack_start(details("设备身份", self.identity_label), False, False, 0)
            self.motion_label = Gtk.Label(label="Motion flag describes target protocol, not verified sensors or game support. No preference is saved.", xalign=0)
            self.motion_label.set_line_wrap(True)
            self.diagnostics.pack_start(details("协议说明", self.motion_label), False, False, 0)
            self.diagnostics.pack_start(text("Xbox 兼容不提供 SDL 体感。DualSense 支持体感协议。方案不会自动保存。"), False, False, 0)
            self.sensitivity()
            Gio.bus_get(Gio.BusType.SYSTEM, self.cancel, self.connected)

        def sensitivity(self) -> None:
            ready = self.bus is not None and not self.busy and not self.closed
            self.load_button.set_sensitive(ready)
            self.device_choice.set_sensitive(ready and bool(self.composites))
            self.scheme_choice.set_sensitive(ready and self.current_known and bool(self.schemes))
            self.apply_button.set_sensitive(ready and self.current_known and self.device_choice.get_active_id() in self.composites and self.scheme_choice.get_active_id() in self.schemes)

        def connected(self, source: object, result: Gio.AsyncResult, *_context: object) -> None:
            try:
                bus = Gio.bus_get_finish(result)
            except GLib.Error as error:
                if not self.closed:
                    self.message.set_text(f"System DBus unavailable: {error}. Inspection/calibration remain independent.")
                return
            if self.closed:
                return
            self.bus = bus
            for interface, signal in (("org.freedesktop.DBus.ObjectManager", "InterfacesAdded"), ("org.freedesktop.DBus.ObjectManager", "InterfacesRemoved"), ("org.freedesktop.DBus.Properties", "PropertiesChanged")):
                self.subscriptions.append(bus.signal_subscribe(self.SERVICE, interface, signal, None, None, Gio.DBusSignalFlags.NONE, self.changed))
            self.subscriptions.append(bus.signal_subscribe("org.freedesktop.DBus", "org.freedesktop.DBus", "NameOwnerChanged", "/org/freedesktop/DBus", self.SERVICE, Gio.DBusSignalFlags.NONE, self.owner_changed))
            self.load()

        def call(self, path: str, interface: str, method: str, parameters: GLib.Variant | None, reply_type: str, done: Callable[[tuple | None, str | None], None], timeout: int = 10000) -> None:
            if self.closed or self.bus is None:
                return
            generation = self.generation

            def finished(bus: Gio.DBusConnection, result: Gio.AsyncResult, *_context: object) -> None:
                reply, error = None, None
                try:
                    reply = bus.call_finish(result).unpack()
                except GLib.Error as failure:
                    error = str(failure)
                if not self.closed and generation == self.generation:
                    done(reply, error)

            self.bus.call(self.SERVICE, path, interface, method, parameters, GLib.VariantType.new(reply_type), Gio.DBusCallFlags.NONE, timeout, self.cancel, finished)

        def fail(self, error: str) -> None:
            self.busy = False
            self.current_known = False
            hint = ""
            if any(word in error for word in ("UnknownMethod", "UnknownInterface", "UnknownProperty")):
                hint = " Required existing target/identity API is unavailable; check InputPlumber version, then Load. Switching is disabled."
            self.message.set_text("InputPlumber: " + error + hint + " Authorization errors must be resolved through normal policy; no bypass is attempted.")
            self.sensitivity()

        def load(self, *_: object) -> None:
            if self.closed or self.bus is None:
                return
            self.generation += 1
            self.busy = True
            self.current_known = False
            self.current = "unknown"
            self.target_types.clear()
            self.schemes.clear()
            self.composites.clear()
            self.scheme_choice.remove_all()
            self.device_choice.remove_all()
            self.identity_label.set_text("Composite/profile unknown until fresh discovery.")
            self.message.set_text("Loading real schemes and live composites; no routing changes…")
            self.sensitivity()
            self.call(self.ROOT + "/Manager", "org.freedesktop.DBus.Properties", "Get", GLib.Variant("(ss)", (self.MANAGER, "SupportedTargetDeviceIds")), "(v)", self.listed)

        def listed(self, reply: tuple | None, error: str | None) -> None:
            if error:
                self.fail(error)
                return
            self.supported = set(reply[0])
            for scheme_id, display, target, motion in (("xbox-compat", "Xbox compatibility", "xb360", False), ("dualsense-motion", "DualSense motion", "ds5", True)):
                if target not in self.supported:
                    continue
                self.schemes[scheme_id] = (display, target, motion)
                capability = "motion-capable protocol" if motion else "no native SDL motion"
                self.scheme_choice.append(scheme_id, "Xbox 兼容" if not motion else "DualSense 体感")
            self.call(self.ROOT, "org.freedesktop.DBus.ObjectManager", "GetManagedObjects", None, "(a{oa{sa{sv}}})", self.discovered)

        @staticmethod
        def identity(properties: dict[str, object]) -> tuple[str, str, list[str]]:
            name, persistent, paths = properties.get("Name"), properties.get("PersistentId"), properties.get("SourceDevicePaths")
            if not isinstance(name, str) or not isinstance(persistent, str) or not persistent or not isinstance(paths, (list, tuple)) or not paths or any(not isinstance(path, str) or not path.startswith("/dev/") for path in paths) or len(set(paths)) != len(paths):
                raise ValueError("Composite lacks a complete valid Name/PersistentId/SourceDevicePaths identity; Load after upgrade/hotplug.")
            return name, persistent, list(paths)

        def discovered(self, reply: tuple | None, error: str | None) -> None:
            if error:
                self.fail(error)
                return
            preferred = []
            for path, interfaces in reply[0].items():
                properties = interfaces.get(self.COMPOSITE)
                if properties is None:
                    continue
                try:
                    name, persistent, paths = self.identity(properties)
                except ValueError:
                    continue
                self.composites[path] = properties
                self.device_choice.append(path, f"{name} · {path.rsplit('/', 1)[-1]}")
                if name.casefold() in ("aynlayout", "ayn layout", "ayn thor", "ayn thor lite") and "/dev/fastrpc-adsp" in paths and any(source.startswith(("/dev/input/event", "/dev/hidraw")) for source in paths):
                    preferred.append(path)
            self.busy = False
            self.message.set_text("Choose a composite and scheme, then Apply explicitly. No automatic switching or persistence.")
            self.sensitivity()
            if len(preferred) == 1:
                # Selecting a uniquely identified Thor only reads its current scheme.
                self.device_choice.set_active_id(preferred[0])
            elif not self.composites:
                self.message.set_text("No composites with valid live identity. Check InputPlumber version/permissions and Load; body capture remains independent.")
            else:
                self.message.set_text("Select the intended composite explicitly; no foreign controller is automatically chosen.")
            if not self.schemes:
                self.message.set_text("Neither xb360 nor ds5 is advertised by SupportedTargetDeviceIds. No available local presets; switching disabled.")

        def selected_device(self, *_: object) -> None:
            if self.busy or self.closed:
                return
            path = self.device_choice.get_active_id()
            self.current_known = False
            self.scheme_choice.set_active(-1)
            if path not in self.composites:
                self.sensitivity()
                return
            self.generation += 1
            self.busy = True
            name, persistent, paths = self.identity(self.composites[path])
            profile = self.composites[path].get("ProfileName") or "unknown"
            self.identity_label.set_text(f"Name: {name} · PersistentId: {persistent}\nSourceDevicePaths: {', '.join(paths)}\nProfile: {profile} (protocol selection does not change mappings/profile)")
            self.message.set_text("Reading live current scheme…")
            self.sensitivity()
            self.read_targets(path, self.composites[path], self.got_current)

        def read_targets(self, path: str, properties: dict[str, object], done: Callable[[list[str] | None, str | None], None], timeout: int = 10000) -> None:
            paths = properties.get("TargetDevices")
            if not isinstance(paths, (list, tuple)) or not paths or any(not isinstance(target, str) or not target.startswith(self.ROOT + "/devices/target/") for target in paths) or len(set(paths)) != len(paths):
                done(None, "Live TargetDevices is empty/invalid; cannot safely infer target types.")
                return

            def graph(reply: tuple | None, error: str | None) -> None:
                if error:
                    done(None, error)
                    return
                types = []
                objects = reply[0]
                for target_path in paths:
                    target_type = objects.get(target_path, {}).get(self.TARGET, {}).get("DeviceType")
                    if not isinstance(target_type, str) or target_type not in self.supported or target_type not in self.GAMEPADS | self.NON_GAMEPADS:
                        done(None, f"Unknown/missing target type at {target_path}; safe preservation impossible. Load after hotplug/version check.")
                        return
                    types.append(target_type)
                done(types, None)

            self.call(self.ROOT, "org.freedesktop.DBus.ObjectManager", "GetManagedObjects", None, "(a{oa{sa{sv}}})", graph, timeout=timeout)

        def got_current(self, types: list[str] | None, error: str | None) -> None:
            if error:
                self.fail(error)
                return
            self.target_types = types
            gamepads = [kind for kind in types if kind in self.GAMEPADS]
            self.current = next((scheme for scheme, (_, target, _) in self.schemes.items() if gamepads == [target]), "custom")
            self.current_known = len(gamepads) == 1
            self.busy = False
            display = self.schemes.get(self.current, (self.current, "", False))[0]
            self.message.set_text(f"Current: {display} ({self.current}) · types: {', '.join(types)}. " + ("Zero/multiple gamepads: switching disabled. " if not self.current_known else "") + "Explicit Apply recreates targets; not atomic, no rollback/persistence guarantee.")
            if self.current in self.schemes:
                self.scheme_choice.set_active_id(self.current)
            self.sensitivity()

        def selected_scheme(self, *_: object) -> None:
            selected = self.schemes.get(self.scheme_choice.get_active_id())
            if selected:
                display, target, motion = selected
                self.motion_label.set_text(f"Selected: {display} · target {target} · " + ("motion-capable protocol (e.g. real DualSense); sensors/game acceptance still require runtime verification." if motion else "Xbox-compatible protocol has no native SDL motion; body SSC is independent.") + " No automatic persistence.")
            self.sensitivity()

        def apply(self, *_: object) -> None:
            path, scheme = self.device_choice.get_active_id(), self.scheme_choice.get_active_id()
            if self.closed or self.busy or not self.current_known or path not in self.composites or scheme not in self.schemes:
                return
            self.generation += 1
            self.busy = True
            self.sensitivity()
            self.message.set_text("Refreshing complete identity immediately before explicit switch…")

            def fresh(reply: tuple | None, error: str | None) -> None:
                if error:
                    self.fail(error)
                    return
                try:
                    properties = reply[0]
                    name, persistent, paths = self.identity(properties)
                    old_name, old_persistent, old_paths = self.identity(self.composites[path])
                    if (name, persistent, sorted(paths)) != (old_name, old_persistent, sorted(old_paths)):
                        raise ValueError("Composite identity/source topology changed. Load and reselect the intended device before Apply; no switch sent.")
                except ValueError as failure:
                    self.fail(str(failure))
                    return
                self.read_targets(path, properties, lambda types, failure: resolved(properties, types, failure))

            def resolved(properties: dict[str, object], types: list[str] | None, error: str | None) -> None:
                if error:
                    self.fail(error)
                    return
                gamepads = [kind for kind in types if kind in self.GAMEPADS]
                if len(gamepads) != 1 or Counter(types) != Counter(self.target_types):
                    self.fail("Target layout changed or does not contain exactly one known gamepad. Load/reselect; no switch sent.")
                    return
                non_gamepads = [kind for kind in types if kind not in self.GAMEPADS]
                target = self.schemes[scheme][1]
                requested = [target if kind in self.GAMEPADS else kind for kind in types]

                def rechecked(reply: tuple | None, error: str | None) -> None:
                    if error:
                        self.fail(error)
                        return
                    latest = reply[0]
                    try:
                        old_name, old_id, old_paths = self.identity(properties)
                        name, persistent, sources = self.identity(latest)
                        if (name, persistent, sorted(sources)) != (old_name, old_id, sorted(old_paths)) or any(sorted(latest.get(key, [])) != sorted(properties.get(key, [])) for key in ("TargetDevices", "DbusDevices")) or latest.get("ProfileName") != properties.get("ProfileName"):
                            raise ValueError("Identity/targets/profile changed immediately before Apply. Load/reselect; no switch sent.")
                    except ValueError as failure:
                        self.fail(str(failure))
                        return
                    if gamepads == [target]:
                        self.composites[path] = latest
                        self.got_current(types, None)
                        self.message.set_text(self.message.get_text() + " Already selected; no SetTargetDevices call sent.")
                        return
                    self.message.set_text("Requesting target types: " + ", ".join(requested) + ". Existing targets may be recreated; no atomic/rollback guarantee.")

                    def queued(reply: tuple | None, error: str | None) -> None:
                        if error:
                            self.fail(error + " Outcome uncertain; Load to inspect actual targets. No automatic rollback.")
                            return
                        self.message.set_text("Request enqueued only; observing actual targets for up to 10 seconds…")
                        self.poll_targets(path, latest, target, non_gamepads, time.monotonic() + 10)

                    self.call(path, self.COMPOSITE, "SetTargetDevices", GLib.Variant("(as)", (requested,)), "()", queued)

                self.call(path, "org.freedesktop.DBus.Properties", "GetAll", GLib.Variant("(s)", (self.COMPOSITE,)), "(a{sv})", rechecked)

            self.call(path, "org.freedesktop.DBus.Properties", "GetAll", GLib.Variant("(s)", (self.COMPOSITE,)), "(a{sv})", fresh)

        def poll_targets(self, path: str, expected: dict[str, object], target: str, non_gamepads: list[str], deadline: float, matches: int = 0, last_error: str = "targets not yet observed") -> None:
            if self.closed:
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self.fail("Target observation timed out after 10 seconds: " + last_error + ". No success/rollback inferred; Load actual state before retrying.")
                return
            timeout = max(1, min(1000, int(remaining * 1000)))
            generation = self.generation

            def retry(error: str, next_matches: int = 0) -> None:
                def again() -> bool:
                    if not self.closed and generation == self.generation:
                        self.poll_targets(path, expected, target, non_gamepads, deadline, next_matches, error)
                    return False

                GLib.timeout_add(min(200, max(1, int((deadline - time.monotonic()) * 1000))), again)

            def properties_read(reply: tuple | None, error: str | None) -> None:
                if error:
                    retry(error)
                    return
                latest = reply[0]
                try:
                    name, persistent, sources = self.identity(latest)
                    old_name, old_id, old_sources = self.identity(expected)
                    if (name, persistent, sorted(sources)) != (old_name, old_id, sorted(old_sources)) or latest.get("ProfileName") != expected.get("ProfileName"):
                        raise ValueError("Composite identity/profile changed during observation. Outcome uncertain; Load; no rollback attempted.")
                except ValueError as failure:
                    self.fail(str(failure))
                    return

                def observed(types: list[str] | None, error: str | None) -> None:
                    if time.monotonic() >= deadline:
                        self.fail("10-second observation expired. Load live targets; no success or rollback inferred.")
                        return
                    if error:
                        retry(error)
                        return
                    gamepads = [kind for kind in types if kind in self.GAMEPADS]
                    preserved = Counter(kind for kind in types if kind not in self.GAMEPADS) == Counter(non_gamepads)
                    dbus_preserved = sorted(latest.get("DbusDevices", [])) == sorted(expected.get("DbusDevices", []))
                    if gamepads != [target] or not preserved or not dbus_preserved:
                        retry(f"observed {types}; non-gamepad preservation={preserved}; DBus preservation={dbus_preserved}")
                        return
                    if matches < 1:
                        retry("first matching live observation; verifying again", matches + 1)
                        return
                    self.composites[path] = latest
                    self.got_current(types, None)
                    self.message.set_text(self.message.get_text() + " Observed desired gamepad and preserved non-gamepad types twice; not a routing/rumble/motion verification.")
                    self.rescan()

                    def delayed_rescan() -> bool:
                        if not self.closed and generation == self.generation:
                            self.rescan()
                        return False

                    GLib.timeout_add(1200, delayed_rescan)

                self.read_targets(path, latest, observed, timeout=max(1, min(1000, int((deadline - time.monotonic()) * 1000))))

            self.call(path, "org.freedesktop.DBus.Properties", "GetAll", GLib.Variant("(s)", (self.COMPOSITE,)), "(a{sv})", properties_read, timeout=timeout)

        def changed(self, bus: Gio.DBusConnection, sender: str, path: str, interface: str, signal: str, parameters: GLib.Variant, *_context: object) -> None:
            if self.closed or self.busy:
                return
            values = parameters.unpack()
            if signal == "PropertiesChanged":
                relevant_target = any(path in properties.get("TargetDevices", []) for properties in self.composites.values())
                if values[0] == self.TARGET and relevant_target:
                    pass
                elif values[0] == self.COMPOSITE and path in self.composites:
                    names = set(values[1]) | set(values[2])
                    if not names.intersection({"PersistentId", "SourceDevicePaths", "Name", "TargetDevices", "DbusDevices", "ProfileName"}):
                        return
                else:
                    return
            elif self.COMPOSITE not in values[1] and not (signal == "InterfacesRemoved" and any(values[0] in properties.get("TargetDevices", []) for properties in self.composites.values())):
                return
            self.generation += 1
            self.current_known = False
            self.message.set_text("Composite/targets changed (hotplug or another client). Load fresh identity/current scheme before Apply.")
            self.sensitivity()

        def owner_changed(self, bus: Gio.DBusConnection, sender: str, path: str, interface: str, signal: str, parameters: GLib.Variant, *_context: object) -> None:
            if self.closed:
                return
            self.generation += 1
            self.busy = False
            self.current_known = False
            self.message.set_text("InputPlumber service owner changed. Load to rediscover; an in-flight switch cannot be assumed rolled back.")
            self.sensitivity()

        def close(self) -> None:
            self.closed = True
            self.generation += 1
            self.cancel.cancel()
            if self.bus:
                for subscription in self.subscriptions:
                    self.bus.signal_unsubscribe(subscription)
            self.subscriptions.clear()

    class Plot(Gtk.DrawingArea):
        def __init__(self) -> None:
            super().__init__()
            self.values: deque[Vector] = deque(maxlen=400)
            self.set_size_request(120, 70)
            self.connect("draw", self.draw)

        def draw(self, widget: object, cr: object) -> None:
            width, height = self.get_allocated_width(), self.get_allocated_height()
            style = self.get_style_context()
            found, background = style.lookup_color("theme_base_color")
            foreground = style.get_color(Gtk.StateFlags.NORMAL)
            cr.set_source_rgba(background.red, background.green, background.blue, 1) if found else cr.set_source_rgb(.12, .14, .17)
            cr.paint()
            cr.set_source_rgba(foreground.red, foreground.green, foreground.blue, .35)
            cr.move_to(0, height / 2)
            cr.line_to(width, height / 2)
            cr.stroke()
            scale = max(.05, max((abs(v) for row in self.values for v in row), default=1))
            dark = not found or (background.red + background.green + background.blue) / 3 < .5
            colors = ((1, .54, .50), (.43, .91, .72), (.58, .77, .99)) if dark else ((.70, .14, .09), (.02, .47, .34), (.11, .31, .85))
            cr.set_source_rgba(foreground.red, foreground.green, foreground.blue, 1)
            cr.set_font_size(10)
            cr.move_to(6, 12)
            cr.show_text(f"+/- {scale:.3g}")
            for axis, color in enumerate(colors):
                cr.set_source_rgb(*color)
                cr.set_dash(((), (6, 3), (2, 3))[axis])
                for index, row in enumerate(self.values):
                    x = index * width / 399
                    y = height / 2 - row[axis] / scale * height * .44
                    (cr.move_to if index == 0 else cr.line_to)(x, y)
                cr.stroke()

    class PoseIllustration(Gtk.DrawingArea):
        """Scalable lower-body diagrams; arrows describe physical up, not sensor axes."""

        def __init__(self) -> None:
            super().__init__()
            self.pose = 0
            self.set_size_request(156, 96)
            self.set_hexpand(False)
            self.set_tooltip_text("实线机身带按键；虚线表示转轴。箭头指向天花板。")
            self.connect("draw", self.draw)

        def draw(self, widget: object, cr: object) -> None:
            color = self.get_style_context().get_color(Gtk.StateFlags.NORMAL)
            cr.set_source_rgba(color.red, color.green, color.blue, color.alpha)
            cr.save()
            width, height = self.get_allocated_width(), self.get_allocated_height()
            cr.translate(width / 2, height / 2)
            scale = min(width / 260, height / 100)
            cr.scale(scale, scale)
            cr.set_line_width(2)
            if self.pose < 2:
                # Face/back views share the same lower body silhouette.
                cr.rectangle(-65, -25, 130, 50)
                cr.stroke()
                cr.set_dash([4, 3])
                cr.move_to(-60, -32)
                cr.line_to(60, -32)
                cr.stroke()
                cr.set_dash([])
                if self.pose == 0:
                    cr.rectangle(-28, -18, 56, 36)
                    cr.stroke()
                    for x in (-46, 46):
                        cr.arc(x, 0, 7, 0, math.tau)
                        cr.stroke()
                else:
                    for x in (-52, 52):
                        cr.arc(x, 14, 3, 0, math.tau)
                        cr.stroke()
                    cr.move_to(-24, 0)
                    cr.line_to(24, 0)
                    cr.stroke()
            else:
                # Front view rotated so the selected physical edge points up.
                angle = (0, math.pi, math.pi / 2, -math.pi / 2)[self.pose - 2]
                cr.save()
                cr.rotate(angle)
                cr.rectangle(-42, -27, 84, 54)
                cr.stroke()
                cr.rectangle(-20, -19, 40, 38)
                cr.stroke()
                for x in (-31, 31):
                    cr.arc(x, 0, 5, 0, math.tau)
                    cr.stroke()
                cr.set_dash([4, 3])
                cr.move_to(-38, -33)
                cr.line_to(38, -33)
                cr.stroke()
                cr.set_dash([])
                cr.restore()
            cr.move_to(94, 26)
            cr.line_to(94, -25)
            cr.line_to(86, -14)
            cr.move_to(94, -25)
            cr.line_to(102, -14)
            cr.stroke()
            cr.restore()

    class Window(Gtk.Window):
        def __init__(self) -> None:
            super().__init__(title="Thor Motion Lab")
            self.get_style_context().add_class("motion-lab")
            display = Gdk.Display.get_default()
            monitor = display.get_primary_monitor() if display else None
            if monitor is None and display and display.get_n_monitors():
                monitor = display.get_monitor(0)
            area = monitor.get_workarea() if monitor else None
            self.set_default_size(min(620, max(1, area.width - 24)) if area else 600,
                                  min(500, max(1, area.height - 48)) if area else 480)
            provider = Gtk.CssProvider()
            css = Path(__file__).with_name("motion-lab.css")
            if css.exists():
                provider.load_from_path(str(css))
                Gtk.StyleContext.add_provider_for_screen(self.get_screen(), provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
            self.running = False
            self.children: dict[str, Gio.Subprocess] = {}
            self.histories = {key: deque(maxlen=1500) for key in ("accel", "gyro")}
            self.plots: dict[str, Plot] = {}
            self.labels: dict[str, Gtk.Label] = {}
            self.bias: Vector | None = None
            self.candidate: Vector | None = None
            self.accel_saved: dict[str, object] | None = None
            self.accel_candidate: dict[str, object] | None = None
            self.faces: dict[str, Vector] = {}
            self.pose_faces: dict[str, str] = {}
            self.pose_index = 0
            self.calibrating_pose: str | None = None
            self.calibrating_face: str | None = None
            self.calibration_start: float | None = None
            self.pending_pose: str | None = None
            self.pose_armed_at = 0.0
            self.config = Path(os.environ.get("XDG_CONFIG_HOME", str(Path.home() / ".config"))) / "thorch-motion" / "calibration.json"
            metadata_status = "No saved calibration loaded."
            try:
                metadata = read_calibration(self.config)
                if "gyro_bias" in metadata:
                    self.bias = checked_vector(metadata["gyro_bias"], .15)
                if "accelerometer" in metadata:
                    self.accel_saved = metadata["accelerometer"]
                if self.bias is not None or self.accel_saved is not None:
                    metadata_status = "Saved metadata loaded for optional preview only; physical frame remains unknown."
            except (OSError, ValueError, RecursionError) as error:
                metadata_status = f"Saved metadata not loaded: {error}"
            self.gyro_button = Gtk.Button(label="采集 8 秒")
            self.gyro_button.connect("clicked", self.calibrate)
            self.gyro_button.set_sensitive(False)
            self.save_button = Gtk.Button(label="保存校准")
            self.save_button.connect("clicked", self.save)
            self.save_button.set_sensitive(False)
            self.status = StatusLabel()
            self.status.set_text("Body inspection stopped.")
            self.controller_status = text()
            self.scheme_panel = SchemePanel(self.scan)
            face_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
            guidance = text("只转动带按键的下半机身。上屏角度不代表姿势；可以合上转轴。")
            self.pose_chips: dict[str, Gtk.Label] = {}
            chips = Gtk.Box(spacing=6, homogeneous=True)
            for pose in POSES:
                self.pose_chips[pose] = text()
                self.pose_chips[pose].set_tooltip_text(pose)
                chips.pack_start(self.pose_chips[pose], True, True, 0)
            face_box.pack_start(chips, False, False, 0)
            pose_row = Gtk.Box(spacing=6)
            face_box.pack_start(pose_row, False, False, 0)
            pose_copy = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
            self.pose_title = text()
            self.pose_title.get_style_context().add_class("section-title")
            pose_copy.pack_start(self.pose_title, False, False, 0)
            self.pose_image = PoseIllustration()
            pose_row.pack_start(self.pose_image, False, False, 0)
            pose_row.pack_start(pose_copy, True, True, 0)
            self.pose_instruction = text()
            self.pose_instruction.set_max_width_chars(32)
            pose_copy.pack_start(self.pose_instruction, False, False, 0)
            self.face_live = text("请先开始采集，再按图放稳下半机身。")
            self.face_live.set_max_width_chars(32)
            pose_copy.pack_start(self.face_live, False, False, 0)
            face_controls = Gtk.Box(spacing=6)
            face_box.pack_start(face_controls, False, False, 0)
            self.pose_previous = Gtk.Button(label="上一步")
            self.pose_previous.connect("clicked", self.change_pose, -1)
            face_controls.pack_start(self.pose_previous, False, False, 0)
            collect = Gtk.Button(label="采集此姿势 · 8 秒")
            self.collect_button = collect
            collect.set_sensitive(False)
            collect.connect("clicked", self.collect_face)
            face_controls.pack_start(collect, True, True, 0)
            self.pose_next = Gtk.Button(label="下一步")
            self.pose_next.connect("clicked", self.change_pose, 1)
            face_controls.pack_start(self.pose_next, False, False, 0)
            reset = Gtk.Button(label="重置六面")
            self.reset_button = reset
            reset.connect("clicked", self.reset_faces)
            self.face_progress = text()
            self.face_diagnostics = text()
            self.update_faces()
            self.preview_toggle = Gtk.CheckButton(label="本地校正预览")
            self.preview_toggle.set_no_show_all(True)
            self.preview_toggle.set_sensitive(self.bias is not None or self.accel_saved is not None)
            self.preview_label = text()
            self.preview_label.set_no_show_all(True)
            for key in ("accel", "gyro", "sdl_accel", "sdl_gyro"):
                self.labels[key] = text("X —   Y —   Z —")
                self.plots[key] = Plot()
                self.plots[key].set_hexpand(True)
            # Callback-owned widgets keep their original data and action semantics.
            root = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
            root.set_border_width(6)
            self.add(root)
            header = Gtk.HeaderBar(title="体感工具", show_close_button=True)
            self.activity = Gtk.Label(label="已停止")
            header.pack_start(self.activity)
            more = Gtk.MenuButton(label="更多")
            popover = Gtk.Popover.new(more)
            diagnostics = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
            diagnostics.set_border_width(8)
            diagnostic_text = text(self.status.get_text())
            diagnostic_text.set_max_width_chars(45)
            diagnostic_text.set_selectable(True)
            self.status.full.connect("notify::label", lambda label, *_: diagnostic_text.set_text(label.get_text()))
            diagnostic_scroll = Gtk.ScrolledWindow()
            diagnostic_scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
            diagnostic_scroll.set_size_request(320, 260)
            diagnostic_content = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
            diagnostic_scroll.add(diagnostic_content)
            diagnostics.pack_start(diagnostic_scroll, True, True, 0)
            diagnostic_content.pack_start(details("方案与设备", self.scheme_panel.diagnostics), False, False, 0)
            diagnostic_content.pack_start(details("采集与校准诊断", diagnostic_text), False, False, 0)
            diagnostic_content.pack_start(details("姿势与通道记录", self.face_diagnostics), False, False, 0)
            diagnostic_content.pack_start(guidance, False, False, 0)
            diagnostic_content.pack_start(reset, False, False, 0)
            popover.add(diagnostics)
            more.set_popover(popover)
            header.pack_end(more)
            self.set_titlebar(header)
            self.pages = Gtk.Stack()
            self.pages.set_homogeneous(False)
            self.pages.set_transition_type(Gtk.StackTransitionType.NONE)
            root.pack_start(self.pages, True, True, 0)

            def move(widget: Gtk.Widget, destination: Gtk.Box) -> None:
                parent = widget.get_parent()
                if parent:
                    parent.remove(widget)
                destination.pack_start(widget, False, False, 0)

            def page(name: str, title: str) -> Gtk.Box:
                content = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
                pane = Gtk.ScrolledWindow()
                pane.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
                pane.add(content)
                self.pages.add_titled(pane, name, title)
                return content

            live_page = page("overview", "总览")
            move(self.scheme_panel, live_page)
            source_row = Gtk.Box(spacing=6)
            self.sources = Gtk.Stack()
            self.sources.set_homogeneous(False)
            source_switcher = Gtk.StackSwitcher(stack=self.sources)
            source_row.pack_start(source_switcher, False, False, 0)
            self.source_caption = text("原始 · 自动量程")
            self.source_caption.set_max_width_chars(26)
            source_row.pack_end(self.source_caption, False, False, 0)
            self.capture_button = Gtk.Button(label="开始采集")
            self.capture_button.connect("clicked", lambda *_: self.stop() if self.running else self.start())
            self.scheme_panel.controls.pack_end(self.capture_button, False, False, 0)
            live_page.pack_start(source_row, False, False, 0)
            live_page.pack_start(self.sources, False, False, 0)
            self.rate_labels: dict[str, Gtk.Label] = {}
            self.sdl_read_times: dict[str, float] = {}
            self.sdl_names: dict[str, str] = {}
            for prefix, name, title in (("", "body", "机身 SSC"), ("sdl_", "controller", "手柄 SDL")):
                source_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
                self.sources.add_titled(source_box, name, title)
                for sensor, unit, caption in (("accel", "m/s²", "加速度"), ("gyro", "rad/s", "角速度")):
                    key = prefix + sensor
                    card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
                    metric = Gtk.Box(spacing=6)
                    metric.pack_start(text(f"{caption} {unit}"), False, False, 0)
                    self.labels[key].get_style_context().add_class("numeric")
                    self.rate_labels[key] = text("尚无数据")
                    self.rate_labels[key].set_max_width_chars(40)
                    metric.pack_end(self.rate_labels[key], False, False, 0)
                    card.pack_start(metric, False, False, 0)
                    move(self.labels[key], card)
                    move(self.plots[key], card)
                    source_box.pack_start(card, False, False, 0)
                if not prefix:
                    self.preview_toggle.set_label("本地预览")
                    move(self.preview_label, source_box)
                else:
                    diagnostic_content.pack_start(details("手柄详情", self.controller_status), False, False, 0)
                    scan_button = Gtk.Button(label="重新扫描手柄")
                    scan_button.connect("clicked", self.scan)
                    diagnostic_content.pack_start(scan_button, False, False, 0)
            diagnostic_content.pack_start(details("数据说明", text("X 实线、Y 长虚线、Z 点划线。报告通道的物理朝向未知。\nSSC 显示主机接收率。SDL 显示主机读取，样本新鲜度未知。\n横轴为最近采样点，各图独立自动量程。Xbox 兼容不提供 SDL 体感。")), False, False, 0)
            quick_actions = Gtk.Box(spacing=6, homogeneous=True)
            quick_actions.pack_start(self.preview_toggle, True, True, 0)
            for title, target in (("静置校准", "gyro"), ("六面校准", "faces")):
                action = Gtk.Button(label=title)
                action.connect("clicked", lambda _button, name=target: self.open_calibration(name))
                quick_actions.pack_start(action, True, True, 0)
            live_page.pack_start(quick_actions, False, False, 0)
            self.overview_result = text()
            self.overview_result.set_no_show_all(True)
            live_page.pack_start(self.overview_result, False, False, 0)

            calibration_page = page("calibration", "校准")
            self.calibration_pages = Gtk.Stack()
            self.calibration_pages.set_homogeneous(False)
            calibration_controls = Gtk.Box(spacing=6)
            back = Gtk.Button(label="返回总览")
            back.connect("clicked", lambda *_: self.pages.set_visible_child_name("overview"))
            calibration_controls.pack_start(back, False, False, 0)
            calibration_controls.pack_start(Gtk.StackSwitcher(stack=self.calibration_pages), True, True, 0)
            self.calibration_capture = Gtk.Button(label="开始采集")
            self.calibration_capture.connect("clicked", lambda *_: self.stop() if self.running else self.start())
            calibration_controls.pack_end(self.calibration_capture, False, False, 0)
            calibration_page.pack_start(calibration_controls, False, False, 0)
            calibration_page.pack_start(self.calibration_pages, False, False, 0)
            stationary = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
            self.calibration_pages.add_titled(stationary, "gyro", "静置陀螺")
            stationary.pack_start(text("放稳机身，保持静止 8 秒。"), False, False, 0)
            self.gyro_button.set_label("采集 8 秒")
            move(self.gyro_button, stationary)
            self.calibration_pages.add_titled(face_box, "faces", "六面加速度")
            self.progress = Gtk.ProgressBar(show_text=True)
            self.progress.set_text("等待采集")
            calibration_page.pack_start(self.progress, False, False, 0)
            move(self.status.summary, calibration_page)
            self.save_button.set_label("保存校准")
            self.save_button.get_style_context().add_class("suggested-action")
            save_row = Gtk.Box(spacing=6)
            move(self.save_button, save_row)
            save_row.pack_start(text("仅保存本机系数 · 不修改其他应用"), False, False, 0)
            calibration_page.pack_start(save_row, False, False, 0)
            diagnostic_content.pack_start(details("校准条件与文件", text("每次采集 8 秒，两传感器各需至少 400 个连续样本。\n采样间隔 ≤0.3 s，首尾覆盖与延迟 ≤0.5 s。\n陀螺每轴标准差 ≤0.015 rad/s，每个样本模长 ≤0.15 rad/s。\n加速度每轴标准差 ≤0.12 m/s²。六姿势原始模长 6–14 m/s²；静止陀螺校准仍要求重力模长 9.0–10.6。\n球面偏移模长 ≤3 m/s²，共同比例 0.9–1.1。无需横向读数归零。\n拟合模长残差 ≤0.15 m/s²，留一姿势误差 ≤0.25 m/s²；检查不同方向及相反方向覆盖。\n慢速恒定旋转仍需操作人员排除，物理朝向尚未验证。\n" + str(self.config))), False, False, 0)
            diagnostic_content.pack_start(details("已保存系数状态", text(metadata_status)), False, False, 0)
            self.overview_status = text()
            self.overview_status.set_no_show_all(True)
            live_page.pack_start(self.overview_status, False, False, 0)
            self.status.summary.connect("notify::label", lambda label, *_: self.overview_status.set_text(label.get_text()))
            diagnostics.show_all()
            self.pages.set_visible_child_name("overview")
            try:
                self.sdl: SDL | None = SDL()
            except (OSError, AttributeError, RuntimeError) as error:
                self.sdl = None
                self.controller_status.set_text(f"SDL unavailable (requires SDL2 ≥ 2.0.14 sensor API): {error}")
            self.scan()
            self.connect("delete-event", self.shutdown)
            GLib.timeout_add(50, self.tick)

        def open_calibration(self, name: str) -> None:
            self.calibration_pages.set_visible_child_name(name)
            self.pages.set_visible_child_name("calibration")

        def scan(self, *_: object) -> None:
            if self.sdl:
                self.sdl_read_times.clear()
                self.sdl_names.clear()
                self.controller_status.set_text(self.sdl.scan())

        def start(self, *_: object) -> None:
            if self.running:
                return
            self.running = True
            for values in self.histories.values():
                values.clear()
            self.status.set_text("Inspecting. Private CLI requests 100 Hz; delivery rate is measured, not assumed.")
            for sensor in ("accel", "gyro"):
                self.spawn(sensor)

        def spawn(self, sensor: str) -> bool:
            if not self.running or sensor in self.children:
                return False
            try:
                child = Gio.Subprocess.new([options.capture, "--sensor", "accelerometer" if sensor == "accel" else "gyroscope", "--sample-rate", "100", "--timeout", "20"], Gio.SubprocessFlags.STDOUT_PIPE | Gio.SubprocessFlags.STDERR_MERGE)
            except GLib.Error as error:
                self.status.set_text(f"Capture unavailable: {error}; retry in 3 seconds")
                GLib.timeout_add_seconds(3, self.spawn, sensor)
                return False
            self.children[sensor] = child
            stream = Gio.DataInputStream.new(child.get_stdout_pipe())
            self.read_line(sensor, child, stream)
            child.wait_async(None, self.exited, sensor)
            GLib.timeout_add_seconds(25, self.watchdog, sensor, child)
            return False

        def watchdog(self, sensor: str, child: Gio.Subprocess) -> bool:
            if self.children.get(sensor) is child:
                child.force_exit()
            return False

        def read_line(self, sensor: str, child: Gio.Subprocess, stream: Gio.DataInputStream) -> None:
            stream.read_line_async(GLib.PRIORITY_DEFAULT, None, self.line, (sensor, child))

        def line(self, stream: Gio.DataInputStream, result: Gio.AsyncResult, context: tuple[str, Gio.Subprocess]) -> None:
            sensor, child = context
            try:
                raw, _ = stream.read_line_finish(result)
            except GLib.Error:
                return
            if raw is None or not self.running or self.children.get(sensor) is not child:
                return
            text = raw.decode(errors="replace")
            match = SAMPLE.search(text)
            if match:
                vector = tuple(float(match[i]) for i in (1, 2, 3))
                if all(math.isfinite(value) for value in vector):
                    self.histories[sensor].append((int(match[4]) / 1e6, time.monotonic(), vector))
                    self.show_vector(sensor, vector)
            elif "fail" in text.lower() or "unable" in text.lower() or "error" in text.lower():
                self.status.set_text(text[:400])
            self.read_line(sensor, child, stream)

        def exited(self, child: Gio.Subprocess, result: Gio.AsyncResult, sensor: str) -> None:
            child.wait_finish(result)
            if self.children.get(sensor) is child:
                del self.children[sensor]
                if self.running:
                    if not child.get_successful():
                        self.status.set_text(f"{sensor} capture exited unsuccessfully. Retrying in 3 seconds; existing service must already be active.")
                    GLib.timeout_add_seconds(3, self.spawn, sensor)

        def show_vector(self, key: str, vector: Vector) -> None:
            self.labels[key].set_text("X {:+.4f}   Y {:+.4f}   Z {:+.4f}".format(*vector))
            self.plots[key].values.append(vector)

        def tick(self) -> bool:
            now = time.monotonic()
            collecting = self.calibration_start is not None or self.pending_pose is not None
            self.capture_button.set_label("停止采集" if self.running else "开始采集")
            self.calibration_capture.set_label("停止采集" if self.running else "开始采集")
            selected_device = self.scheme_panel.device_choice.get_active_id()
            self.scheme_panel.device_choice.set_visible(len(self.scheme_panel.composites) != 1 or selected_device not in self.scheme_panel.composites)
            self.gyro_button.set_sensitive(self.running and not collecting)
            self.collect_button.set_sensitive(self.running and self.calibration_start is None)
            self.collect_button.set_label("取消等待" if self.pending_pose is not None else "采集此姿势")
            self.pose_previous.set_sensitive(not collecting and self.pose_index > 0)
            self.pose_next.set_sensitive(not collecting and self.pose_index < len(POSES) - 1)
            self.reset_button.set_sensitive(not collecting)
            self.preview_toggle.set_sensitive(any(value is not None for value in (self.bias, self.candidate, self.accel_saved, self.accel_candidate)))
            has_pending = self.candidate is not None or self.accel_candidate is not None
            warning = bool(self.faces or has_pending or collecting)
            for button in (self.capture_button, self.calibration_capture):
                button.set_label("停止并清除" if self.running and warning else "停止采集" if self.running else "开始采集")
                button.set_tooltip_text("停止会清除未保存校准结果和六面会话" if warning else "仅控制本工具采集，不修改服务")
            body_view = self.sources.get_visible_child_name() == "body"
            self.preview_toggle.set_visible(body_view)
            self.preview_label.set_visible(body_view and self.preview_toggle.get_active())
            if body_view:
                self.source_caption.set_text("原始 · 自动量程")
            else:
                names = list(dict.fromkeys(self.sdl_names.values()))
                name = names[-1] if names else "手柄未提供数据"
                self.source_caption.set_text(name[:14] + " · 新鲜度未知")
                self.source_caption.set_tooltip_text(name)
            if warning:
                self.overview_result.set_text("校准结果待保存 · 停止将清除未保存结果" if has_pending else "六面会话进行中 · 停止将清除本次结果")
            elif self.bias is not None or self.accel_saved is not None:
                self.overview_result.set_text("校准系数已保存 · 本地预览可用")
            else:
                self.overview_result.set_text("")
            self.overview_result.set_visible(bool(self.overview_result.get_text()))
            status_value = self.status.get_text().lower()
            self.overview_status.set_visible(any(word in status_value for word in ("error", "failed", "unavailable", "rejected", "unsuccessfully", "unable")))
            self.activity.set_text("采集中" if self.running else "已停止")
            sdl_error = False
            if self.running and self.sdl:
                try:
                    for name, sensor, vector in self.sdl.poll():
                        self.show_vector("sdl_" + sensor, vector)
                        self.sdl_read_times[sensor] = now
                        self.sdl_names[sensor] = name
                        self.labels["sdl_" + sensor].set_tooltip_text(name + ": host polling; channels shown from last controller with sensor")
                except RuntimeError as error:
                    self.controller_status.set_text(str(error))
                    sdl_error = True
            for sensor, history in self.histories.items():
                if self.running and (not history or now - history[-1][1] > 2):
                    self.labels[sensor].set_text("X —   Y —   Z —")
                    self.rate_labels[sensor].set_text("中断 · 历史曲线" if history else "等待数据 · 历史曲线" if self.plots[sensor].values else "等待数据")
                    self.activity.set_text("数据中断")
                elif history:
                    recent = [item for item in history if now - item[1] < 2]
                    hz = (len(recent) - 1) / (recent[-1][0] - recent[0][0]) if len(recent) > 1 and recent[-1][0] > recent[0][0] else 0
                    self.labels[sensor].set_tooltip_text(f"Measured host receipt rate {hz:.1f} Hz; age {now - history[-1][1]:.2f}s")
                    self.rate_labels[sensor].set_text(f"{hz:.1f} Hz · {now - history[-1][1]:.2f} s 前" if self.running else "已停止 · 历史曲线")
                else:
                    self.rate_labels[sensor].set_text("已停止")
                if not self.running:
                    self.labels[sensor].set_text("X —   Y —   Z —")
            for sensor in ("accel", "gyro"):
                key = "sdl_" + sensor
                age = now - self.sdl_read_times.get(sensor, float("-inf"))
                if sdl_error:
                    self.labels[key].set_text("X —   Y —   Z —")
                    self.rate_labels[key].set_text("读取失败 · 历史曲线 · 详情见更多")
                elif not self.running or age > 2:
                    self.labels[key].set_text("X —   Y —   Z —")
                    self.rate_labels[key].set_text("已停止 · 历史曲线" if not self.running else "SDL 不可用 · 详情见更多" if self.sdl is None else "无可用传感器数据 · 详情见更多")
                else:
                    self.rate_labels[key].set_text(f"读取 {age:.2f} s 前")
            if collecting and self.calibrating_pose is not None:
                self.face_live.set_text("正在采集，请保持此姿势不动。")
            elif self.running:
                face, hint = pose_face_hint(self.histories["accel"], now)
                problem = pose_mapping_problem(self.pose_faces, POSES[self.pose_index], face) if face else None
                self.face_live.set_text(problem or hint)
            else:
                self.face_live.set_text("请先开始采集，再按图放稳下半机身。")
            if self.preview_toggle.get_active():
                parts = []
                bias = self.candidate if self.candidate is not None else self.bias
                accel_metadata = self.accel_candidate if self.accel_candidate is not None else self.accel_saved
                for sensor in ("accel", "gyro"):
                    history = self.histories[sensor]
                    if not self.running or not history or now - history[-1][1] >= .5:
                        parts.append(f"{'加速度' if sensor == 'accel' else '角速度'}：暂无新数据")
                        continue
                    raw = history[-1][2]
                    if sensor == "gyro" and bias is not None:
                        corrected = tuple(raw[i] - bias[i] for i in range(3))
                    elif sensor == "accel" and accel_metadata is not None:
                        offset = checked_vector(accel_metadata["offset"], 3.0 if accel_metadata.get("model") == "sphere" else .6)
                        scale = checked_vector(accel_metadata["scale"], 1.1)
                        corrected = tuple((raw[i] - offset[i]) * scale[i] for i in range(3))
                    else:
                        parts.append(f"{'加速度' if sensor == 'accel' else '角速度'}：暂无系数")
                        continue
                    caption = "加速度 m/s²" if sensor == "accel" else "角速度 rad/s"
                    unsaved = self.accel_candidate is not None if sensor == "accel" else self.candidate is not None
                    parts.append(caption + (" · 未保存结果：" if unsaved else " · 已保存系数：") + ", ".join(f"{v:+.4f}" for v in corrected))
                self.preview_label.set_text("校正预览\n" + "\n".join(parts))
            else:
                self.preview_label.set_text("预览已关闭" if self.preview_toggle.get_sensitive() else "暂无校正系数")
            if self.pending_pose is not None:
                self.progress.set_fraction(0)
                self.progress.set_text("等待放稳 · 可取消")
                self.activity.set_text("等待放稳")
                self.try_begin_pose(now)
            elif self.calibration_start is not None:
                elapsed = min(8, now - self.calibration_start)
                self.progress.set_fraction(elapsed / 8)
                self.progress.set_text(f"剩余 {8 - elapsed:.1f} 秒")
                self.activity.set_text(f"采集 · {8 - elapsed:.1f} 秒")
                if now - self.calibration_start >= 8:
                    self.finish_calibration(now)
                else:
                    target = self.calibrating_pose or "BODY gyro"
                    self.status.set_text(f"Keep Thor completely still · {target} · {8 - (now - self.calibration_start):.1f}s remaining")
            else:
                self.progress.set_fraction(0)
                self.progress.set_text("结果待保存" if self.candidate is not None or self.accel_candidate is not None else "等待采集")
            for plot in self.plots.values():
                plot.queue_draw()
            return True

        def calibrate(self, *_: object) -> None:
            if self.pending_pose is not None:
                self.status.set_text("请先取消姿势等待，再开始陀螺校准。")
                return
            if not self.running:
                self.status.set_text("Start inspection first, then place Thor on a stable surface. Calibration is for BODY gyro only.")
                return
            if self.calibration_start is not None:
                self.status.set_text("An 8-second collection is already running. Wait or Stop to cancel.")
                return
            self.candidate = None
            self.calibrating_face = None
            self.calibration_start = time.monotonic()
            self.update_save()

        def update_save(self) -> None:
            self.save_button.set_sensitive(self.calibration_start is None and self.pending_pose is None and (self.candidate is not None or self.accel_candidate is not None))

        def update_faces(self) -> None:
            self.pose_faces = {pose: face for pose, face in self.pose_faces.items() if face in self.faces}
            for index, (pose, chip) in enumerate(self.pose_chips.items()):
                caption = ("屏上", "屏下", "上边", "下边", "左边", "右边")[index]
                chip.set_text(("› " if index == self.pose_index else "") + caption + (" ✓" if pose in self.pose_faces else " ·"))
                chip.set_tooltip_text(pose + (" · 已完成" if pose in self.pose_faces else " · 待采集"))
            self.face_progress.set_text(f"已完成 {len(self.pose_faces)}/6")
            self.face_diagnostics.set_text("\n".join(pose + ": " + self.pose_faces.get(pose, "尚未采集") for pose in POSES) + "\n这是本次测量关联，未验证物理安装坐标。")
            self.pose_title.set_text(f"{self.pose_index + 1}/6 · {POSES[self.pose_index]} · 完成 {len(self.pose_faces)}/6")
            self.pose_instruction.set_text(POSE_GUIDANCE[self.pose_index])
            self.pose_image.pose = self.pose_index
            self.pose_image.queue_draw()

        def change_pose(self, button: Gtk.Button, direction: int) -> None:
            if self.calibration_start is not None or self.pending_pose is not None:
                return
            self.pose_index = min(len(POSES) - 1, max(0, self.pose_index + direction))
            self.update_faces()

        def collect_face(self, *_: object) -> None:
            if self.pending_pose is not None:
                self.pending_pose = None
                self.update_save()
                self.status.set_text("已取消等待，已有结果保持不变。")
                return
            if self.calibration_start is not None:
                self.status.set_text("Start inspection first; wait for any current collection to finish.")
                return
            if not self.running:
                self.start()
            self.pending_pose = POSES[self.pose_index]
            self.pose_armed_at = time.monotonic()
            self.update_save()
            self.status.set_text("请按图摆放，放稳后自动开始采集，无需再次点击。")

        def try_begin_pose(self, now: float) -> None:
            pose = self.pending_pose
            if pose is None:
                return
            if now - self.pose_armed_at < 1.3:
                return
            recent = deque(row for row in self.histories["accel"] if row[1] >= self.pose_armed_at)
            face, hint = pose_face_hint(recent, now)
            if face is None:
                self.status.set_text("等待放稳：" + hint)
                return
            gyro = [row for row in self.histories["gyro"] if max(self.pose_armed_at, now-1.2) <= row[1] <= now]
            if (len(gyro) < 50 or gyro[-1][1]-gyro[0][1] < 1 or now-gyro[-1][1] > .15
                or any(abs(row[1]-row[0]) > .5 for row in gyro)
                or any(b[1]-a[1] > .3 or not 0 < b[0]-a[0] <= .3 for a, b in zip(gyro, gyro[1:]))
                or max(statistics.pstdev(row[2][i] for row in gyro) for i in range(3)) > .015
                or any(math.sqrt(sum(v*v for v in row[2])) > .15 for row in gyro)):
                self.status.set_text("等待放稳：陀螺仪尚未稳定或数据未恢复。")
                return
            problem = pose_mapping_problem(self.pose_faces, pose, face)
            if problem:
                self.status.set_text(problem)
                return
            old_face = self.pose_faces.pop(pose, None)
            if old_face is not None:
                self.faces.pop(old_face, None)
            self.calibrating_pose = pose
            self.calibrating_face = face
            self.pending_pose = None
            self.faces.pop(face, None)
            self.accel_candidate = None
            self.calibration_start = time.monotonic()
            self.update_faces()
            self.update_save()

        def reset_faces(self, *_: object) -> None:
            if self.calibration_start is not None or self.pending_pose is not None:
                self.status.set_text("Stop or finish the current collection before resetting faces.")
                return
            self.faces.clear()
            self.pose_faces.clear()
            self.calibrating_pose = None
            self.pose_index = 0
            self.accel_candidate = None
            self.update_faces()
            self.update_save()
            self.status.set_text("Six-face session reset; saved metadata and gyro candidate remain unchanged.")

        def finish_calibration(self, now: float) -> None:
            start = self.calibration_start
            self.calibration_start = None
            face = self.calibrating_face
            self.calibrating_face = None
            pose = self.calibrating_pose
            self.calibrating_pose = None
            self.update_save()
            if start is None:
                return
            rows = {key: [row for row in history if row[1] >= start and row[0] >= start] for key, history in self.histories.items()}
            for key, data in rows.items():
                if len(data) < 400 or data[0][0] - start > .5 or now - data[-1][0] > .5 or any(abs(row[1] - row[0]) > .5 for row in data) or any(b[1] - a[1] > .3 or not 0 < b[0] - a[0] <= .3 for a, b in zip(data, data[1:])):
                    self.status.set_text(f"Calibration rejected: insufficient continuous {key} data. Wait for live capture and retry.")
                    return
            gyro = [row[2] for row in rows["gyro"]]
            accel = [row[2] for row in rows["accel"]]
            bias = tuple(statistics.mean(v[i] for v in gyro) for i in range(3))
            gyro_std = max(statistics.pstdev(v[i] for v in gyro) for i in range(3))
            accel_std = max(statistics.pstdev(v[i] for v in accel) for i in range(3))
            norms = [math.sqrt(sum(x*x for x in v)) for v in accel]
            gravity_ok = (all(6.0 < value < 14.0 for value in norms) if face is not None
                          else 9.0 < statistics.mean(norms) < 10.6)
            if gyro_std > .015 or max(math.sqrt(sum(x*x for x in v)) for v in gyro) > .15 or accel_std > .12 or not gravity_ok:
                self.status.set_text("Calibration rejected: motion or invalid gravity detected. Hold still; slow constant rotation cannot be conclusively excluded by this heuristic.")
                return
            if face is not None:
                axis = "XYZ".index(face[1])
                sign = 1 if face[0] == "+" else -1
                mean = tuple(statistics.mean(v[i] for v in accel) for i in range(3))
                if any(sign*v[axis] < .65*norm for v, norm in zip(accel, norms)):
                    self.status.set_text("采集中姿势方向改变，请放稳后重新采集。")
                    return
                if pose not in POSES:
                    self.status.set_text("本次姿势记录已失效，请重新选择姿势并采集。")
                    return
                problem = pose_mapping_problem(self.pose_faces, pose, face)
                if problem:
                    self.status.set_text(problem)
                    return
                self.faces[face] = mean
                self.pose_faces[pose] = face
                self.update_faces()
                if len(self.faces) == 6:
                    if set(self.pose_faces) != set(POSES) or set(self.pose_faces.values()) != set(FACES) or any(pose_mapping_problem(self.pose_faces, name, internal) for name, internal in self.pose_faces.items()):
                        self.status.set_text("六个姿势的记录不一致，请重置后重新采集。")
                        return
                    self.derive_accelerometer()
                    if self.accel_candidate is not None:
                        self.accel_candidate["user_pose_reported_channels"] = dict(self.pose_faces)
                        self.accel_candidate["pose_mapping_validation"] = "measured-unique-six-signed-channels-with-three-opposite-pairs-not-mount-verification"
                        self.status.set_text("球面拟合与留一姿势检查通过，可保存候选并预览。仍需额外姿势验证。")
                else:
                    self.status.set_text(f"{pose}已完成，共 {len(self.pose_faces)}/6。请点击下一步，按图摆放后再采集。")
                return
            self.candidate = bias
            self.update_save()
            self.status.set_text("Stationary window accepted. BODY gyro bias rad/s: " + ", ".join(f"{v:.6f}" for v in bias) + ". Save explicitly; no automatic application.")

        def derive_accelerometer(self) -> None:
            self.accel_candidate = None
            try:
                self.accel_candidate = fit_sphere([self.faces[face] for face in FACES])
            except ValueError as error:
                self.status.set_text(str(error))
                self.update_save()
                return
            self.accel_candidate.update({"face_means_m_s2": dict(self.faces), "window_seconds": 8})
            self.update_save()
            self.status.set_text("球面模型数值检查通过，请手动保存；尚未独立验证安装方向。")

        def save(self, *_: object) -> None:
            if self.pending_pose is not None or self.calibration_start is not None or (self.candidate is None and self.accel_candidate is None):
                return
            temporary: str | None = None
            try:
                # Re-read at the explicit save action: keep unrelated/other-sensor fields.
                payload = read_calibration(self.config)
                payload.update({"schema": 1, "source": "thor-body-private-ssc", "frame": "unknown", "timestamp_basis": "host-monotonic-receipt-not-dsp", "saved_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "automatic_apply": False})
                if self.candidate is not None:
                    payload.update({"units": "rad/s", "gyro_bias": self.candidate, "operation": "corrected = raw - gyro_bias"})
                if self.accel_candidate is not None:
                    existing_accel = payload.get("accelerometer", {})
                    payload["accelerometer"] = {**existing_accel, **self.accel_candidate}
                    payload["accelerometer_six_face"] = "numerically-accepted-physical-validation-required"
                self.config.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
                with tempfile.NamedTemporaryFile(mode="w", dir=self.config.parent, prefix=".calibration-", delete=False) as file:
                    temporary = file.name
                    json.dump(payload, file, indent=2, allow_nan=False)
                    file.write("\n")
                    file.flush()
                    os.fsync(file.fileno())
                os.replace(temporary, self.config)
                temporary = None
                directory = os.open(self.config.parent, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
                self.bias = checked_vector(payload["gyro_bias"], .15) if "gyro_bias" in payload else None
                self.accel_saved = payload.get("accelerometer")
                self.candidate = None
                self.accel_candidate = None
                self.update_save()
                self.status.set_text(f"Saved user-local coefficients: {self.config}. Raw displays remain unchanged.")
            except (OSError, ValueError, RecursionError) as error:
                self.status.set_text(f"Save failed: {error}")
            finally:
                if temporary is not None:
                    Path(temporary).unlink(missing_ok=True)

        def stop(self, *_: object) -> None:
            self.pending_pose = None
            self.running = False
            self.calibration_start = None
            self.calibrating_face = None
            self.calibrating_pose = None
            self.candidate = None
            self.accel_candidate = None
            self.faces.clear()
            self.pose_faces.clear()
            self.pose_index = 0
            self.update_faces()
            self.save_button.set_sensitive(False)
            for child in self.children.values():
                child.force_exit()
            self.children.clear()
            self.status.set_text("Stopped; capture children terminated. No services changed.")

        def shutdown(self, *_: object) -> bool:
            self.scheme_panel.close()
            self.stop()
            if self.sdl:
                self.sdl.close()
            Gtk.main_quit()
            return False

    window = Window()
    window.show_all()
    Gtk.main()


if __name__ == "__main__":
    main()
