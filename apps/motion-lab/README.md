# Thor Motion Lab

## Sphere accelerometer calibration

The six-pose workflow now fits an isotropic sphere: corrected =
(raw - offset) * common_scale. It no longer assumes the other channels are
zero or each raw gravity norm is already near 9.80665 m/s². Six distinct
signed directions and opposite coverage are required. Raw norm bounds are
6–14 m/s²; offset norm is limited to 3 m/s², common scale to 0.9–1.1.
Fit residual is limited to 0.15 m/s² and leave-one-pose-out error to 0.25.
Continuity, eight-second collection and stationary gyro gates remain active.
Firmware gyro transients still cause rejection rather than unsafe acceptance.

Old diagonal metadata remains readable. New sphere metadata is explicitly
marked as requiring independent validation. Saving is manual and user-local;
raw plots stay unchanged and corrected preview does not alter global output.
Sphere fitting does not establish mounting orientation or gyro calibration.
Actual recorded Android six-pose means were passed through the implementation;
max fit residual 0.018 m/s², max held-out error 0.0571 m/s², agreeing with the
earlier NumPy analysis. Installed GUI acceptance remains pending deployment.

Standalone native GTK 3 inspector, GPL-3.0-or-later. No web frontend, mock sensors,
service management, privileged writes, or global package/library modifications.

## Compact native interface

The research source now opens a dense **总览**: scheme combo, explicit Apply,
and capture toggle share one toolbar. SSC/SDL switching, two raw charts with
compact SI/XYZ/rate/age rows, and **静置校准 / 六面校准** actions share that view.
There is no three-page navigation bar or separate input-scheme landing page.
Calibration has **返回总览** and retains its two calibration modes.
Capture remains explicit; changing views does not start, stop, or restart capture.
Each source view shows its own two raw sensor charts. SSC receipt rate and age
are visible; SDL reports host reads and does not claim fresh hardware samples.
Stale/stopped values become dashes; retained charts are marked historical.
Local corrected preview is separate from raw charts and available only for SSC.

Calibration has stationary-gyro and physical-pose six-face modes, an eight-second
progress bar, and explicit save. The six compact pose markers are read-only;
previous/next and explicit collection remain touch-sized controls. The lower-body
diagram is 156×96 minimum and sits beside the pose instruction and readiness text.
All inferred channel association/preflight/opposite-pair checks are retained.
Stop clears unsaved candidates and faces as before. Diagnostics,
device identity, and calibration constraints move into **更多** without suppressing
visible failure summaries. Backend discovery/preflight/target mutation/observation,
calibration acceptance thresholds, and atomic merged JSON saving are unchanged.

`motion-lab.css` is loaded beside `motion_lab.py`, scoped to this application's
window, using installed GTK theme colors and 44-pixel touch targets. Include this
file when copying or packaging the research source. Default size is capped by
the primary monitor workarea, with a 600×480 fallback.

Geometry budget: 6-pixel outer margin, 2–6-pixel content gaps, 44-pixel minimum
touch controls, two 70-pixel minimum plots, and two text rows per sensor. Normal
single-device overview targets about 440–490 logical pixels including HeaderBar,
subject to actual theme/font/workarea allocation. Plot cards have no padding or
decorative borders. Multiple-device selection, errors and corrected preview may
need vertical scrolling; never truncate units or hide required device selection.
Hidden Stack pages do not set the visible page's maximum natural height.

P1 remains: wide-screen side-by-side charts, explicit SDL multi-controller filtering,
and real timestamp chart axes. Multiple SDL controllers retain the existing
last-reporting-controller policy, with the actual source name now visible.
Real Thor GUI/touch validation and package deployment are performed separately
by the main session; source checks do not establish runtime calibration quality.
Local verification: Python AST parsing and actual `--help` succeeded; the installed
host GTK3 parsed `motion-lab.css` without errors. Static source review against
the calibration-agent handoff preserves backend methods, pose inference,
face collection, full calibration gates, fitting and JSON saving. Density edits
are constructors, tick presentation and update_faces labels plus view navigation.
No unit tests or device operations were run.

## Run / packaging handoff

Public open-source dependencies on Arch/Thor: `python`, `python-gobject`, `gtk3`,
`python-cairo`, and `sdl2` or `sdl2-compat` providing the SDL2 GameController sensor
API (SDL 2.0.14+). Existing `thorch-motion` supplies
`/usr/bin/thorch-motion-capture`, whose wrapper scopes corrected libssc to its own
child. The motion service must **already** be active; this application never starts it.

Run as the desktop user, not root:

```sh
sh research/motion-integration/motion-lab/run-motion-lab
```

Choose **Start inspection** explicitly. Each independent accel/gyro child requests
100 Hz with a 20-second capture timeout, a 25-second kill watchdog, and 3-second
restart delay. Asynchronous Gio reads keep GTK responsive. Stop/window close kills
owned children; no unrelated processes are touched. The wrapper execs the CLI so
there is no extra shell child to orphan. An alternative existing private wrapper
can be selected with `--capture /absolute/path/to/wrapper`.

For a future user-local installation, copy this directory to
`~/.local/share/thor-motion-lab`, install an executable launcher named
`~/.local/bin/thor-motion-lab` invoking that directory's `run-motion-lab` with `sh`,
and copy the provided desktop file to `~/.local/share/applications/`. Ensure
`~/.local/bin` is on desktop PATH. No installation is performed here.

## InputPlumber protocol selector (original API only)

The **输入方案 / InputPlumber gamepad protocol** panel uses asynchronous Gio on
the system bus, service `org.shadowblip.InputPlumber`. No new named-scheme backend
API is required. It reads manager `SupportedTargetDeviceIds` at
`/org/shadowblip/InputPlumber/Manager` through `org.freedesktop.DBus.Properties.Get`.
Two **local UI presets** appear only if their real target IDs are advertised:
`xbox-compat → xb360` (no native SDL motion) and `dualsense-motion → ds5`
(DualSense motion-capable protocol, not a promise of available sensors/game support).
They are frontend labels, not entries fabricated as backend scheme discovery.

`ObjectManager.GetManagedObjects` at `/org/shadowblip/InputPlumber` discovers live
composites. No CompositeDevice0 assumption: each composite is identified using
`Name`, `PersistentId`, and the complete `/dev/` `SourceDevicePaths`. Only a unique
AYN Layout/Thor name with `/dev/fastrpc-adsp` plus an input-event/hidraw source is
preselected, and even that only reads state. Otherwise choose the intended
composite explicitly; foreign gamepads are never selected automatically.
`ProfileName` is displayed or marked unknown. Composite `TargetDevices` are **DBus
paths**, resolved using each target's existing `org.shadowblip.Input.Target.DeviceType`.
The current preset is inferred from exactly one gamepad type; other recognized
single-gamepad layouts show `custom`. Zero/multiple gamepads or any unrecognized
target type disable switching rather than guessing from a name/path.

**应用 / Apply** is the only mutation. It re-reads composite properties, resolves
all live target types, and rechecks Name/PersistentId/all source paths, target
paths, DBus target paths and profile immediately before sending the original
`org.shadowblip.Input.CompositeDevice.SetTargetDevices(as target_type_ids)`.
The replacement list changes only the gamepad type and preserves every known
non-gamepad type/count (mouse, keyboard, standalone touch, null, debug, DBus as
applicable). Already-selected protocols are a read-only no-op. No profile/source
mapping or SSC calibration is changed. **Existing target instances may be stopped
and recreated by this original backend method**, including non-gamepad instances:
preserving their types is not preserving device handles or promising uninterrupted
input. Dedicated `DbusDevices` are additionally checked unchanged after switching.

The setter reply means **enqueued**, not committed. The UI asynchronously polls
fresh composite properties plus target types for at most 10 seconds after that
reply. It requires two successive observations of exactly one requested gamepad,
matching non-gamepad type counts, unchanged dedicated DBus targets, identity and
profile. Only then is matching state reported and SDL rescanned immediately and
again after 1.2 seconds (old SDL handles are closed by rescan). This confirms
observable metadata, **not** actual gameplay/motion/rumble/routing success.
Timeouts, unknown types, missing APIs and Polkit errors disable Apply with a clear
message; use **刷新 / Load** to rediscover. No authorization bypass, new backend
API, automatic application/persistence, privileged command, or rollback is used.

**Race limitation:** original SetTargetDevices has no expected-identity arguments
or atomic transaction guarantee. Local preflight cannot close the gap between
last read and backend processing, prevent another client changing targets, or
guarantee rollback after failure. A timeout/window close does not cancel an already
enqueued backend operation. Closing cancels only client calls/subscriptions; late
callbacks/timers are generation-guarded and do not update destroyed UI. Relevant
hotplug/owner/property changes require a fresh Load before another Apply. Body
capture supervision and all calibration data are independent of this selector.
PersistentId may itself fall back to the composite DBus path when hardware has
no persistent identifier; this is not a durable cross-restart identity guarantee.

## Data and calibration

Body accel uses m/s²; body gyro uses rad/s from the corrected private CLI.
Only documented measurement lines with `t_us` are accepted. These timestamps are
host monotonic callback receipt, **not DSP timestamps**. SDL uses real enumeration,
HasSensor, SetSensorEnabled and GetSensorData. Missing sensors (including current
Xbox-style virtual controllers) are reported explicitly, never synthesized from
body data. Rescan after hotplug; if several controllers have sensors, the last
reporting controller is plotted and identified in the vector tooltip. SDL values
are host-polled snapshots, not a measured hardware delivery rate. Body delivery
rate and sample age are available in vector tooltips. Plots auto-scale independently.
Axes are unknown physical frame; channel colors do not assert device orientation.

Place Thor on a stable surface, start inspection, then select stationary gyro.
An 8-second window requires at least 400 samples of **both** body sensors,
continuous delivery (no >0.3-second gap), gyro per-axis standard deviation ≤0.015
rad/s, every gyro norm ≤0.15 rad/s, accel per-axis standard deviation ≤0.12 m/s²,
and mean gravity norm between 9.0 and 10.6 m/s². These conservative heuristics
reject obvious motion but cannot establish that very slow constant rotation is
absent. Operator responsibility: keep the device stationary. Windows overlapping
capture restarts may be rejected; wait for fresh live capture and retry.

### Guided six-face acceleration

Open **校准 → 六面加速度**. Ordinary-user instructions name six physical poses:
**屏幕朝上 / 屏幕朝下 / 上边朝上 / 下边朝上 / 左边朝上 / 右边朝上**.
These refer to the **lower body carrying the buttons and lower screen**, where
the sensor is located. Moving only the upper display does not change the pose;
the hinge can be closed. The lower-body diagrams use a dashed hinge reference
and an upward arrow toward the ceiling. Securely support the device; avoid
pressing buttons or damaging screens when face-down. No external images or emoji.

Use **上一步 / 下一步** to select a pose, follow its illustration/instruction,
then explicitly press **采集此姿势 · 8 秒**. The app never auto-captures or
auto-advances. Before that button starts a window, it recognizes an **internal
reported channel** from ≥1 second of recent stable raw acceleration, ≥50 samples,
no >0.3-second gaps, ≤0.5-second timestamp delay and a final sample ≤0.15 seconds
old. Accel stability, gravity and dominant/cross-channel gates match the strict
capture gates below. A gravity norm around 8.2 m/s² is rejected, not normalized
or silently accepted. Internal signed axes are visible only in advanced diagnostics.

No physical-pose-to-axis mapping is assumed in advance. Only a successful real
8-second window records the user-selected pose's association with its measured
channel. A channel cannot be reused by another pose; each physical opposite pair
must have opposite signs on the same internal axis. Contradictions prompt physical
repositioning/reset, never guessed relabeling. Six poses must cover all six unique
signed channels and all three opposite pairs before the unchanged diagonal fit
runs. This detects numerical contradictions, not whether the user followed the
illustration correctly. Physical mounting remains **unknown/unverified**.

Completed chips use pose names. A quality-approved recollection invalidates the
previous pose/channel result immediately before starting; if the new capture
fails, that pose stays incomplete. A preflight rejection does not discard an
existing result. Reset/Stop clears in-memory face/pose associations only and
preserves saved JSON. Accepted candidate metadata includes
`user_pose_reported_channels` and a measured-consistency validation marker;
these never rotate global coordinates or apply a mounting transform.

Each face uses the gyro stationary rejection above, including the raw 0.15 rad/s
norm ceiling even when gyro bias is nonzero; both sensors must have ≥400 samples,
start/end coverage within 0.5 seconds, no host receipt/sample-time gap >0.3 seconds,
and no delayed sample whose CLI timestamp differs from UI receipt by >0.5 seconds.
Face windows also require **every** accel norm between 9.0 and 10.6 m/s², the
chosen signed channel ≥8.8 m/s² for every sample (mean between 9.0 and 10.6),
and every other channel ≤1.25 m/s² in magnitude. These are reported coordinate
channels, not top/bottom/left/right mounting directions.

Only six accepted faces allow deriving a **diagonal** model, with `g = 9.80665`:

```text
offset[i] = (positive_face_mean[i] + negative_face_mean[i]) / 2
scale[i]  = 2*g / (positive_face_mean[i] - negative_face_mean[i])
corrected[i] = (raw[i] - offset[i]) * scale[i]
```

Each offset must be within ±0.6 m/s² and each scale within 0.9..1.1. All six
corrected mean vectors must have norm residual ≤0.25 m/s², signed target-channel
residual ≤0.2 m/s², and each cross-axis magnitude ≤0.5 m/s². If the fit fails,
recollect misaligned faces; there is no cross-axis/rotation compensation or
assumption about mounting orientation. Face means and residuals are saved with
accepted coefficients. Numerical consistency of the same six collected windows
is **not** an independent physical validation or a claim of full sensor calibration.

### Saving and local preview

**Save calibration / 保存** explicitly writes atomic JSON, mode 0600, under
`${XDG_CONFIG_HOME:-~/.config}/thorch-motion/calibration.json`. Existing JSON at
that path is re-read and merged only on this action: accel-only saves preserve
gyro bias; gyro-only saves preserve accel coefficients; unrelated top-level and
accelerometer metadata fields survive. Invalid/non-finite/duplicate-key JSON,
files over 1 MiB, unsupported schema/source/frame/units, and unsafe coefficient
bounds block saving instead of silently overwriting the file. Atomic replacement
is followed by directory fsync. This is a single-writer desktop workflow, not a
multi-process calibration transaction API.

Startup cautiously reads compatible gyro/accel metadata. The optional **Corrected
preview** displays local BODY corrections using accepted unsaved candidates first,
otherwise loaded/saved coefficients. It does not change raw charts, captured
samples, calibration checks, SDL values, or any global consumer. Gyro coefficients
describe `raw - bias`; accel coefficients use the diagonal formula above. Nothing
is applied to a sensor partition, calibration service, firmware, or other app.
Stopping clears unsaved candidates/faces; saved metadata remains intact.

## Verification and limitations

Read-only Thor checks on 2026-10-12: Python 3.14.7, python-gobject 3.56.3,
GTK 3.24.52, sdl2-compat 2.32.74; PySide6 absent; thorch-motion service active.
Python Cairo 1.29.2 is installed; GTK/Gio imports and all three real SDL2 sensor
symbols were verified on Thor without initializing controllers or sampling sensors.
Installed CLI `--help` exposes `--sample-rate`, `--sensor`, `--timeout` (its generic
sensor help omits gyroscope, but the private source supports it).
Local Python AST syntax validation, launcher shell syntax, and actual application
`--help` completed successfully without GUI startup or device access.
For the protocol panel, original binding XML and implementation were read;
read-only live metadata confirmed supported xb360/ds5 IDs, composite `AYN Layout`,
full `/dev/input/event3` + `/dev/fastrpc-adsp` source identity and target DeviceType
`ds5` with mouse/keyboard targets. These paths were observed, not hardcoded into
selection. A subsequent Gio metadata-shape query could not connect (`No route
to host`), so actual asynchronous UI switching still requires main-session desktop
validation after packaging. No SetTargetDevices call was made during development.
No device sampling or calibration was performed during development. No unit tests.
The main session subsequently deployed the original app user-locally and launched
it without reported errors; live inspection/calibration were not physically verified.
This six-face implementation updates repository source only; no deployment,
device sampling, device writes, or service changes were performed here.
Main session/user: after deliberate deployment, physically collect six orientations,
check accepted/rejected motion windows, verify local preview and JSON preservation,
and independently verify other stationary orientations. That real device validation
remains required; it must not be inferred from syntax or `--help` success.
