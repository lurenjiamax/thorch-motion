# Thor SSC → SensorProxy orientation compatibility bridge

## Goal, facts, method, expected result

Expose corrected native private SSC accelerometer readings through the system-bus
contract KWin already consumes, without changing display layouts, KDE settings,
or global packages. The coordinator reports private libssc 0.4.4 plus
`ssc_sensor_set_sample_rate()` working at 100 Hz at
`/usr/lib/thorch-motion/ssc/libssc.so.2`; the system libssc has the wrong matrix.
This program uses public libssc headers and loads **only that absolute library**
with GModule. There is no fallback and no `-lssc` linkage.

This is a userspace **SSC compatibility bridge**, not an IIO device or a genuine
IIO backend. Upstream iio-sensor-proxy 3.9 itself contains an SSC driver; this
separate bridge exists to isolate the corrected private library and physical
frame validation. The expected result is orientation signals KWin understands;
physical rotation and KWin display selection remain unverified.

## Contract inspection (2026-10-12)

Read upstream 3.9:

- https://gitlab.freedesktop.org/hadess/iio-sensor-proxy/-/raw/3.9/src/net.hadess.SensorProxy.xml
- https://gitlab.freedesktop.org/hadess/iio-sensor-proxy/-/raw/3.9/src/iio-sensor-proxy.c
- https://gitlab.freedesktop.org/hadess/iio-sensor-proxy/-/raw/3.9/src/orientation.c
- https://gitlab.freedesktop.org/hadess/iio-sensor-proxy/-/raw/3.9/src/monitor-sensor.c

Read KWin source (master, not a claim about the installed device version):

- https://raw.githubusercontent.com/KDE/kwin/master/src/utils/orientationsensor.cpp
- https://raw.githubusercontent.com/KDE/kwin/master/src/net.hadess.SensorProxy.xml

Both use service/interface `net.hadess.SensorProxy`, object
`/net/hadess/SensorProxy`. KWin reads `HasAccelerometer:b`, calls
`ClaimAccelerometer()`, gets `AccelerometerOrientation:s`, listens to
`org.freedesktop.DBus.Properties.PropertiesChanged(sa{sv}as)`, releases when
unsubscribing and watches service ownership. Recognized strings are `undefined`,
`normal`, `bottom-up`, `left-up`, `right-up`.

**AccelerometerUnit is NOT an upstream 3.9 property and KWin does not need it.**
Here it is an explicitly nonstandard read-only `s` extension, `m/s^2`, describing
the SSC values used internally (no raw acceleration is exported). Tilt is
`undefined`, as it is not implemented. `HasAmbientLight` and `HasProximity` are
false; no fabricated light/proximity values, methods, streams or compass object
are registered. Use `monitor-sensor --accel`, not its all-sensor default.
This is an orientation-compatible subset, not the entire upstream interface.

Claims are idempotent per D-Bus unique sender. One client cannot release another
client's claim. Each sender has a GLib name watch removed on release or disconnect;
the final release/disconnect asynchronously closes the stream. Discovery retains
an unstreamed sensor object when idle. Calls may claim before availability and
discovery retries every 2 seconds. A stream with no finite samples for 5 seconds
is marked unavailable, closed, rediscovered and reopened if claims remain.

Differences from the upstream implementation: claims reply immediately (initial
orientation can be `undefined` until a stable sample), rather than delaying the
first reply until a reading. The supplied bus policy explicitly permits local
clients including KDE to claim orientation; it does not implement upstream's
Polkit active-session check. Unsupported sensor APIs are deliberately absent.

The actual public SSC signal in the supplied upstream source is
`measurement(SSCSensorAccelerometer *, gfloat x, gfloat y, gfloat z, gpointer)`;
it has three floats and **no timestamp**. The bridge uses GLib monotonic time.
SSC open/create/close calls use their real asynchronous finish callbacks, serially.

## Physical frame: deliberately no default

`--frame` is mandatory: a signed permutation such as `+x,+y,+z`. That example
**is not a Thor mapping recommendation**. Map the corrected libssc output into
the display's physically observed orientation frame. No calibration values or
device-specific guessed axes are supplied. Placement correction is already
applied by private libssc; do not apply its raw mount matrix a second time.

The classification follows upstream sign conventions: display-frame negative Y
means `normal`, positive Y `bottom-up`, positive X `left-up`, negative X
`right-up`. Validate all four held-up edges against the intended panel's natural
orientation, including both actual Thor screens. This proxy exposes a single
device orientation and cannot choose which KWin output should rotate. Do not
edit KScreen configuration or silently normalize either panel's current layout.

100 Hz SSC acquisition is the reported working rate; evaluation is throttled to
10 Hz (`--process-ms=100`) and publishing occurs only on stable changes. Lower
SSC rate is configurable with `--sample-hz`, but **not validated**; do not describe
this as proven low-power 10 Hz acquisition. Default debounce is 450 ms. Near-flat,
diagonal, nonfinite, free-fall or strong-motion readings do not choose a new
orientation. A dominant-axis 0.18g margin and 0.65g projected threshold avoid
boundary jitter. No acceleration/calibration data are logged.

## Build and package handoff (staging only)

Host prerequisites: C compiler, make, GLib/GIO/GModule development files and
matching corrected libssc public headers (the patched rate declaration is
required). From this directory:

```sh
make
./thor-sensorproxy --help
```

The local default header path is the existing corrected reference tree. For an
aarch64 native package build, unpack the corrected libssc source/header payload
and run:

```sh
make CC=cc SSC_INCLUDE=/absolute/path/to/corrected-libssc/src
```

Cross builds must use target GLib pkg-config metadata/sysroot; the locally built
binary is a **host build artifact**, not a Thor deployment binary. Verify the
target executable has no system `libssc.so.2` NEEDED dependency. Runtime loading
fails if the private library or any required rate symbol is missing.

Coordinator may stage these paths in its package root:

| Source here | Intended installed path |
| --- | --- |
| target `thor-sensorproxy` | `/usr/lib/thorch-motion/thor-sensorproxy` |
| `thor-sensorproxy.service` | `/usr/lib/systemd/system/thor-sensorproxy.service` |
| `net.hadess.SensorProxy.conf` | `/usr/share/dbus-1/system.d/thor-sensorproxy.conf` |

The coordinator must separately create `/etc/thorch-motion/sensorproxy.env` with
`THOR_SENSOR_FRAME=` set to a **physically validated** mapping. No config is
installed here. No install target, auto-enable hook, D-Bus activation file or
`[Install]` section is supplied. Treat this as a separate opt-in service, not a
silent replacement for `iio-sensor-proxy`. It conflicts with that service and
does not queue for or replace an existing bus owner. If the system proxy is
D-Bus auto-activated, the coordinator must explicitly arrange its temporary
conflict/masking policy for validation and document rollback; Conflicts alone
does not remove its activation file.

The unit uses `Type=simple`, not `Type=dbus`: systemd rejects loading two units
with identical `BusName=` even if they conflict. The process owns its name via
GDBus with DO_NOT_QUEUE and exits nonzero on a collision; service-start success
is not a readiness assertion. The coordinator must check the actual bus owner.

Only root may own the bus name under the staged policy; non-root KDE clients can
call Claim/Release plus read-only Get/GetAll, Introspect and Ping at this exact
object. No wildcard destination, sensor Set permission, compass permission or
unrelated service permissions are granted. Existing installed policy can broaden
permissions, so the coordinator must inspect the combined policy. Root owns the
service because SSC/FastRPC permissions are device-specific; it is not sandboxed
away from its required hardware devices.

## Coordinator validation / rollback checklist

No device install/start/reboot or unit tests were performed by this worker.
Host compilation uses `-Wall -Wextra -Werror`; CLI help and argument rejection
can be exercised without touching any bus or hardware.

1. Record the actual two-screen layout and installed KWin version. Preserve them.
2. Build natively/stage package; inspect symbols/dependencies, private library
   ownership and runtime dependency paths. Check combined D-Bus policy.
3. Resolve the system proxy name conflict explicitly, opt in with validated
   frame and coordinator-managed service start. Never enable automatically.
4. Inspect with `busctl --system introspect net.hadess.SensorProxy
   /net/hadess/SensorProxy`; Get/GetAll must expose the documented types.
5. Run `monitor-sensor --accel` while moving through four orientations, near-flat
   and diagonal holds. Monitor PropertiesChanged, stable debounce and no jitter.
   A one-shot `busctl call ... ClaimAccelerometer` disconnects immediately:
   its claim should be cleaned up, not kept as a monitoring subscription.
6. Two long-lived clients should keep acquisition alive until both release or
   disconnect. A crashed client must not leak a claim. Observe SSC open/close
   through coordinator-owned diagnostics without publishing calibration values.
7. Only after bus behavior is correct, validate KWin's existing rotation path
   against actual screens. No KDE settings change is authorized by this bridge.
8. On rollback stop the bridge, restore the original proxy conflict/mask state
   and package policy; preserve original display settings throughout.

Runtime hardware integration, physical frame, energy usage, disconnect behavior
on a real system bus, and KWin behavior are coordinator validation gates, not
results claimed by this source handoff.

Host verification: compilation succeeded with warnings as errors; `--help`
executed; duplicated-axis input rejected with exit 2; `readelf -d` showed GLib,
GIO, GModule, libm and libc, but no libssc NEEDED entry. Initial unit verification
found the duplicate BusName issue above, which was corrected. Final unit
verification is expected to report the staged executable is absent from its
future installed path on this host; nothing was installed to satisfy it.
