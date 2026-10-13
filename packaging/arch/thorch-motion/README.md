# Experimental Thor motion runtime

`thorch-motion` 0.5.0-5 provides HexagonRPC `dd9ac70` plus the exact eight
external patches, built as a native daemon with json-c. No registry generator
is built, installed or invoked. Installation does not enable or start a service;
this package is not part of the default image.

## Evidence and limits

On the investigated AYN Thor, kernel22 and reference firmware1-13.4, fresh
calibration-only registry seeds produced actual accelerometer and gyroscope
numeric samples on separate cold boots with both full configs and
subtype-selected configs (`logs081333` and `logs081513`, nine numeric samples
each). Full configs therefore do not require subtype filtering. This is limited
hardware evidence, not general compatibility or completed motion support.

Cached cold-boot restart and 60-second simultaneous movement capture succeeded:
59 numeric samples from each sensor changed with movement. Sensor attributes
identify SENODIA SH5001. The plain non-observation reference daemon also passed
cold-boot cache reuse. The native package has been built and installed; its
service validation is recorded separately in the investigation findings.
Axis orientation, scale, calibration correctness, suspend/resume, concurrent
audio and InputPlumber integration remain unverified. Process liveness does not
prove SSC availability or sample delivery.

## Private capture CLI

Revision 0.5.0-5 adds a private libssc 0.4.4 and capture CLI. Source staging
requires the main session's checksum-matching final invalid-rotation patch before
the package can build; the package deliberately cannot omit that correction.
Nothing replaces `/usr/lib/libssc.so*`, the system `ssccli`, or system loader
configuration. The wrapper uses `/usr/lib/thorch-motion/ssc` only for its child;
the private CLI additionally has `$ORIGIN` RUNPATH. Discovery/default-instance
selection is unchanged; this is not the all-instance research diagnostic.

Once the main session has built, installed and validated the runtime, capture
manually (these commands never start or enable the service):

```sh
thorch-motion-capture --sensor accelerometer --sample-rate 100 --timeout 10
thorch-motion-capture --sensor gyroscope --sample-rate 100 --timeout 10
LIBSSC_TRACE_PLACEMENT=1 thorch-motion-capture \
  --sensor gyroscope --sample-rate 100 --timeout 10
```

`--sample-rate` is optional, integer **1..200 Hz**, and valid only with
accelerometer/gyroscope. The setter runs after discovery and before open; omission
retains upstream's first-positive advertised default. Requested rate is not a
guaranteed delivery rate. Main observed 1116 callbacks per sensor over the studied
10-second request at 100 Hz, approximately 109.24 Hz: callback timing evidence,
not proof of DSP cadence, gameplay readiness or all-rate support.

Accelerometer lines use **m/s²**; gyro lines use **rad/s**, without unit conversion.
`t_us` is host monotonic callback time, **not a DSP event timestamp**. Output
flushing and scheduling can affect measured delivery. Opt-in placement tracing
prints only encountered placement metadata to stderr; it does not require
`--debug` (which also enables upstream generic QMI tracing).

Placement extraction ignores translation in a row-major 3x4 attribute and retains
bounded legacy 3x3 support. The required final patch rejects nonfinite, singular,
scaled or nonorthonormal matrices (engineering tolerance 0.01) and resets **all
nine orientation entries to identity**, rather than guessing axes. Actual Thor
metadata showed an invalid rank-one placeholder; accepting it loses two dimensions.
Identity here is a neutral fallback, not new calibration or proof of hardware axes.
The library already multiplies accelerometer callbacks by its accepted matrix;
gyro callbacks remain unchanged. Do not apply the matrix a second time, and do
not infer a verified sensor-to-device transform from the placement layout alone.

No automatic calibration, DSU server, Steam motion integration or InputPlumber
mapping is provided. Fresh-state bootstrap readiness and complete orientation/
calibration validation remain separate main-session work. Capture does not alter
the existing importer, supervisor, service or boot-enable policy.

## Import an existing private runtime

### First-use bootstrap on the validated Thor

A newly imported calibration-only runtime did not expose SSC on its first
daemon start. Keeping that DSP-generated registry unchanged and rebooting once
made both sensors available on the second start. This was reproduced with the
packaged daemon outside systemd; disabling the service filesystem sandbox or
changing its startup delay did not resolve first-use discovery.

Treat first-use registry generation and post-reboot numeric-sample validation
as separate stages. Do not reimport or discard the generated registry between
them, and do not enable boot startup merely because the daemon stays alive.
The package does not automatically reboot the device.

Use an already extracted private source owned by this device; this package never
mounts Android partitions or serves the original persist partition. Supply:

```sh
sudo thorch-motion-import \
  --source /path/to/existing-private-runtime \
  --destination /var/lib/aynthor/motion/runtime-root \
  --calibration-directory sensors/registry
```

The source must contain `dsp/adsp`, `socinfo`, `sensors/config`,
`sensors/sns_reg.conf`, matching `sensors/sns_reg_version`, and original
device-owned `.fac_cal` files. If preserved calibration files are in a nested
tree, pass its relative path instead (for example
`sensors/persist-inherited/registry/registry`). Retain the original backup and
record the seed's provenance; the successful investigation seed contains 99
device-owned groups from an independent DSP-updated runtime. This is not evidence
that every different calibration seed will work. All config files stay unchanged.
SoC metadata and version come from that same source, not package defaults.

Import stages a new independent directory, refuses any existing destination,
dereferences source-internal symlinks and rejects escaping links, cycles and
special files. Materialize external links into your private source beforehand.
Nothing is written to the source. Failed staging is retained with `.importing`
to block service startup; choose another new destination after inspecting it.
Treat source files as stable during import. Do not import an active runtime.

Only `.fac_cal` files seed physical `sensors/registry`; scratch suffixes `.temp`
and `.tmp` and other registry files are excluded. Legitimate `.temp.fac_cal`
temperature groups are preserved. The firmware owns config rescanning and
registry population. No synthetic cache, calibration or version is generated.
Other persist content is excluded by default. If a known required non-registry
file exists, explicitly select it with repeatable
`--persist-file relative/path` (relative to source `sensors/`, copied directly
under destination `sensors/` as expected by this external stack). Registry
and cache paths are rejected. No private files are shipped in the package.

The external daemon maps virtual `/persist/sensors/registry` to physical
`<runtime-root>/sensors/`: its virtual `registry/registry` resolves to physical
`sensors/registry`. Do not add another registry nesting level.

## Manual operation only

Ensure matching ADSP firmware/device tree, FastRPC and QRTR infrastructure are
already available. Do not hot-restart ADSP while audio is active.

```sh
sudo systemctl start thorch-motion.service
journalctl -u thorch-motion.service
ssccli --sensor accelerometer --timeout 10 --debug
ssccli --sensor gyroscope --timeout 10 --debug
sudo systemctl stop thorch-motion.service
```

The root-run unit uses a private writable runtime root, launches SensorsPD first,
waits one second, then launches RootPD; either exit stops both. It does not
automatically restart, overwrite metadata or enable itself at boot. QRTR and
sound ordering do not constitute readiness checks. This kernel supplies QRTR
name service without a separate `qrtr-ns.service`. Validate actual numeric
samples after a controlled cold boot before considering further integration.
Keep the service disabled during this experimental phase.

See `payload/PROVENANCE.md` for pinned sources, checksums and GPL licensing.
