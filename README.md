# Thorch Motion

Linux SSC runtime integration and motion inspection tools. Currently verified
only on AYN Thor / Qualcomm SM8550 with SENODIA SH5001. Other devices are not
claimed supported merely because their SoC exposes FastRPC.

## Layout

- `runtime/`: private-runtime importer, capture wrapper and DSP supervisor.
- `systemd/`: readiness-aware runtime service. It sends `READY=1` only after
  both accelerometer and gyroscope return samples. InputPlumber no longer
  depends on this service's lifetime and reconnects its optional SSC source.
- `profiles/`: declarative device documentation and compatibility boundaries.
- `apps/motion-lab/`: native GTK inspection and explicit user-local calibration.
- `apps/sensorproxy/`: native SensorProxy bridge for desktop orientation.
- `packaging/arch/thorch-motion/`: Arch recipe, pinned upstream patches and
  build metadata. Payload links reference the sole source rather than copies.

The supervisor accepts an explicit `--profile` manifest for FastRPC device and
DSP domain. This is not automatic model detection; daemon topology and service
paths remain Thor-specific. Supporting another model requires
reviewed DSP topology and firmware matching, safe calibration provisioning,
coordinate characterization, packaging, boot and suspend validation.

## Dependencies and downstreams

- [hexagonrpc](https://github.com/linux-msm/hexagonrpc), downstream
  [maintained main](https://github.com/lurenjiamax/hexagonrpc/tree/main).
- [libssc](https://codeberg.org/DylanVanAssche/libssc), GitHub downstream
  [maintained main](https://github.com/lurenjiamax/libssc/tree/main).
- [InputPlumber](https://github.com/ShadowBlip/InputPlumber), downstream
  [maintained main](https://github.com/lurenjiamax/InputPlumber/tree/main).

Package recipes retain checksum-pinned upstream baselines and reviewed patches.
Publishing downstreams does not silently change the binaries installed on the
device. Motion Lab uses InputPlumber's existing target-switch APIs.

The motion service no longer conflicts with suspend.target. Kernel task
freezing pauses userspace across sleep; an SSC stream that becomes stale is
reopened by InputPlumber's worker after resume. Physical input and virtual
targets must remain present if the motion service is deliberately stopped.

**Suspend status: unresolved.** Service-lifetime independence was observed on
Thor, but full system suspend still hangs/resets in reported runs. Stopping the
motion supervisor did not interrupt the existing SSC stream, so that check does
not prove DSP-loss reconnection. These changes are not a verified platform
suspend fix.

## Calibration and privacy

Original Android files are read-only sources; import uses a private runtime.
No firmware, device keys, factory calibration or private captures are included.
The accelerometer sphere model estimates offset and one common scale, not
mounting rotation or gyro calibration. Continuity, stationarity, direction
coverage, residual and held-out-pose checks precede explicit saving. Corrected
preview is local; raw charts and global SSC output remain unchanged.

## Verification

On Thor: actual SSC samples, SDL accel/gyro, explicit target switching, package
installation and systemd notify startup have been exercised. Independent sphere
calibration validation, cold-boot notify acceptance and subjective haptics
acceptance remain pending. No unit-test execution is claimed.

Build package inputs with symbolic links dereferenced, then use `makepkg` and
pacman. Packaging requires the dependencies listed in PKGBUILD and private
device-owned runtime provisioning. Do not enable this service on unverified
hardware merely to make a generic dependency graph succeed.
