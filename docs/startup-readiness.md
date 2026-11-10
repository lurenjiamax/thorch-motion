# Non-blocking sensor startup

The enabled service uses Type=exec. Boot does not wait for DSP startup or SSC
samples. The supervisor waits in the background until the configured FastRPC
device exists, sns_reg_version is present and the import marker is absent.
Unlike ConditionPathExists, this keeps watching prerequisites that appear late.
STATUS reports waiting prerequisites, SSC startup or sensor readiness; an active
unit alone is not proof that samples are available. SIGTERM stops the wait.
After prerequisites are ready, the existing bounded SSC startup and restart
policy handles daemon failures. Installation still does not enable the service.

On 2026-11-10 the service-only 0.5.0-11 package was installed on the development
Thor without replacing runtime binaries or device calibration. The service
started asynchronously rather than being skipped. Initial sampling reported
SSC QMI Service not found; end-to-end sensor acceptance remains pending.
