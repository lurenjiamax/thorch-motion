# Thor Motion Lab package

Installable native GTK inspection/calibration application. Reads the dedicated
Thorch SSC capture tool and genuine SDL controller sensors independently. It
never changes firmware, original Android calibration or system input routing.

Explicit saves write the desktop user's thorch-motion/calibration.json only.
Raw plots remain raw; optional corrected preview is application-local, not an
automatic global calibration. Six-face calibration requires physical collection
and validation. See the installed application README for operating instructions.

Application payload links to the sole Git-managed source in
research/motion-integration/motion-lab. Packaging must dereference these links
into package inputs; do not maintain another editable application source copy.
The package is user-launched and does not enable an application boot service.
