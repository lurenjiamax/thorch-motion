# Source and licensing

HexagonRPC: https://github.com/linux-msm/hexagonrpc
Pinned commit: `dd9ac70c026e1bad93e8cffa3801255b8ceb551e`.
Codeload archive SHA-256:
`575bf34b8e2a51f851b078cc4b87db3c9bb121ce5849be765352b016756d63eb`.
The investigation's git-archive tarball has a different archive layout and hash:
`f7363ffdc18574113ddb2149e2a7ccfff11eadc86e9ddda251a6f792edad36d4`.

The eight patches are byte-for-byte copies of the investigated external stack:
https://github.com/MaSieS4Fun/SteamOS-ARM-SM8550/tree/3548c401e6d2193a8a9ba7ef08add7074ea53f9e/external-and-mods/kernel/vendor/qcom-gyro/src/hexagonrpc

Patch headers preserve authorship (including suckbluefrog) and original comments.
The external recipe declares GPL-3.0+; HexagonRPC and these derivative patches are
distributed under GPL-3.0-or-later, with upstream COPYING included. The new Thorch
importer and supervisor are also GPL-3.0-or-later. Package checksums pin every
patch. Patches are applied in numerical order; patch 0002 needs fuzz 2 against
this pinned base, as in the reproduced reference preparation. Patch 0001 is
retained for source equivalence only: its registry generator is neither built,
installed nor invoked.

No proprietary DSP payloads, calibration data, keys or private sensor configs
are included. Existing payload/socinfo files are legacy investigation metadata;
they are not installed or used to override imported source metadata.

## Private SSC capture addition (0.5.0-5 staging)

Official upstream: https://codeberg.org/DylanVanAssche/libssc
Version 0.4.4, commit `0cf77b93b55752da34dca2dcecc06fca8665184b`.
Archive: https://codeberg.org/DylanVanAssche/libssc/archive/0cf77b93b55752da34dca2dcecc06fca8665184b.tar.gz
SHA-256: `716d6bd6b34d2d753060c6b54c9a87e34fae75b724c763bf9ef487efa3621587`.
The archive contains the original public source, not a diagnostic source tree.
Official GitHub repositories were not found; an inspected GitHub mirror's codeload
endpoint returned 404 for this commit. Canonical Codeberg is used rather than
substituting another development branch or calling a third-party mirror official.
Upstream copyright: Dylan Van Assche and contributors; GPL-3.0-or-later headers,
with the upstream LICENSE installed separately.

Applied in this order, each pinned in PKGBUILD:

1. `libssc-placement-trace.patch`: byte-preserved placement-study derivative of
   upstream libssc; stride-four 3x4 placement extraction, bounded legacy 3x3
   compatibility, rad/s gyro label, and opt-in placement-only metadata trace.
2. `libssc-sample-rate-override.patch`: exact public MaSi-vendored
   `0003-sensor-allow-sample-rate-override.patch`, author **suckbluefrog**;
   commit header `6f7f01f5cb7df9d463da02815dbd9b42d1206244`. Existing provenance:
   https://github.com/MaSieS4Fun/SteamOS-ARM-SM8550
   The retained patch headers/hash are authoritative; a direct public blob URL
   for the separately collected live vendor tree has not been established here.
3. `libssc-cli-rate-capture.patch`: byte-preserved placement-study derivative;
   optional bounded requested rate, pre-open setter, host monotonic callback time.
4. `libssc-reject-invalid-rotation.patch`: reserved required final patch,
   main-session integration from completed `libssc-invalid-rotation.patch`, hash
   `146cdf3d4c8f2ddfee66c8116815277a59c297ef6cbe0954c2f5e4a898915125`.
   Missing/mismatching final patch fails source verification; it is never skipped.

Local derivative patches and the new build recipe/wrapper are distributed under
GPL-3.0-or-later with the upstream license. Patch contents/headers are preserved;
the placement-study integration is credited to Thorch contributors. No
all-instance discovery diagnostic, private runtime/config, calibration, keys,
DSP payload or prebuilt host library is an input to this capture build.

The capture CLI and SONAME library install only under `/usr/lib/thorch-motion/ssc`.
The CLI links with `$ORIGIN` RUNPATH and the wrapper scopes LD_LIBRARY_PATH to
that child. No public headers/pkg-config metadata, global library replacement,
ld.so configuration, or system-wide libssc symlink is installed.
