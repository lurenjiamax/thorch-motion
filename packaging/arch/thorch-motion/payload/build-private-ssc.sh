#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Adapted from the placement study's direct native recipe; no Meson or tests.
set -euo pipefail
src=$(CDPATH= cd -- "$1" && pwd)
mkdir -p -- "$2"
out=$(CDPATH= cd -- "$2" && pwd)
if [[ "$out" == "$src" || "$out" == "$src/src" || "$out" == "$src/data" ]]; then
    echo 'Private output must be separate from the source directories.' >&2
    exit 1
fi
cc=${CC:-cc}
for tool in "$cc" pkg-config protoc protoc-gen-c sed; do
    command -v "$tool" >/dev/null || { echo "Missing tool: $tool" >&2; exit 1; }
done
pkg-config --exists 'glib-2.0 >= 2.56' gio-2.0 gio-unix-2.0 gobject-2.0 'qmi-glib >= 1.33.4' libprotobuf-c
mkdir -p -- "$out/generated" "$out/obj"
protos=(common sensor-suid sensor-proximity sensor-light sensor-accelerometer sensor-magnetometer sensor-rotationvector sensor-gyroscope)
generated=()
for name in "${protos[@]}"; do
    protoc --proto_path="$src/data" --c_out="$out/generated" "$src/data/ssc-$name.proto"
    generated+=("$out/generated/ssc-$name.pb-c.c")
done
sed -e 's/@LIBSSC_MAJOR_VERSION@/0/g' -e 's/@LIBSSC_MINOR_VERSION@/4/g' \
    -e 's/@LIBSSC_PATCH_VERSION@/4/g' -e 's/@VERSION@/0.4.4/g' \
    "$src/src/libssc-version-private.h.in" > "$out/generated/libssc-version-private.h"
read -r -a pkg_cflags <<< "$(pkg-config --cflags glib-2.0 gio-2.0 gio-unix-2.0 gobject-2.0 qmi-glib libprotobuf-c)"
read -r -a pkg_libs <<< "$(pkg-config --libs glib-2.0 gio-2.0 gio-unix-2.0 gobject-2.0 qmi-glib libprotobuf-c)"
read -r -a cppflags <<< "${CPPFLAGS:-}"
read -r -a cflags <<< "${CFLAGS:--O2 -g}"
read -r -a ldflags <<< "${LDFLAGS:-}"
flags=(-std=gnu99 -fPIC -Wall -Wextra -Wno-unused-parameter -Wno-cast-function-type -Wno-packed
       -DGLIB_VERSION_MIN_REQUIRED=GLIB_VERSION_2_56 -DGLIB_VERSION_MAX_ALLOWED=GLIB_VERSION_2_56
       -DGLIB_DISABLE_DEPRECATION_WARNINGS '-DG_LOG_DOMAIN="libssc"'
       -I"$src" -I"$src/src" -I"$src/data" -I"$out/generated")
# Exactly the upstream nine hand-written library sources plus eight protobufs.
names=(common client sensor sensor-proximity sensor-light sensor-accelerometer sensor-magnetometer sensor-compass sensor-gyroscope)
sources=()
for name in "${names[@]}"; do sources+=("$src/src/libssc-$name.c"); done
sources+=("${generated[@]}")
objects=()
for file in "${sources[@]}"; do
    obj="$out/obj/$(basename -- "${file%.c}").o"
    "$cc" "${cppflags[@]}" "${cflags[@]}" "${flags[@]}" "${pkg_cflags[@]}" -c "$file" -o "$obj"
    objects+=("$obj")
done
"$cc" -shared -Wl,-soname,libssc.so.2 -Wl,--no-undefined \
    "-Wl,--version-script=$src/libssc.map" "${ldflags[@]}" -o "$out/libssc.so.2" \
    "${objects[@]}" "${pkg_libs[@]}" -lm
ln -sfn libssc.so.2 "$out/libssc.so"
"$cc" "${cppflags[@]}" "${cflags[@]}" "${flags[@]}" "${pkg_cflags[@]}" "$src/src/libssc-cli.c" \
    -L"$out" "${ldflags[@]}" -Wl,-rpath,'$ORIGIN' -lssc "${pkg_libs[@]}" -lm -o "$out/ssccli"
# Package only target-built outputs; never copy host research artifacts.
