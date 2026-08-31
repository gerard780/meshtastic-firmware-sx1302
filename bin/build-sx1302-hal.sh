#!/usr/bin/env bash

set -euo pipefail

# Build the pinned Semtech SX1302 HAL as a PIC shared library with the small
# Meshtastic ABI patch. The result can be copied to /usr/local/lib/libloragw.so
# on a miner, or selected with Lora.SX1302_LIB.

hal_repository="https://github.com/Lora-net/sx1302_hal.git"
hal_revision="4b42025d1751e04632c0b04160e0d29dbbb222a5"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
patch_file="${script_dir}/patches/sx1302-meshtastic-syncword.patch"
output_path="${1:-${PWD}/libloragw-meshtastic.so}"
build_dir="$(mktemp -d)"
object_dir="${build_dir}/pic_obj"

cleanup() {
    rm -rf -- "${build_dir}"
}
trap cleanup EXIT

git -C "${build_dir}" init --quiet
git -C "${build_dir}" remote add origin "${hal_repository}"
git -C "${build_dir}" fetch --quiet --depth 1 origin "${hal_revision}"
git -C "${build_dir}" checkout --quiet --detach FETCH_HEAD
git -C "${build_dir}" apply "${patch_file}"

make -s -C "${build_dir}/libloragw" inc/config.h
mkdir -p "${object_dir}"

for source in "${build_dir}"/libtools/src/*.c; do
    object="${object_dir}/tools_$(basename "${source%.c}").o"
    gcc -c -O2 -fPIC -Wall -Wextra -std=c99 \
        -I"${build_dir}/libtools/inc" -I"${build_dir}/libtools" \
        "${source}" -o "${object}"
done

for source in "${build_dir}"/libloragw/src/*.c; do
    object="${object_dir}/loragw_$(basename "${source%.c}").o"
    gcc -c -O2 -fPIC -Wall -Wextra -std=c99 \
        -I"${build_dir}/libloragw/inc" -I"${build_dir}/libloragw" -I"${build_dir}/libtools/inc" \
        "${source}" -o "${object}"
done

gcc -shared -Wl,-soname,libloragw.so -o "${build_dir}/libloragw.so" "${object_dir}"/*.o -lrt -lm -lpthread
mkdir -p "$(dirname "${output_path}")"
cp "${build_dir}/libloragw.so" "${output_path}"

printf 'Built %s from Semtech SX1302 HAL %s\n' "${output_path}" "${hal_revision}"
