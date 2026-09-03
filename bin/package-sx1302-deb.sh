#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 4 ]]; then
	echo "Usage: $0 <meshtasticd> <libloragw.so> <web-root> <output-dir>" >&2
	exit 2
fi

binary=$(realpath "$1")
hal=$(realpath "$2")
web_root=$(realpath "$3")
output_dir=$(realpath -m "$4")
architecture=${PACKAGE_ARCHITECTURE:-$(dpkg --print-architecture)}
version=${PACKAGE_VERSION:-$(./bin/buildinfo.py deb)}
package_root=$(mktemp -d)
chmod 0755 "${package_root}"

cleanup() {
	rm -rf -- "${package_root}"
}
trap cleanup EXIT

dpkg --validate-archname "${architecture}"
dpkg --validate-version "${version}"

install -D -m 0755 "${binary}" "${package_root}/usr/bin/meshtasticd"
install -D -m 0755 bin/meshtasticd-start.sh "${package_root}/usr/bin/meshtasticd-start.sh"
install -D -m 0755 bin/meshtasticd-sx1302-setup "${package_root}/usr/sbin/meshtasticd-sx1302-setup"
install -D -m 0644 "${hal}" "${package_root}/usr/lib/meshtasticd/libloragw.so"
install -D -m 0644 bin/config-dist.yaml "${package_root}/etc/meshtasticd/config.yaml"
install -D -m 0644 bin/meshtasticd.service "${package_root}/lib/systemd/system/meshtasticd.service"
install -D -m 0644 bin/99-meshtasticd-udev.rules "${package_root}/lib/udev/rules.d/99-meshtasticd-udev.rules"
install -d -m 0755 "${package_root}/etc/meshtasticd/available.d" "${package_root}/etc/meshtasticd/config.d"
install -d -m 0755 "${package_root}/usr/share/meshtasticd/web" "${package_root}/var/lib/meshtasticd"
cp -a bin/config.d/. "${package_root}/etc/meshtasticd/available.d/"
cp -a "${web_root}/." "${package_root}/usr/share/meshtasticd/web/"
find "${package_root}/etc/meshtasticd/available.d" "${package_root}/usr/share/meshtasticd/web" -type d -exec chmod 0755 {} +
find "${package_root}/etc/meshtasticd/available.d" "${package_root}/usr/share/meshtasticd/web" -type f -exec chmod 0644 {} +

strip --strip-unneeded "${package_root}/usr/bin/meshtasticd" "${package_root}/usr/lib/meshtasticd/libloragw.so"

if shlibs=$(dpkg-shlibdeps --ignore-missing-info -O -e"${binary}" -e"${hal}"); then
	dependencies=${shlibs#shlibs:Depends=}
else
	echo "dpkg-shlibdeps could not resolve every runtime library; falling back to package ownership" >&2
	dependencies=$(
		while IFS= read -r library; do
			owner_line=$(dpkg-query --search "$(readlink -f "${library}")" 2>/dev/null | head -n 1 || true)
			owner=${owner_line%%: /*}
			owner=${owner%:"${architecture}"}
			[[ -n ${owner} ]] && printf '%s\n' "${owner}"
		done < <(
			{
				ldd "${binary}" || true
				ldd "${hal}" || true
			} |
				awk '$2 == "=>" && $3 ~ /^\// { print $3 } $1 ~ /^\// { print $1 }' |
				sort -u || true
		)
	)
	dependencies=$(printf '%s\n' "${dependencies}" | sort -u | paste -sd, -)
fi

install -d -m 0755 "${package_root}/DEBIAN"
cat >"${package_root}/DEBIAN/control" <<EOF
Package: meshtasticd-sx1302
Version: ${version}
Architecture: ${architecture}
Maintainer: Gerard780 <gerard780@users.noreply.github.com>
Depends: adduser, ${dependencies}
Provides: meshtasticd
Conflicts: meshtasticd
Replaces: meshtasticd
Section: net
Priority: optional
Homepage: https://github.com/gerard780/meshtastic-firmware-sx1302
Description: Meshtastic daemon with SX1302 concentrator support
 Prebuilt Meshtastic Portduino daemon and patched Semtech SX1302 HAL for
 64-bit Raspberry Pi OS.
EOF

cat >"${package_root}/DEBIAN/conffiles" <<'EOF'
/etc/meshtasticd/config.yaml
EOF

cat >"${package_root}/DEBIAN/postinst" <<'EOF'
#!/bin/sh
set -e

getent group spi >/dev/null || addgroup --system spi
getent group gpio >/dev/null || addgroup --system gpio
getent group meshtasticd >/dev/null || addgroup --system meshtasticd
getent passwd meshtasticd >/dev/null || \
    adduser --system --ingroup meshtasticd --home /var/lib/meshtasticd --no-create-home meshtasticd

for group in spi gpio plugdev dialout i2c video audio input; do
    if getent group "${group}" >/dev/null; then
        adduser meshtasticd "${group}" >/dev/null 2>&1 || true
    fi
done

chown -R meshtasticd:meshtasticd /var/lib/meshtasticd
systemctl daemon-reload >/dev/null 2>&1 || true
EOF

cat >"${package_root}/DEBIAN/prerm" <<'EOF'
#!/bin/sh
set -e

if [ "$1" = remove ]; then
    systemctl disable --now meshtasticd.service >/dev/null 2>&1 || true
fi
EOF

cat >"${package_root}/DEBIAN/postrm" <<'EOF'
#!/bin/sh
set -e

systemctl daemon-reload >/dev/null 2>&1 || true
EOF

chmod 0755 "${package_root}/DEBIAN/postinst" "${package_root}/DEBIAN/prerm" "${package_root}/DEBIAN/postrm"
mkdir -p "${output_dir}"
output="${output_dir}/meshtasticd-sx1302_${version}_${architecture}.deb"
dpkg-deb --root-owner-group --build "${package_root}" "${output}"
echo "Built ${output}"
