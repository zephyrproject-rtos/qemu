#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Build qemu-system-rh850 and pack it as qemu-rh850-linux-<arch>.tar.gz.
# The archive holds bin/qemu-system-rh850 of a /opt/qemu-rh850 install.
#
# usage: scripts/rh850-release.sh [build-dir] [output-dir]

set -e

src=$(cd "$(dirname "$0")/.." && pwd)
build=${1:-$PWD/build-rh850-release}
out=${2:-$PWD}
prefix=/opt/qemu-rh850
name=qemu-rh850-linux-$(uname -m).tar.gz

mkdir -p "$build" "$out"
out=$(cd "$out" && pwd)
cd "$build"
"$src/configure" --target-list=rh850-softmmu --prefix=$prefix \
    --without-default-features --disable-docs --disable-tools \
    --disable-install-blobs --enable-strip
make -j"$(nproc)"
rm -rf stage
make install DESTDIR="$build/stage"
tar -czf "$out/$name" --owner=0 --group=0 -C "stage$prefix" bin/qemu-system-rh850
echo "$out/$name"
