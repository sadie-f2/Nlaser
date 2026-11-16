#!/bin/sh
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes
#
# Draws every fixture in docs/lightburn-fixtures.md with NotoCAD, one DXF per
# fixture, plus the raster test images, into OUTDIR -- a folder ready to copy to
# a thumb drive.
#
#   tools/fixtures/make_lightburn_fixtures.sh [OUTDIR] [path/to/ncad]
#
# Defaults: OUTDIR=build/lightburn-fixtures, ncad from ../NotoCAD/build.

set -eu
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
out=${1:-$root/build/lightburn-fixtures}
ncad=${2:-$root/../NotoCAD/build/src/app/ncad}

if [ ! -x "$ncad" ]; then
    echo "ncad not found at $ncad (build NotoCAD, or pass its path)" >&2
    exit 1
fi
mkdir -p "$out"

make() {
    "$ncad" -q "$here/lightburn_fixtures.lsp" -e "($1 \"$out/$2.dxf\")"
}

make F01 F01-baseline
make F02 F02-passes
make F03 F03-start-absolute
make F04 F04-start-user-origin
make F05 F05-origin-centre
make F06 F06-origin-corners
make F07 F07-circles
make F08 F08-spline-arc
make F09 F09-segments
make F10 F10-nesting
make F11 F11-three-layers
make F12 F12-aci-colours
make F13 F13-min-max-power
make F14 F14-tube-2

python3 -I "$here/raster_images.py" "$out"
cp "$here/INSTRUCTIONS.txt" "$out/"
echo "fixtures in $out"
