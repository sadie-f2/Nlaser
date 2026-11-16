# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Sadie Forbes
#
# The raster fixtures (R01-R03) as 8-bit greyscale PNGs, standard library only.
#   python3 -I raster_images.py OUTDIR

import struct
import sys
import zlib


def png(path, width, height, rows):
    def chunk(kind, data):
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    raw = b"".join(b"\x00" + bytes(r) for r in rows)  # filter type 0 per row
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)  # 8-bit greyscale
    # pHYs: 254 DPI = 10000 pixels per metre, so the image imports at 50 x 10 mm.
    phys = struct.pack(">IIB", 10000, 10000, 1)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", ihdr))
        f.write(chunk(b"pHYs", phys))
        f.write(chunk(b"IDAT", zlib.compress(raw, 9)))
        f.write(chunk(b"IEND", b""))


def main(out):
    w, h = 500, 100  # 50 x 10 mm at 254 DPI
    # R01/R02: a 256-step wedge, black (full power) at the left to white.
    wedge = [[255 * x // (w - 1) for x in range(w)] for _ in range(h)]
    png(f"{out}/R01-R02-wedge.png", w, h, wedge)
    # R03: 1-pixel black lines every 4 pixels, across a white field.
    lines = [[0 if (x % 4 == 0) else 255 for x in range(w)] for _ in range(h)]
    png(f"{out}/R03-lines.png", w, h, lines)
    print(f"wrote {out}/R01-R02-wedge.png, {out}/R03-lines.png")


if __name__ == "__main__":
    main(sys.argv[1])
