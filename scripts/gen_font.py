#!/usr/bin/env python3
"""Convert font8x8_basic.h (public domain, Daniel Hepper) into font.hex.

Output: 1024 lines, one byte per line, index = {char[6:0], row[2:0]}.
Bit order is flipped so bit 7 is the leftmost pixel.
"""
import re, sys

src, dst = sys.argv[1], sys.argv[2]
rows = re.findall(r"\{\s*(0x[^}]*)\}\s*,?\s*//\s*U\+00([0-7][0-9A-F])", open(src).read())
assert len(rows) == 128, len(rows)
with open(dst, "w") as f:
    for body, _ in rows:
        for b in [int(x, 16) for x in body.split(",")]:
            f.write("%02x\n" % int(f"{b:08b}"[::-1], 2))
