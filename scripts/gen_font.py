#!/usr/bin/env python3
"""Convert font8x8_basic.h (public domain, Daniel Hepper) into font.hex.

Output: 1024 lines, one byte per line, index = {char[6:0], row[2:0]}.
Bit order is flipped so bit 7 is the leftmost pixel.
"""
import re, sys

src, dst = sys.argv[1], sys.argv[2]
rows = re.findall(r"\{\s*(0x[^}]*)\}\s*,?\s*//\s*U\+00([0-7][0-9A-F])", open(src).read())
assert len(rows) == 128, len(rows)
GLYPHS = {  # custom UI glyphs, bit 7 = leftmost pixel
    1: [0x3C,0x66,0xDB,0xDB,0xC3,0xDB,0x7E,0x3C],   # circled A
    2: [0x3C,0x46,0xDB,0xC7,0xDB,0xC7,0x7E,0x3C],   # circled B
    3: [0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00],   # middle dot
    4: [0x00,0x01,0x03,0x06,0x8C,0xD8,0x70,0x20],   # check
    5: [0x00,0xC6,0x6C,0x38,0x38,0x6C,0xC6,0x00],   # cross
    6: [0x18,0x3C,0x7E,0x18,0x18,0x7E,0x3C,0x18],   # up/down
}
out = []
for i, (body, _) in enumerate(rows):
    for r, b in enumerate([int(x, 16) for x in body.split(",")]):
        out.append(GLYPHS[i][r] if i in GLYPHS else int(f"{b:08b}"[::-1], 2))
with open(dst, "w") as f:
    f.write("".join("%02x\n" % v for v in out))
