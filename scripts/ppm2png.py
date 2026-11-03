#!/usr/bin/env python3
"""ppm2png.py in.ppm out.png [scale] - tiny dependency-free converter for screen sims."""
import sys, zlib, struct
src, dst = sys.argv[1], sys.argv[2]
s = int(sys.argv[3]) if len(sys.argv) > 3 else 2
tok = open(src).read().split(); w, h = int(tok[1]), int(tok[2]); px = list(map(int, tok[4:]))
raw = b''
for y in range(h):
    row = bytes(px[(y*w)*3:(y*w+w)*3]); row = b''.join(row[x*3:x*3+3]*s for x in range(w)); raw += (b'\x00' + row) * s
def chunk(t, d): return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t+d) & 0xffffffff)
open(dst,'wb').write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w*s, h*s, 8, 2, 0, 0, 0)) + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))
