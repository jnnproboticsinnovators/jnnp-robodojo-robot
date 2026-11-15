#!/usr/bin/env python3
"""Turn the saved ROS map (~/robodojo_field.pgm) into a PNG picture in your Windows Downloads folder."""
import sys, zlib, struct, glob, os
src = os.path.expanduser('~/robodojo_field.pgm')
data = open(src, 'rb').read()
fields, pos = [], 0
while len(fields) < 4:                     # P5, width, height, maxval (skip # comments)
    while data[pos:pos+1].isspace(): pos += 1
    if data[pos:pos+1] == b'#':
        pos = data.index(b'\n', pos) + 1; continue
    end = pos
    while not data[end:end+1].isspace(): end += 1
    fields.append(data[pos:end]); pos = end
pos += 1
w, h = int(fields[1]), int(fields[2])
pix = data[pos:pos + w * h]
raw = b''.join(b'\x00' + pix[y*w:(y+1)*w] for y in range(h))
def chunk(t, d): return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0)) + chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b'')
dl = sorted(glob.glob('/mnt/c/Users/*/Downloads'), key=os.path.getmtime)
out = os.path.join(dl[-1] if dl else os.path.expanduser('~'), 'robodojo_field.png')
open(out, 'wb').write(png)
print('Map picture saved:', out, f'({w} x {h})')
