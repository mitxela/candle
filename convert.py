#!/bin/env python3

import sys, os
from PIL import Image

if len(sys.argv) != 2:
	print("Usage: ",sys.argv[0],"dir")
	exit()

def image_to_bytes(i):
	im = Image.open(i)
	w,h = im.size
	pixels = [1 if p[0]>2 else 0 for p in list(im.getdata())]
	print("  {",end="")

	for x in range(w):
		word = 0x0401FF00 | (1<<x)

		for y in range(h):
			p = (pixels[y*8+x])
			if p:
				if y == 8:
					word &= ~(1<<16)
				elif y == 9:
					word &= ~(1<<24)
				else:
					word &= ~(1<<(y+8))
		print(f"0x{word:08X}",end = ", " if x<w-1 else "")

	print("  },")

files = [os.path.join( sys.argv[1], f) for f in os.listdir( sys.argv[1] )]
files.sort()


n=0
print(f"const uint32_t framedata[{len(files)//24}][24][8]=","{")
for f in files:
	if (n%24) == 0:
		print(" },{" if n>0 else " {")
	n+=1
	image_to_bytes(f)


print(" }\n};")

