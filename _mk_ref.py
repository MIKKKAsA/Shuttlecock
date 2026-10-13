# -*- coding: utf-8 -*-
from PIL import Image
import os

src = r'F:\Project of doubao\毽球训练\发球\裁判形象.png'
im = Image.open(src).convert('RGB')
W, H = im.size
print('size', W, H)

# 正面人物（三视图第一个）
box = (int(W*0.03), int(H*0.07), int(W*0.35), int(H*0.95))
crop = im.crop(box)

# 白底转透明（越白越透明）
crop = crop.convert('RGBA')
px = crop.load()
w, h = crop.size
for y in range(h):
    for x in range(w):
        r, g, b, a = px[x, y]
        lum = (r + g + b) / 3
        if lum > 100:
            na = int(max(0, 255 - (lum - 100) * 1.6))
        else:
            na = 255
        px[x, y] = (r, g, b, na)

# 自动裁掉透明边
bbox = crop.getbbox()
if bbox:
    crop = crop.crop(bbox)

out = r'F:\Project of doubao\毽球训练\发球\裁判_正面.png'
crop.save(out)
print('saved', out, crop.size)
