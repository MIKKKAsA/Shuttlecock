# -*- coding: utf-8 -*-
from PIL import Image
src=r"F:\Project of doubao\毽球训练\发球\毽球协会图标(1).png"
dst=r"F:\Project of doubao\毽球训练\发球\毽球协会图标_水印.png"
im=Image.open(src).convert("RGBA")
w,h=im.size; px=im.load()
for y in range(h):
    for x in range(w):
        r,g,b,a=px[x,y]
        if x>w*0.78 and y>h*0.89:      # 去掉右下角生成字样
            px[x,y]=(r,g,b,0);continue
        lum=(r+g+b)/3
        na=int(255-max(0,min(255,(lum-100)*1.55)))  # 越白越透明
        px[x,y]=(r,g,b,na)
im.save(dst)
print("saved",dst,im.size)
