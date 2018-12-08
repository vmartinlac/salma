import os
import shutil

def rename(a, b):
    #print(a,b)
    shutil.move(a,b)

lst = list()
for f in os.listdir("."):
    if f.endswith("left.bmp"):
        lst.append(f)

for f in lst:
    f2 = f + "_"
    g = f.replace("left", "right")
    rename(f, f2)
    rename(g,f)
    rename(f2,g)

