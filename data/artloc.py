ASCII = True
filename = "track_art_ascii.txt" if ASCII else "track_art.txt"

with open(filename) as f:
    art = f.readlines()

arrows = "<>^v" if ASCII else "→←↓↑↖↗↘↙"

sensor_locs = []
switch_locs = []
center_switch_locs = []

def foo(name):
    for arrow in arrows:
        txt = name + arrow
        for ri, row in enumerate(art):
            if (col := row.find(txt)) != -1:
                sensor_locs.append((ri, col, len(name)))
                return
    raise ValueError(f"{name} not found in art")

def bar(n):
    s = str(n)
    l = len(s)
    if l < 3:
        txt = " " + s + " "
        o = 1
        locs = switch_locs
    else:
        txt = s
        o = 0
        locs = center_switch_locs
    for ri, row in enumerate(art):
        if (col := row.find(txt)) != -1:
            locs.append((ri, col + o, l))
            return
    raise ValueError(f"{s} not found in art")

for i in range(80):
    q, r = divmod(i, 16)
    foo(chr(ord('A') + q) + str(r + 1))

for i in range(18):
    bar(i + 1)

for i in range(4):
    bar(i + 153)

print("constexpr ArtLoc SENSOR_LOCS[80] = {", end="")
for r, c, l in sensor_locs:
    print(f"{{{r},{c},{l}}},", end="")
print("};")

print("constexpr ArtLoc SWITCH_LOCS[18] = {", end="")
for r, c, l in switch_locs:
    print(f"{{{r},{c},{l}}},", end="")
print("};")

print("constexpr ArtLoc CENTER_SWITCH_LOCS[4] = {", end="")
for r, c, l in center_switch_locs:
    print(f"{{{r},{c},{l}}},", end="")
print("};")
