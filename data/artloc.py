from pathlib import Path

ASCII = True
# filename = "track_art_ascii.txt" if ASCII else "track_art.txt"
filename = "track_art_ascii_rotated.txt" if ASCII else "track_art.txt"

art = (Path(__file__).resolve().parent / filename).read_text().splitlines()

arrows = "<>^v" if ASCII else "→←↓↑↖↗↘↙"

regular_switch_markers = {
    1: (">", "v"),
    2: (">", "v"),
    3: (">", "v"),
    4: (">", "^"),
    5: ("v", "<"),
    6: ("^", "<"),
    7: ("^", ">"),
    8: (">", "^"),
    9: (">", "v"),
    10: ("^", ">"),
    11: ("^", ">"),
    12: ("^", ">"),
    13: ("^", "<"),
    14: ("<", "v"),
    15: ("<", "^"),
    16: ("v", "<"),
    17: ("v", ">"),
    18: ("v", ">"),
}
center_switch_markers = {
    153: ("C", "S"),
    154: ("C", "S"),
    155: ("C", "S"),
    156: ("C", "S"),
}

sensor_locs = []
switch_locs = []
center_switch_locs = []
switch_loc_by_id = {}
center_switch_loc_by_id = {}


def find_sensor(name):
    for arrow in arrows:
        txt = name + arrow
        for ri, row in enumerate(art):
            if (col := row.find(txt)) != -1:
                sensor_locs.append((ri, col, len(txt)))
                return
    raise ValueError(f"{name} not found in art")


def find_switch(n):
    s = str(n)
    l = len(s)
    if l < 3:
        txt = " " + s + " "
        offset = 1
        locs = switch_locs
        loc_by_id = switch_loc_by_id
    else:
        txt = s
        offset = 0
        locs = center_switch_locs
        loc_by_id = center_switch_loc_by_id
    for ri, row in enumerate(art):
        if (col := row.find(txt)) != -1:
            loc = (ri, col + offset, l)
            locs.append(loc)
            loc_by_id[n] = loc
            return
    raise ValueError(f"{s} not found in art")


def is_standalone_marker(row, col):
    return (col == 0 or row[col - 1] == " ") and (col + 1 == len(row) or row[col + 1] == " ")


def collect_marker_locs():
    locs = {marker: [] for marker in set(arrows) | {"S", "C"}}

    for ri, row in enumerate(art):
        for ci, ch in enumerate(row):
            if ch in arrows and is_standalone_marker(row, ci):
                locs[ch].append((ri, ci, 1))

        start = 0
        while (ci := row.find("SC", start)) != -1:
            locs["S"].append((ri, ci, 1))
            locs["C"].append((ri, ci + 1, 1))
            start = ci + 2

    return locs


marker_locs = collect_marker_locs()


def marker_distance(origin, candidate):
    origin_center = origin[1] + (origin[2] - 1) / 2
    candidate_center = candidate[1] + (candidate[2] - 1) / 2
    dr = abs(origin[0] - candidate[0])
    dc = abs(origin_center - candidate_center)
    return (dr + dc, dr * dr + dc * dc, candidate[0], candidate[1])


def find_closest_marker(origin, marker):
    candidates = marker_locs[marker]
    if not candidates:
        raise ValueError(f"{marker} marker not found in art")
    return min(candidates, key=lambda candidate: marker_distance(origin, candidate))


for i in range(80):
    q, r = divmod(i, 16)
    find_sensor(chr(ord("A") + q) + str(r + 1))

for i in range(18):
    find_switch(i + 1)

for i in range(4):
    find_switch(i + 153)

switch_state_locs = []
for switch_id in range(1, 19):
    curved_marker, straight_marker = regular_switch_markers[switch_id]
    switch_loc = switch_loc_by_id[switch_id]
    switch_state_locs.append(
        (
            find_closest_marker(switch_loc, curved_marker),
            find_closest_marker(switch_loc, straight_marker),
        )
    )

center_switch_state_locs = []
for switch_id in range(153, 157):
    curved_marker, straight_marker = center_switch_markers[switch_id]
    switch_loc = center_switch_loc_by_id[switch_id]
    center_switch_state_locs.append(
        (
            find_closest_marker(switch_loc, curved_marker),
            find_closest_marker(switch_loc, straight_marker),
        )
    )


def print_art_loc_array(name, locs):
    print(f"constexpr ArtLoc {name}[{len(locs)}] = {{", end="")
    for r, c, l in locs:
        print(f"{{{r}, {c}, {l}}},", end="")
    print("};")


def print_switch_state_loc_array(name, locs):
    print(f"constexpr ArtLoc {name}[{len(locs)}][2] = {{", end="")
    for curved, straight in locs:
        print(
            f"{{{{{curved[0]}, {curved[1]}, {curved[2]}}},{{{straight[0]}, {straight[1]}, {straight[2]}}}}},",
            end=""
        )
    print("};")


print_art_loc_array("SENSOR_LOCS", sensor_locs)
print_art_loc_array("SWITCH_LOCS", switch_locs)
print_art_loc_array("CENTER_SWITCH_LOCS", center_switch_locs)
print_switch_state_loc_array("SWITCH_STATE_LOCS", switch_state_locs)
print_switch_state_loc_array("CENTER_SWITCH_STATE_LOCS", center_switch_state_locs)
