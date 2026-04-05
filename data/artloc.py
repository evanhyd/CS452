from pathlib import Path

ASCII = True
# filename = "track_art_ascii.txt" if ASCII else "track_art.txt"
filename = "track_art_ascii_rotated.txt" if ASCII else "track_art.txt"

art = (Path(__file__).resolve().parent / filename).read_text().splitlines()

arrows = "<>^v" if ASCII else "→←↓↑↖↗↘↙"

regular_switch_markers = {
    1: (">", "v"),
    2: (">", "v"),
    3: ("v", ">"),
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
dot_locs = []
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


def collect_dot_locs():
    for ri, row in enumerate(art):
        for ci, ch in enumerate(row):
            if ch == "o":
                dot_locs.append((ri, ci, 1))


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


def art_loc_center_times_4(loc):
    row, col, length = loc
    return 4 * row, 4 * col + 2 * (length - 1)


def pair_origin_times_4(pair_index):
    a = art_loc_center_times_4(sensor_locs[2 * pair_index])
    b = art_loc_center_times_4(sensor_locs[2 * pair_index + 1])
    return (a[0] + b[0]) // 2, (a[1] + b[1]) // 2


def dot_cost(origin_times_4, dot_index):
    dot = dot_locs[dot_index]
    dot_center = (4 * dot[0], 4 * dot[1])
    dr = abs(origin_times_4[0] - dot_center[0])
    dc = abs(origin_times_4[1] - dot_center[1])
    primary = dr + dc
    secondary = dr * dr + dc * dc
    return primary * 1_000_000_000_000 + secondary * 1_000_000 + dot_index


def solve_assignment(costs):
    n = len(costs)
    m = len(costs[0])
    if n > m:
        raise ValueError("assignment matrix has more rows than columns")

    inf = 10**30
    u = [0] * (n + 1)
    v = [0] * (m + 1)
    p = [0] * (m + 1)
    way = [0] * (m + 1)

    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [inf] * (m + 1)
        used = [False] * (m + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = inf
            j1 = 0
            for j in range(1, m + 1):
                if used[j]:
                    continue
                cur = costs[i0 - 1][j - 1] - u[i0] - v[j]
                if cur < minv[j]:
                    minv[j] = cur
                    way[j] = j0
                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break

    assignment = [-1] * n
    for j in range(1, m + 1):
        if p[j] != 0:
            assignment[p[j] - 1] = j - 1
    return assignment


for i in range(80):
    q, r = divmod(i, 16)
    find_sensor(chr(ord("A") + q) + str(r + 1))

for i in range(18):
    find_switch(i + 1)

for i in range(4):
    find_switch(i + 153)

collect_dot_locs()
if len(dot_locs) != len(sensor_locs) // 2:
    raise ValueError(f"expected {len(sensor_locs) // 2} dots, found {len(dot_locs)}")

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

dot_assignment = solve_assignment(
    [[dot_cost(pair_origin_times_4(pair_index), dot_index) for dot_index in range(len(dot_locs))]
     for pair_index in range(len(sensor_locs) // 2)]
)
if len(set(dot_assignment)) != len(dot_assignment):
    raise ValueError("dot assignment is not bijective")

sensor_pair_dot_locs = [dot_locs[dot_index] for dot_index in dot_assignment]


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
print_art_loc_array("DOT_LOCS", sensor_pair_dot_locs)
print_switch_state_loc_array("SWITCH_STATE_LOCS", switch_state_locs)
print_switch_state_loc_array("CENTER_SWITCH_STATE_LOCS", center_switch_state_locs)
