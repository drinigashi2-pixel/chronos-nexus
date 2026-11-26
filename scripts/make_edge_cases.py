#!/usr/bin/env python3
"""Build edge-case / malformed variants of a dataset to check the planner never crashes.

usage: python3 scripts/make_edge_cases.py <base_dataset_dir> <out_root>
Then run build/chronos on every <out_root>/<case> (see README, Tests): every case must finish
with exit code 0 and a finite score. Cases: CRLF+BOM, shuffled/quoted columns, each optional
file missing, header-only files, single planet, garbage values, dangling references, degenerate
ships/missions, disconnected graph, event storm, duplicates, ragged rows.
Each case is a directory under out_root.
"""
import csv
import os
import random
import shutil
import sys

base, root = sys.argv[1], sys.argv[2]
FILES = ["planets.csv", "routes.csv", "spacecraft.csv", "missions.csv", "infrastructure.csv", "events.csv",
         "config.csv"]
rng = random.Random(7)


def read(name):
    p = os.path.join(base, name)
    with open(p, newline="") as f:
        return list(csv.reader(f))


def write(case, name, rows, newline="\n", bom=False):
    d = os.path.join(root, case)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, name), "w", newline="") as f:
        if bom:
            f.write("﻿")
        for r in rows:
            f.write(",".join(r) + newline)


def copy_all(case, skip=()):
    d = os.path.join(root, case)
    os.makedirs(d, exist_ok=True)
    for n in FILES:
        if n in skip:
            continue
        shutil.copy(os.path.join(base, n), os.path.join(d, n))


data = {n: read(n) for n in FILES}

# 1. Windows line endings + UTF-8 BOM on every file
for n in FILES:
    write("crlf_bom", n, data[n], "\r\n", bom=True)

# 2. shuffled columns, upper-case headers, quoted fields
for n in FILES:
    rows = data[n]
    order = list(range(len(rows[0])))
    rng.shuffle(order)
    out = [['"%s"' % rows[0][i].upper() for i in order]]
    for r in rows[1:]:
        out.append(['"%s"' % r[i] if i < len(r) else "" for i in order])
    write("shuffled_quoted", n, out)

# 3. each optional file missing
for n in FILES[1:]:
    copy_all("missing_" + n.split(".")[0], skip=(n,))

# 4. only planets
copy_all("only_planets", skip=FILES[1:])

# 5. header-only optional files
copy_all("empty_files")
for n in FILES[1:]:
    write("empty_files", n, [data[n][0]])

# 6. single planet, everything else referencing it or others
copy_all("single_planet")
write("single_planet", "planets.csv", data["planets.csv"][:2])

# 7. garbage values in every numeric field (10 % of cells)
for n in FILES:
    rows = [data[n][0]]
    for r in data[n][1:]:
        r = list(r)
        for i in range(len(r)):
            if rng.random() < 0.10:
                r[i] = rng.choice(["abc", "", "nan", "inf", "-inf", "-5", "1e308", "-1e308", "0", "1e-320"])
        rows.append(r)
    write("garbage", n, rows)

# 8. dangling references: shift every id reference by +100000 in 30 % of rows
copy_all("dangling")
for n, cols in [("routes.csv", ["from", "to"]), ("spacecraft.csv", ["location"]),
                ("missions.csv", ["origin", "destination"]), ("infrastructure.csv", ["planet"]),
                ("events.csv", ["target"])]:
    hdr = data[n][0]
    idx = [hdr.index(c) for c in cols if c in hdr]
    rows = [hdr]
    for r in data[n][1:]:
        r = list(r)
        if rng.random() < 0.3:
            for i in idx:
                r[i] = str(100000 + rng.randint(0, 999))
        rows.append(r)
    write("dangling", n, rows)

# 9. degenerate ships / missions
copy_all("degenerate")
hdr = data["spacecraft.csv"][0]
rows = [hdr]
for r in data["spacecraft.csv"][1:]:
    r = list(r)
    for c, v in [("cargo_capacity", "0"), ("fuel_capacity", "0"), ("speed", "0"), ("reliability", "0"),
                 ("operating_cost", "-3")]:
        if c in hdr and rng.random() < 0.3:
            r[hdr.index(c)] = v
    rows.append(r)
write("degenerate", "spacecraft.csv", rows)
hdr = data["missions.csv"][0]
rows = [hdr]
for r in data["missions.csv"][1:]:
    r = list(r)
    k = rng.random()
    if k < 0.15:
        r[hdr.index("destination")] = r[hdr.index("origin")]
    elif k < 0.3:
        r[hdr.index("deadline")] = "-10"
    elif k < 0.45:
        r[hdr.index("amount")] = "1e9"
    elif k < 0.6:
        r[hdr.index("priority")] = "99"
    elif k < 0.7:
        r[hdr.index("release")] = "100000"
    rows.append(r)
write("degenerate", "missions.csv", rows)

# 10. disconnected graph: keep only routes among the first half of the planets
copy_all("disconnected")
pl = [r[0] for r in data["planets.csv"][1:]]
keep = set(pl[: len(pl) // 2])
hdr = data["routes.csv"][0]
fi, ti = hdr.index("from"), hdr.index("to")
write("disconnected", "routes.csv",
      [hdr] + [r for r in data["routes.csv"][1:] if (r[fi] in keep) == (r[ti] in keep)])

# 11. event storm: every type, odd times and targets, many events
copy_all("event_storm")
types = ["WormholeCollapse", "RouteUnavailable", "PirateBlockade", "ShipDestroyed", "EmergencyMedical",
         "ProductionDrop", "NewColony", "PopulationSurge", "ResourceUnavailable", "NewMission", "RouteDanger",
         "MechanicalFailure", "UnknownType", "99", "-1"]
rows = [["time", "type", "target", "severity", "a", "b", "c", "d"]]
for k in range(400):
    t = rng.choice([-5, 0, rng.uniform(0, 336), 336, 10000, "x"])
    rows.append([str(t), rng.choice(types), str(rng.choice([-1, 0, 3, 50, 99, 1000000])),
                 str(rng.choice([0, 0.5, 1, 5, -1])), str(rng.choice([0, 1, 48, -3, 1e9])),
                 str(rng.choice([0, 1, 2, -1, 7])), "0", "0"])
write("event_storm", "events.csv", rows)

# 12. duplicates everywhere
copy_all("duplicates")
for n in ["planets.csv", "routes.csv", "spacecraft.csv", "missions.csv"]:
    write("duplicates", n, data[n] + data[n][1:])

# 13. short rows / extra columns / comments / blank lines
for n in FILES:
    rows = [data[n][0] + ["extra_col"]]
    for r in data[n][1:]:
        k = rng.random()
        if k < 0.1:
            rows.append(r[: max(1, len(r) // 2)])
        elif k < 0.2:
            rows.append(r + ["junk", "more"])
        else:
            rows.append(r + ["x"])
        if rng.random() < 0.05:
            rows.append(["# comment line"])
        if rng.random() < 0.05:
            rows.append([""])
    write("ragged", n, rows)

print("cases written to", root)
