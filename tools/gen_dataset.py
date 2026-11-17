#!/usr/bin/env python3
"""Generate a Chronos Nexus dataset (galaxy.txt + events.txt). Deterministic per seed.

  python3 tools/gen_dataset.py --size small --seed 1 --out data/small
  python3 tools/gen_dataset.py --planets 5000 --routes 40000 --missions 2000 --out data/custom

Format is documented in docs/dataset_format.md.
"""
import argparse, math, os, random, sys

PRESETS = {  # planets, routes, missions  (routes = directed edges)
    "small": (100, 500, 50), "medium": (1000, 10000, 500),
    "large": (10000, 100000, 5000), "extreme": (100000, 1000000, 50000),
}
NRES = 7  # Food Water Energy Fuel Medicine RareMinerals Construction
MEDICINE = 4
CAP_CARGO, CAP_MEDICAL, CAP_ARMED, CAP_HEAVY = 1, 2, 4, 8
PER_CAPITA = [0.002, 0.003, 0.0015, 0.0008, 0.0002, 0.0001, 0.0003]  # units/person/day
RES_WEIGHTS = [0.25, 0.20, 0.15, 0.10, 0.15, 0.08, 0.07]
HORIZON = 720.0  # hours simulated (30 days)

# type, cost, pop_impact, energy_impact, strategic, build_days, {resource: +/- per day}
FACILITIES = [
    (0, 120, 0.00,  400, 0.5, 20, {2: 500}),
    (1,  60, 0.02,  -20, 0.2, 10, {0: 300, 1: -50}),
    (2,  90, 0.03,  -30, 0.4, 15, {4: 80}),
    (3, 150, 0.00, -150, 0.5, 25, {6: 200, 5: -40}),
    (4, 110, 0.00,  -80, 0.9, 18, {}),
    (5, 130, 0.00, -100, 0.8, 30, {}),
    (6, 140, 0.01, -120, 0.6, 22, {3: 150}),
    (7, 100, 0.00,  -90, 0.3, 12, {5: 250}),
]
# event type ids (see src/model/event.hpp) and sampling weights
EVENT_WEIGHTS = [4, 8, 6, 5, 5, 6, 3, 5, 5, 6, 8, 5]


def gen_planets(rng, n, f):
    side = math.sqrt(n) * 10.0  # ~constant density -> avg neighbour spacing ~10
    xs, ys, zs = [], [], []
    f.write("PLANETS %d\n" % n)
    for i in range(n):
        x, y, z = rng.uniform(0, side), rng.uniform(0, side), rng.uniform(0, side * 0.05)
        xs.append(x); ys.append(y); zs.append(z)
        t = rng.random()
        ltype = 0 if t < 0.85 else (1 if t < 0.97 else 2)
        pop = 10 ** rng.uniform(2, 4) if ltype == 2 else 10 ** rng.uniform(3, 7.5)
        cons, prod, stock, cap = [], [], [], []
        for r in range(NRES):
            c = pop * PER_CAPITA[r] * rng.uniform(0.8, 1.2)
            producer = rng.random() < 0.35
            p = c * (rng.uniform(1.5, 4.0) if producer else rng.uniform(0.2, 1.1))
            s = c * rng.uniform(5, 60)
            cons.append(round(c, 2)); prod.append(round(p, 2)); stock.append(round(s, 2))
            cap.append(round(max(s * rng.uniform(1.5, 3.0), c * 30), 2))
        f.write("%d %d %.3f %.3f %.3f %.0f %d %.1f %.3f %s %s %s %s\n" % (
            i, ltype, x, y, z, pop, rng.randint(1, 10), rng.uniform(0, 100),
            min(1.0, rng.random() * 0.7 + (0.3 if ltype == 1 else 0.0)),
            " ".join(map(str, prod)), " ".join(map(str, cons)),
            " ".join(map(str, stock)), " ".join(map(str, cap))))
    return xs, ys, zs, side


def gen_routes(rng, n, m_target, xs, ys, zs, side, tmp_path):
    """Connected by construction: a serpentine backbone of bidirectional links, then
    random local edges, then a few wormhole shortcuts. Returns (count, wormhole_ids)."""
    side_cells = max(1, int(math.sqrt(n / 2)))
    w = side / side_cells
    cells = {}
    for i in range(n):
        cx, cy = min(int(xs[i] / w), side_cells - 1), min(int(ys[i] / w), side_cells - 1)
        cells.setdefault((cx, cy), []).append(i)

    seen = set()
    count = 0
    wormholes = []
    out = open(tmp_path, "w")

    def dist(a, b):
        return math.sqrt((xs[a]-xs[b])**2 + (ys[a]-ys[b])**2 + (zs[a]-zs[b])**2)

    def emit(a, b, wormhole=False):
        nonlocal count
        d = dist(a, b)
        if wormhole:
            d *= rng.uniform(0.1, 0.3)  # space-bending shortcut
            tm = d / rng.uniform(1.0, 2.0)
            risk = rng.uniform(0.05, 0.35)
        else:
            d *= rng.uniform(1.0, 1.3)
            tm = d / rng.uniform(0.8, 1.5)
            risk = min(0.6, rng.betavariate(1.2, 8))
        rel = max(0.5, 1 - risk * rng.uniform(0.1, 0.5))
        out.write("%d %d %.3f %.3f %.3f %.3f %.3f %.3f %.0f %.4f 1 %d\n" % (
            a, b, d, tm, tm * 0.8, tm * 1.5, d * rng.uniform(1.2, 2.0), risk,
            rng.choice([100, 200, 500, 1000, 2000]), rel, 1 if wormhole else 0))
        if wormhole:
            wormholes.append(count)
        count += 1

    # backbone in serpentine cell order
    order = []
    for cy in range(side_cells):
        row = [cells.get((cx, cy), []) for cx in range(side_cells)]
        if cy % 2: row.reverse()
        for c in row: order.extend(c)
    for a, b in zip(order, order[1:]):
        emit(a, b); emit(b, a)
        seen.add(a * n + b); seen.add(b * n + a)

    n_worm = int(m_target * 0.02)
    budget = m_target - n_worm
    attempts = 0
    while count < budget and attempts < budget * 20:
        attempts += 1
        a = rng.randrange(n)
        cx, cy = min(int(xs[a] / w), side_cells - 1), min(int(ys[a] / w), side_cells - 1)
        cand = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                cand.extend(cells.get((cx + dx, cy + dy), []))
        b = cand[rng.randrange(len(cand))]
        if a == b or (a * n + b) in seen: continue
        seen.add(a * n + b); emit(a, b)
        if rng.random() < 0.5 and (b * n + a) not in seen and count < budget:
            seen.add(b * n + a); emit(b, a)
    attempts = 0
    while len(wormholes) < n_worm and attempts < n_worm * 20 + 100:
        attempts += 1
        a, b = rng.randrange(n), rng.randrange(n)
        if a == b or (a * n + b) in seen: continue
        seen.add(a * n + b); emit(a, b, wormhole=True)
    out.close()
    return count, wormholes


def gen_spacecraft(rng, n_planets, n_ships, f):
    f.write("SPACECRAFT %d\n" % n_ships)
    for i in range(n_ships):
        cargo = rng.choice([200, 500, 1000, 2500, 5000])
        caps = CAP_CARGO
        if rng.random() < 0.25: caps |= CAP_MEDICAL
        if rng.random() < 0.15: caps |= CAP_ARMED
        if cargo >= 2500: caps |= CAP_HEAVY
        fuel_cap = rng.uniform(400, 3000)
        f.write("%d %d %d %.0f %.2f %.0f %.1f %.4f %.2f %d\n" % (
            i, rng.randrange(n_planets), cargo, fuel_cap, rng.uniform(0.7, 2.0),
            fuel_cap * rng.uniform(0.5, 1.0), rng.uniform(0, 48) if rng.random() < 0.3 else 0,
            rng.uniform(0.90, 0.999), rng.uniform(1, 10), caps))


def gen_missions(rng, n_planets, n_missions, xs, ys, f):
    f.write("MISSIONS %d\n" % n_missions)
    for i in range(n_missions):
        o = rng.randrange(n_planets)
        d = rng.randrange(n_planets - 1)
        if d >= o: d += 1
        res = rng.choices(range(NRES), RES_WEIGHTS)[0]
        amount = rng.uniform(20, 2500)
        caps = CAP_CARGO
        if res == MEDICINE: caps |= CAP_MEDICAL
        if amount > 1500: caps |= CAP_HEAVY
        prio = rng.randint(4, 5) if res == MEDICINE else rng.choices([1, 2, 3, 4, 5], [3, 4, 4, 2, 1])[0]
        release = rng.uniform(0, HORIZON * 0.6)
        est = math.hypot(xs[o] - xs[d], ys[o] - ys[d])  # ~hours at speed 1
        deadline = release + 12 + est * rng.uniform(1.5, 4.0)
        f.write("%d %d %d %d %.0f %.1f %.1f %d %.3f %d\n" % (
            i, o, d, res, amount, release, deadline, prio, rng.uniform(0.1, 1.0) * prio, caps))


def gen_facilities(rng, n_planets, f):
    f.write("BUDGET %d\n" % (n_planets * 50))
    f.write("FACILITIES %d\n" % len(FACILITIES))
    for t, cost, pop, en, strat, bt, imp in FACILITIES:
        j = rng.uniform(0.9, 1.1)
        res = [round(imp.get(r, 0) * j, 1) for r in range(NRES)]
        f.write("%d %.0f %.3f %.0f %.2f %d %s\n" % (
            t, cost * j, pop, en, strat, bt, " ".join(map(str, res))))


def gen_events(rng, n_events, n_planets, n_edges, n_ships, wormholes, side, path):
    times = sorted(rng.uniform(0, HORIZON) for _ in range(n_events))
    lines = []
    for t in times:
        et = rng.choices(range(12), EVENT_WEIGHTS)[0]
        if et == 0 and not wormholes: et = 1
        target, sev, p1, p2 = -1, rng.uniform(0.3, 1.0), 0, 0
        if et == 0: target = rng.choice(wormholes); sev = 1.0
        elif et in (1, 2, 10): target = rng.randrange(n_edges)
        elif et in (3, 11):
            target = rng.randrange(n_ships)
            p2 = rng.uniform(6, 72) if et == 11 else 0
        elif et == 4: target = rng.randrange(n_planets); p1 = MEDICINE; p2 = rng.uniform(50, 800)
        elif et == 5: target = rng.randrange(n_planets); sev = 0.6; p1 = rng.randrange(NRES)
        elif et == 6: target = -1; p1 = rng.uniform(0, side); p2 = rng.uniform(0, side)
        elif et == 7: target = rng.randrange(n_planets); sev = rng.uniform(0.2, 1.0)
        elif et == 8: target = rng.randrange(n_planets); p1 = rng.randrange(NRES); p2 = rng.uniform(12, 120)
        elif et == 9: target = rng.randrange(n_planets); p1 = rng.randrange(NRES); p2 = rng.uniform(100, 2000)
        lines.append("%.2f %d %d %.3f %.3f %.3f\n" % (t, et, target, sev, p1, p2))
    with open(path, "w") as f:
        f.write("EVENTS %d\n" % len(lines)); f.writelines(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", choices=PRESETS)
    ap.add_argument("--planets", type=int); ap.add_argument("--routes", type=int)
    ap.add_argument("--missions", type=int)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    if a.size: n, m, q = PRESETS[a.size]
    elif a.planets and a.routes and a.missions: n, m, q = a.planets, a.routes, a.missions
    else: sys.exit("give --size or all of --planets --routes --missions")
    if n < 2: sys.exit("need at least 2 planets")
    rng = random.Random(a.seed)
    os.makedirs(a.out, exist_ok=True)
    tmp = os.path.join(a.out, ".routes.tmp")
    ships = max(10, q // 2)
    with open(os.path.join(a.out, "galaxy.txt"), "w") as f:
        f.write("CHRONOS 1\n")
        xs, ys, zs, side = gen_planets(rng, n, f)
        cnt, worm = gen_routes(rng, n, m, xs, ys, zs, side, tmp)
        f.write("ROUTES %d\n" % cnt)
        with open(tmp) as t:
            for line in t: f.write(line)
        os.remove(tmp)
        gen_spacecraft(rng, n, ships, f)
        gen_missions(rng, n, q, xs, ys, f)
        gen_facilities(rng, n, f)
    gen_events(rng, max(10, q // 2), n, cnt, ships, worm, side, os.path.join(a.out, "events.txt"))
    print("wrote %s: planets=%d routes=%d (wormholes=%d) ships=%d missions=%d events=%d" % (
        a.out, n, cnt, len(worm), ships, q, max(10, q // 2)))


if __name__ == "__main__":
    main()
