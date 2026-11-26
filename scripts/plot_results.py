#!/usr/bin/env python3
"""Draw the experiment figures as SVG files in docs/figures (Python 3 standard library only).

Reads results/raw/*.json produced by scripts/run_experiments.py and writes:
  runtime_vs_size.svg      total execution time per dataset size (mean over seeds)
  memory_vs_size.svg       peak memory per dataset size
  states_naive_vs_pruned.svg  states explored vs naive baseline (log scale)
  replan_time_per_event.svg   re-planning time per event, incremental vs full re-solve
  convergence.svg          ALNS best/current cost over iterations (cycle 0)
  optimality_gap.svg       gap to exact B&B per method (small instances)
  routing_settled_<size>.svg  average settled nodes per routing algorithm and profile
  scheduler_comparison.svg score per scheduling method
  ablation.svg             score with each hybrid-pipeline stage switched off
Missing inputs are skipped with a message; no value is ever invented.
"""
import csv
import glob
import json
import math
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAW = os.path.join(ROOT, "results", "raw")
FIG = os.path.join(ROOT, "docs", "figures")
SIZES = ["small", "medium", "large", "extreme"]
PALETTE = ["#2f6db3", "#d1495b", "#3c9d5d", "#e69f00", "#7b5ea7", "#4d4d4d", "#56b4e9", "#cc79a7"]


def esc(s):
    return str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


class Chart:
    """Tiny SVG chart: line or grouped bars, optional log y axis."""

    def __init__(self, title, xlabel, ylabel, logy=False, w=720, h=420):
        self.title, self.xlabel, self.ylabel, self.logy = title, xlabel, ylabel, logy
        self.w, self.h = w, h
        self.ml, self.mr, self.mt, self.mb = 80, 170, 50, 60
        self.series = []  # (name, [(x, y)], kind)
        self.categories = None

    def line(self, name, pts):
        self.series.append((name, [(x, y) for x, y in pts if y is not None], "line"))

    def bars(self, categories, name, values):
        self.categories = categories
        self.series.append((name, list(zip(range(len(categories)), values)), "bar"))

    def _ty(self, y, lo, hi):
        if self.logy:
            y, lo, hi = math.log10(max(y, 1e-12)), math.log10(max(lo, 1e-12)), math.log10(max(hi, 1e-12))
        if hi - lo < 1e-12:
            hi = lo + 1
        ph = self.h - self.mt - self.mb
        return self.mt + ph - (y - lo) / (hi - lo) * ph

    def save(self, path):
        ys = [y for _, pts, _ in self.series for _, y in pts if y is not None]
        if not ys:
            print("  skip %s (no data)" % os.path.basename(path))
            return
        lo = min(ys) if self.logy else min(0.0, min(ys))
        hi = max(ys)
        if self.logy:
            lo = 10 ** math.floor(math.log10(max(lo, 1e-12)))
            hi = 10 ** math.ceil(math.log10(max(hi, 1e-12)))
        else:
            hi = hi * 1.1 if hi > 0 else 1
        pw = self.w - self.ml - self.mr
        out = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" font-family="sans-serif" font-size="12">'
               % (self.w, self.h), '<rect width="100%" height="100%" fill="white"/>',
               '<text x="%d" y="24" font-size="15" font-weight="bold">%s</text>' % (self.ml, esc(self.title))]
        # y grid + ticks
        ticks = []
        if self.logy:
            e = math.log10(lo)
            while e <= math.log10(hi) + 1e-9:
                ticks.append(10 ** e)
                e += 1
        else:
            for i in range(6):
                ticks.append(lo + (hi - lo) * i / 5)
        for t in ticks:
            y = self._ty(t, lo, hi)
            out.append('<line x1="%d" x2="%d" y1="%.1f" y2="%.1f" stroke="#ddd"/>' % (self.ml, self.ml + pw, y, y))
            label = ("%.0e" % t) if self.logy else ("%.3g" % t)
            out.append('<text x="%d" y="%.1f" text-anchor="end">%s</text>' % (self.ml - 6, y + 4, label))
        out.append('<line x1="%d" x2="%d" y1="%d" y2="%d" stroke="black"/>'
                   % (self.ml, self.ml + pw, self.h - self.mb, self.h - self.mb))
        out.append('<line x1="%d" x2="%d" y1="%d" y2="%d" stroke="black"/>' % (self.ml, self.ml, self.mt, self.h - self.mb))
        # x axis
        if self.categories is not None:
            n = len(self.categories)
            slot = pw / max(1, n)
            nb = sum(1 for s in self.series if s[2] == "bar")
            bw = slot * 0.8 / max(1, nb)
            for i, c in enumerate(self.categories):
                out.append('<text x="%.1f" y="%d" text-anchor="middle">%s</text>'
                           % (self.ml + slot * (i + 0.5), self.h - self.mb + 18, esc(c)))
            bi = 0
            for si, (name, pts, kind) in enumerate(self.series):
                if kind != "bar":
                    continue
                for x, y in pts:
                    if y is None:
                        continue
                    x0 = self.ml + slot * x + slot * 0.1 + bi * bw
                    y0 = self._ty(y, lo, hi)
                    base = self._ty(lo if self.logy else 0, lo, hi)
                    out.append('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="%s"/>'
                               % (x0, y0, bw * 0.95, max(0.5, base - y0), PALETTE[si % len(PALETTE)]))
                bi += 1
        else:
            xs = [x for _, pts, _ in self.series for x, _ in pts]
            xlo, xhi = min(xs), max(xs)
            if xhi - xlo < 1e-12:
                xhi = xlo + 1

            def tx(x):
                return self.ml + (x - xlo) / (xhi - xlo) * pw

            for i in range(6):
                xv = xlo + (xhi - xlo) * i / 5
                out.append('<text x="%.1f" y="%d" text-anchor="middle">%.3g</text>' % (tx(xv), self.h - self.mb + 18, xv))
            for si, (name, pts, kind) in enumerate(self.series):
                col = PALETTE[si % len(PALETTE)]
                if len(pts) > 1:
                    d = " ".join("%s%.1f,%.1f" % ("M" if i == 0 else "L", tx(x), self._ty(y, lo, hi))
                                 for i, (x, y) in enumerate(sorted(pts)))
                    out.append('<path d="%s" fill="none" stroke="%s" stroke-width="2"/>' % (d, col))
                if len(pts) <= 60:
                    for x, y in pts:
                        out.append('<circle cx="%.1f" cy="%.1f" r="3" fill="%s"/>' % (tx(x), self._ty(y, lo, hi), col))
        out.append('<text x="%.1f" y="%d" text-anchor="middle">%s</text>' % (self.ml + pw / 2, self.h - 15, esc(self.xlabel)))
        out.append('<text transform="translate(18,%.1f) rotate(-90)" text-anchor="middle">%s</text>'
                   % (self.mt + (self.h - self.mt - self.mb) / 2, esc(self.ylabel)))
        for si, (name, _, _) in enumerate(self.series):
            y = self.mt + 10 + 20 * si
            out.append('<rect x="%d" y="%d" width="12" height="12" fill="%s"/>' % (self.w - self.mr + 15, y - 10,
                                                                                    PALETTE[si % len(PALETTE)]))
            out.append('<text x="%d" y="%d">%s</text>' % (self.w - self.mr + 32, y, esc(name)))
        out.append("</svg>")
        with open(path, "w") as f:
            f.write("\n".join(out))
        print("  wrote %s" % os.path.relpath(path, ROOT))


def load_runs():
    runs = []
    for p in sorted(glob.glob(os.path.join(RAW, "*.json"))):
        if os.path.basename(p).startswith("bench_"):
            continue
        with open(p) as f:
            runs.append(json.load(f))
    return runs


def mean(v):
    v = [x for x in v if x is not None]
    return sum(v) / len(v) if v else None


def by(runs, exp, size, key, extra=False):
    vals = [(r["stats"]["extra"] if extra else r["stats"]).get(key) for r in runs
            if r["experiment"] == exp and r["size"] == size]
    return mean(vals)


def main():
    os.makedirs(FIG, exist_ok=True)
    runs = load_runs()
    sizes = [s for s in SIZES if any(r["size"] == s for r in runs)]
    if runs:
        c = Chart("Total execution time vs dataset size", "dataset", "time (ms, mean over seeds)", logy=True)
        c.bars(sizes, "default pipeline", [by(runs, "scaling", s, "total_exec_ms") for s in sizes])
        c.save(os.path.join(FIG, "runtime_vs_size.svg"))
        c = Chart("Peak memory vs dataset size", "dataset", "peak RSS (MB)", logy=True)
        c.bars(sizes, "default pipeline", [by(runs, "scaling", s, "peak_memory_mb") for s in sizes])
        c.save(os.path.join(FIG, "memory_vs_size.svg"))
        c = Chart("States explored: ours vs naive baseline", "dataset", "states (log)", logy=True)
        c.bars(sizes, "naive (sampled Dijkstra + all slots + full re-solve)",
               [by(runs, "scaling", s, "naive_states", True) for s in sizes])
        c.bars(sizes, "explored (pruned)", [by(runs, "scaling", s, "states_explored") for s in sizes])
        c.save(os.path.join(FIG, "states_naive_vs_pruned.svg"))
        meths = ["sched_greedy", "sched_alns", "sched_ga", "sched_sa"]
        ss = [s for s in ("small", "medium") if s in sizes]
        if ss:
            c = Chart("Civilization score per scheduling method", "dataset", "score")
            for m in meths:
                c.bars(ss, m.replace("sched_", ""), [by(runs, m, s, "total_score") for s in ss])
            c.save(os.path.join(FIG, "scheduler_comparison.svg"))
            c = Chart("Hybrid pipeline ablation", "dataset", "score")
            for m in ["ablation_full_hybrid", "ablation_no_regret", "ablation_no_alns", "ablation_no_ls",
                      "ablation_no_repair"]:
                c.bars(ss, m.replace("ablation_", ""), [by(runs, m, s, "total_score") for s in ss])
            c.save(os.path.join(FIG, "ablation.svg"))
    # re-planning time per event
    c = Chart("Re-planning time per event (small, seed 1)", "event index", "ms", logy=True)
    for mode in ("incremental", "full"):
        p = os.path.join(RAW, "replan_%s_small_s1_replanning_log.csv" % mode)
        if os.path.exists(p):
            with open(p) as f:
                pts = [(i, max(1e-3, float(r["ms"]))) for i, r in enumerate(csv.DictReader(f))]
            c.line(mode, pts)
    c.save(os.path.join(FIG, "replan_time_per_event.svg"))
    # ALNS convergence
    c = Chart("ALNS convergence (cycle 0)", "iteration", "objective")
    for size in ("small", "medium"):
        p = os.path.join(RAW, "sched_alns_%s_s1_convergence.csv" % size)
        if os.path.exists(p):
            with open(p) as f:
                rows = list(csv.DictReader(f))
            c.line("%s best" % size, [(float(r["iteration"]), float(r["best_cost"])) for r in rows])
    c.save(os.path.join(FIG, "convergence.svg"))
    # optimality gap
    p = os.path.join(RAW, "bench_gap_small_s1.json")
    if os.path.exists(p):
        with open(p) as f:
            g = json.load(f)
        inst = g.get("instances", [])
        meths = ["regret_alns", "greedy", "ga", "sa"]
        c = Chart("Gap to exact branch and bound (small instances)", "method", "mean gap (%)")
        c.bars(meths, "mean gap %", [mean([i.get(m + "_gap_pct") for i in inst]) for m in meths])
        c.save(os.path.join(FIG, "optimality_gap.svg"))
    # routing algorithms
    for size in SIZES:
        p = os.path.join(RAW, "bench_routing_%s_s1.json" % size)
        if not os.path.exists(p):
            continue
        with open(p) as f:
            b = json.load(f)
        res = b.get("results", [])
        algos = ["dijkstra", "bidirectional", "alt_astar"]
        profiles = sorted({r["profile"] for r in res})
        c = Chart("Settled nodes per query (%s)" % size, "profile", "avg settled (log)", logy=True)
        for a in algos:
            c.bars(profiles, a, [mean([r["avg_settled"] for r in res if r["profile"] == pr and r["algorithm"] == a])
                                 for pr in profiles])
        c.save(os.path.join(FIG, "routing_settled_%s.svg" % size))


if __name__ == "__main__":
    main()
