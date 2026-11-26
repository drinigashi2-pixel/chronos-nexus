#!/usr/bin/env python3
"""Fill the measured-results tables of README.md from the experiment outputs.

usage:  python3 scripts/fill_readme.py            (after run_experiments.py)

Reads   results/summary.csv, results/raw/bench_*.json, results/runs/*/replanning_log.csv and
        cycles.csv, and results/hardware.txt (optional, see README §13).
Writes  README.md in place (a copy of the previous version goes to README.md.bak).

Every generated table is wrapped in <!-- AUTO:name --> ... <!-- /AUTO:name --> markers, so the
script can be re-run after new experiments: it replaces the marked blocks and leaves all other
text alone. Numbers are copied from the measurements only, never estimated; anything that was not
measured is written as "not run".
"""
import csv
import glob
import json
import os
import re
import shutil
import statistics as st
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
README = os.path.join(ROOT, "README.md")
RES = os.path.join(ROOT, "results")
SIZES = ["small", "medium", "large", "extreme"]


# ------------------------------------------------------------------ loading
def num(x):
    try:
        v = float(x)
        return v if v == v else None
    except (TypeError, ValueError):
        return None


def load_summary():
    path = os.path.join(RES, "summary.csv")
    if not os.path.exists(path):
        sys.exit("results/summary.csv not found: run scripts/run_experiments.py first")
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


ROWS = load_summary()


def rows(exp, size):
    return [r for r in ROWS if r["experiment"] == exp and r["size"] == size]


def vals(rs, key):
    return [v for v in (num(r.get(key)) for r in rs) if v is not None]


def bench(kind):
    out = {}
    for p in sorted(glob.glob(os.path.join(RES, "raw", "bench_%s_*.json" % kind))):
        size = os.path.basename(p).split("_")[2]
        with open(p) as f:
            out[size] = json.load(f)
    return out


def run_csv(tag, size, name):
    """All rows of <name> over the seeds of run <tag>/<size>."""
    out = []
    for d in sorted(glob.glob(os.path.join(RES, "runs", "%s_%s_s*" % (tag, size)))):
        p = os.path.join(d, name)
        if os.path.exists(p):
            with open(p, newline="") as f:
                out.extend(csv.DictReader(f))
    return out


# ------------------------------------------------------------------ formatting
def f(v, d=1):
    if v is None:
        return "—"
    if abs(v) >= 1e9:
        return "%.2fG" % (v / 1e9)
    if abs(v) >= 1e6:
        return "%.2fM" % (v / 1e6)
    if abs(v) >= 1e4:
        return "{:,.0f}".format(v)
    return ("%." + str(d) + "f") % v


def mean_rng(vs, d=1):
    """mean (min–max) over seeds; a single value is printed alone."""
    if not vs:
        return "not run"
    if len(vs) == 1 or max(vs) - min(vs) < 1e-12:
        return f(st.mean(vs), d)
    return "%s (%s–%s)" % (f(st.mean(vs), d), f(min(vs), d), f(max(vs), d))


def mean(vs, d=1):
    return f(st.mean(vs), d) if vs else "not run"


def table(header, body):
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(r) + " |" for r in body]
    return "\n".join(out)


# ------------------------------------------------------------------ tables
def t_scaling():
    body = []
    for s in SIZES:
        rs = rows("scaling", s)
        if not rs:
            body.append([s] + ["not run"] * 10)
            continue
        body.append([s, mean_rng(vals(rs, "total_score"), 2), mean(vals(rs, "population_survival_rate"), 4),
                     mean(vals(rs, "missions_completed"), 0), mean(vals(rs, "missions_failed"), 0),
                     mean(vals(rs, "states_explored"), 0), mean(vals(rs, "naive_states"), 0),
                     mean_rng(vals(rs, "search_space_reduction_pct"), 2), mean(vals(rs, "avg_replan_ms"), 2),
                     mean_rng(vals(rs, "total_exec_ms"), 0), mean(vals(rs, "peak_memory_mb"), 1)])
    return table(["size", "score", "survival", "completed", "failed", "states", "naive states", "reduction %",
                  "avg re-plan ms", "time ms", "peak MB"], body)


def t_stats():
    def cell(s, keys, d):
        rs = rows("scaling", s)
        return " / ".join(mean(vals(rs, k), d) for k in keys) if rs else "not run"
    spec = [("resources delivered / shortages / waste", ["resources_delivered", "resource_shortages", "resource_waste"], 0),
            ("total distance / fuel / transport cost / risk",
             ["total_distance", "total_fuel", "total_transport_cost", "total_risk"], 0),
            ("nodes explored / candidates evaluated", ["nodes_explored", "candidates_evaluated"], 0),
            ("events handled", ["events_handled"], 0),
            ("missions on time / expired", ["missions_on_time", "missions_expired"], 0),
            ("hard constraint violations", ["validation_hard_violations"], 0)]
    return table(["statistic"] + SIZES, [[name] + [cell(s, k, d) for s in SIZES] for name, k, d in spec])


def t_replan():
    body = []
    for s in ["small", "medium"]:
        for mode in ["incremental", "full"]:
            tag = "replan_" + mode
            rs = rows(tag, s)
            log = run_csv(tag, s, "replanning_log.csv")
            inv = [num(r["invalid"]) for r in log if num(r.get("invalid")) is not None]
            unch = [100.0 * num(r["unchanged"]) / num(r["active"]) for r in log
                    if num(r.get("active")) and num(r.get("unchanged")) is not None]
            body.append([s, mode, mean(vals(rs, "avg_replan_ms"), 3), mean(vals(rs, "max_replan_ms"), 2),
                         mean(inv, 1), mean(unch, 1), mean_rng(vals(rs, "total_score"), 2)])
    return table(["size", "mode", "avg ms/event", "max ms", "mean invalid", "mean unchanged %", "score"], body)


def t_gap():
    inst = []
    for b in bench("gap").values():
        inst += b.get("instances", [])
    names = [("regret_alns", "regret + ALNS + LS"), ("greedy", "greedy"), ("ga", "GA"), ("sa", "SA"), ("bnb", "B&B (reference)")]
    body = []
    for key, label in names:
        g = [num(i.get(key + "_gap_pct")) for i in inst if num(i.get(key + "_gap_pct")) is not None]
        ms = [num(i.get(key + "_ms")) for i in inst if num(i.get(key + "_ms")) is not None]
        body.append([label, mean(g, 2), f(max(g), 2) if g else "not run", mean(ms, 2)])
    proved = sum(1 for i in inst if i.get("bnb_proved_optimal"))
    return table(["method", "mean gap to B&B %", "max gap %", "mean ms"], body) + \
        "\n\nB&B proved optimality on %d of %d instances." % (proved, len(inst))


def routing_text():
    parts, mism = [], 0
    for s, b in bench("routing").items():
        by = {}
        for r in b.get("results", []):
            by.setdefault(r["algorithm"], []).append(r)
            mism += int(r.get("cost_mismatches_vs_dijkstra", 0))
        seg = ", ".join("%s %s settled / %s ms" % (a, f(st.mean(num(x["avg_settled"]) for x in v), 0),
                                                   f(st.mean(num(x["avg_ms"]) for x in v), 3))
                        for a, v in sorted(by.items()))
        parts.append("%s (V=%s): %s" % (s, b.get("V"), seg))
    if not parts:
        return "not run"
    return "; ".join(parts) + "; %d cost mismatches vs Dijkstra" % mism


def flow_text():
    parts = []
    for s, b in bench("flow").items():
        by = {r["method"]: r for r in b.get("results", [])}
        if "min_cost_flow" in by and "greedy" in by:
            m, g = by["min_cost_flow"], by["greedy"]
            parts.append("%s: objective MCF %s vs greedy %s, shortage units %s vs %s, %s vs %s ms" % (
                s, f(num(m["objective"])), f(num(g["objective"])), f(num(m["shortage_units"])),
                f(num(g["shortage_units"])), f(num(m["ms"]), 1), f(num(g["ms"]), 1)))
    return "; ".join(parts) or "not run"


def infra_text():
    parts = []
    for s, b in bench("infra").items():
        rs = b.get("results", [])
        ratio = [num(r["greedy_ratio"]) for r in rs]
        bb = [r for r in rs if "bnb_value" in r]
        same = sum(1 for r in bb if abs(num(r["bnb_value"]) - num(r["dp_value"])) <= 1e-6 * max(1.0, num(r["dp_value"])))
        parts.append("%s: greedy/DP value ratio %s (min %s); DP = B&B on %d of %d; DP %s ms, greedy %s ms" % (
            s, f(st.mean(ratio), 4), f(min(ratio), 4), same, len(bb),
            f(st.mean(num(r["dp_ms"]) for r in rs), 2), f(st.mean(num(r["greedy_ms"]) for r in rs), 2)))
    return "; ".join(parts) or "not run"


def compare_text(a, b, keys=("total_score", "missions_completed", "total_exec_ms")):
    parts = []
    for s in ["small", "medium"]:
        ra, rb = rows(a, s), rows(b, s)
        if ra and rb:
            parts.append("%s: %s" % (s, ", ".join("%s %s vs %s" % (k.replace("total_", "").replace("missions_", ""),
                                                                    mean(vals(ra, k), 2), mean(vals(rb, k), 2))
                                                  for k in keys)))
    return "; ".join(parts) or "not run"


def ablation_text():
    parts = []
    for s in ["small", "medium"]:
        full = vals(rows("ablation_full_hybrid", s), "total_score")
        if not full:
            continue
        segs = []
        for tag, lab in [("no_regret", "regret"), ("no_alns", "ALNS"), ("no_ls", "local search"), ("no_repair", "repair")]:
            v = vals(rows("ablation_" + tag, s), "total_score")
            if v:
                segs.append("−%s %+.2f" % (lab, st.mean(v) - st.mean(full)))
        parts.append("%s (full %s): %s" % (s, f(st.mean(full), 2), ", ".join(segs)))
    return "; ".join(parts) or "not run"


def worst_text():
    parts = []
    for s in SIZES:
        rs = rows("scaling", s)
        if not rs:
            continue
        cyc = run_csv("scaling", s, "cycles.csv")
        cms = [sum(num(r.get(k)) or 0 for k in ("alloc_ms", "sched_ms", "infra_ms")) for r in cyc]
        log = run_csv("scaling", s, "replanning_log.csv")
        esc = sum(1 for r in log if r.get("escalated") in ("1", "true"))
        parts.append("%s: max re-plan %s ms, max cycle %s ms, %d escalations" % (
            s, f(max(vals(rs, "max_replan_ms")), 1), f(max(cms), 0) if cms else "—", esc))
    return "; ".join(parts) or "not run"


def t_compare():
    body = [["ALT A\\* vs bidirectional vs Dijkstra", "avg settled / ms per query, exactness", routing_text()],
            ["MCF vs greedy allocation", "objective, shortage units, ms", flow_text()],
            ["DP vs hull greedy vs B&B (infra)", "value ratio, ms", infra_text()],
            ["GA vs ALNS (scheduler)", "score, completed, time", compare_text("sched_ga", "sched_alns")],
            ["hybrid ablation", "score change per removed stage", ablation_text()],
            ["pruning on/off", "states, time, score",
             compare_text("pruning_on", "pruning_off", ("states_explored", "total_exec_ms", "total_score"))],
            ["worst-case behaviour", "max re-plan ms, max cycle ms, escalations", worst_text()]]
    return table(["comparison", "metric", "result"], body)


def observed():
    """Observed column of the §9 table, keyed by the first cell of each row."""
    rb = bench("routing")
    pick = rb.get("medium") or next(iter(rb.values()), None)
    obs = {}
    if pick:
        by = {}
        for r in pick.get("results", []):
            by.setdefault(r["algorithm"], []).append(num(r["avg_settled"]))
        label = "%s settled/query (%s)" % ("%s", [k for k in rb if rb[k] is pick][0])
        for row, algo in [("Dijkstra", "dijkstra"), ("Bidirectional Dijkstra", "bidirectional"), ("ALT A\\* query", "alt_astar")]:
            if algo in by:
                obs[row] = label % f(st.mean(by[algo]), 0)
    fb = bench("flow")
    ph = ["%s %s" % (s, b2["phases"]) for s, b in fb.items() for b2 in b.get("results", []) if b2["method"] == "min_cost_flow"]
    if ph:
        obs["Min-cost flow (primal–dual)"] = "phases: " + ", ".join(ph)
    it = []
    for s in ["small", "medium", "large"]:
        rs = rows("scaling", s)
        a, t = vals(rs, "alns_iterations"), vals(rs, "total_exec_ms")
        if a and t:
            it.append("%s %s it/s" % (s, f(st.mean(a) / (st.mean(t) / 1000.0), 0)))
    if it:
        obs["ALNS iteration"] = ", ".join(it)
    ib = bench("infra")
    nodes = [num(r["bnb_nodes"]) for b in ib.values() for r in b.get("results", []) if "bnb_nodes" in r]
    if nodes:
        obs["Exact B&B"] = "%s nodes mean (infra B&B, max %s)" % (f(st.mean(nodes), 0), f(max(nodes), 0))
    rp = ["%s %s" % (s, mean(vals(rows("scaling", s), "avg_replan_ms"), 2)) for s in SIZES if rows("scaling", s)]
    if rp:
        obs["Incremental re-plan"] = "ms/event: " + ", ".join(rp)
    cy = []
    for s in SIZES:
        c = run_csv("scaling", s, "cycles.csv")
        if c:
            cy.append("%s %s ms" % (s, f(st.mean(sum(num(r.get(k)) or 0 for k in ("alloc_ms", "sched_ms", "infra_ms"))
                                                   for r in c), 0)))
    if cy:
        obs["Simulation cycle"] = "mean planning ms/cycle: " + ", ".join(cy)
    return obs


# ------------------------------------------------------------------ README surgery
def put_block(text, name, content, anchor_regex):
    """Replace the marked block <name>, or the first table matching anchor_regex on first use."""
    start, end = "<!-- AUTO:%s -->" % name, "<!-- /AUTO:%s -->" % name
    block = "%s\n%s\n%s" % (start, content, end)
    if start in text:
        return re.sub(re.escape(start) + r".*?" + re.escape(end), lambda _: block, text, count=1, flags=re.S)
    m = re.search(anchor_regex, text, flags=re.M)
    if not m:
        print("  warning: anchor for %s not found, skipped" % name)
        return text
    # the table runs from the header line to the first line that does not start with '|'
    lines = text[m.start():].split("\n")
    n = 0
    while n < len(lines) and lines[n].startswith("|"):
        n += 1
    old = "\n".join(lines[:n])
    return text[:m.start()] + block + text[m.start() + len(old):]


def fill_observed(text, obs):
    out = []
    in9 = False
    for line in text.split("\n"):
        if line.startswith("## 9."):
            in9 = True
        elif line.startswith("## ") and in9:
            in9 = False
        if in9 and line.startswith("| ") and not line.startswith("| algorithm"):
            cells = line.split(" | ")
            key = cells[0][2:].strip()
            if key in obs and len(cells) >= 5:
                cells[2] = obs[key]
                line = " | ".join(cells)
        out.append(line)
    return "\n".join(out)


def fill_hardware(text):
    p = os.path.join(RES, "hardware.txt")
    if not os.path.exists(p):
        print("  note: results/hardware.txt missing, §13 hardware line left as is")
        return text
    with open(p) as f:
        hw = "; ".join(l.strip() for l in f if l.strip())
    line = "* **Hardware.** %s (single-threaded: 1 core used)." % hw
    return re.sub(r"^\* \*\*Hardware\.\*\*.*$", lambda _: line, text, count=1, flags=re.M)


def main():
    with open(README) as fh:
        text = fh.read()
    shutil.copy(README, README + ".bak")
    text = put_block(text, "scaling", t_scaling(), r"^\| size \| score \| survival \|")
    text = put_block(text, "stats", t_stats(), r"^\| statistic \| small \|")
    text = put_block(text, "replan", t_replan(), r"^\| size \| mode \| avg ms/event \|")
    text = put_block(text, "gap", t_gap(), r"^\| method \| mean gap to B&B % \|")
    text = put_block(text, "compare", t_compare(), r"^\| comparison \| metric \| result \|")
    text = fill_observed(text, observed())
    text = fill_hardware(text)
    text = text.replace("> All tables below are placeholders. Fill them from `results/summary.csv` and `results/raw/`.",
                        "> Generated by `scripts/fill_readme.py` from `results/summary.csv` and `results/raw/`; "
                        "values are means over seeds, (min–max) where they differ.")
    with open(README, "w") as fh:
        fh.write(text)
    left = len(re.findall(r"\*\*RESULTS", text))
    print("README.md updated (previous version in README.md.bak); %d RESULTS markers left for prose" % left)


if __name__ == "__main__":
    main()
