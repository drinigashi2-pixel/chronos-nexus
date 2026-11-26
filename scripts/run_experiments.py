#!/usr/bin/env python3
"""Run the Chronos Nexus experimental evaluation (Python 3 standard library only).

Experiments (all raw outputs go to results/raw/, one JSON per run + copied CSV logs):
  scaling      every size x seed with the default (meta-selected) pipeline
  algorithms   scheduler (greedy / regret+ALNS / GA / SA), allocation (MCF / greedy),
               routing (ALT / Dijkstra / bidirectional) on small + medium
  ablation     hybrid pipeline stages switched off one at a time (regret, ALNS, local search, repair)
  replan       incremental (our dependency-guided ALNS) vs full re-solve per event
  pruning      search-space reduction on/off
  bench        micro-benchmarks: routing, flow, optimality gap, infrastructure

Usage:
  python3 scripts/run_experiments.py                       # small, medium, large; seeds 1..3
  python3 scripts/run_experiments.py --sizes small medium --seeds 1 2
  python3 scripts/run_experiments.py --extreme             # also the extreme size (long!)
  python3 scripts/run_experiments.py --only scaling replan
A summary table is written to results/summary.csv. Numbers are only ever copied from the
program's own stats.json files; nothing is estimated here.
"""
import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD = os.path.join(ROOT, "build")
CHRONOS = os.path.join(BUILD, "chronos")
GEN = os.path.join(BUILD, "gen_galaxy")
RAW = os.path.join(ROOT, "results", "raw")
RUNS = os.path.join(ROOT, "results", "runs")
BUDGET_MS = {"small": 20000, "medium": 60000, "large": 180000, "extreme": 900000}
MEM_MB = {"small": 1024, "medium": 2048, "large": 4096, "extreme": 8192}
COPY_FILES = ["stats.json", "score.json", "cycles.csv", "convergence.csv", "replanning_log.csv"]


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


def ensure_binaries():
    if not (os.path.exists(CHRONOS) and os.path.exists(GEN)):
        log("building (scripts/build.sh --no-tests)")
        subprocess.check_call(["sh", os.path.join(ROOT, "scripts", "build.sh"), "--no-tests"])


def dataset(size, seed):
    d = os.path.join(ROOT, "data", size, "s%d" % seed)
    if not os.path.exists(os.path.join(d, "planets.csv")):
        log("generating %s seed %d" % (size, seed))
        subprocess.check_call([GEN, size, d, str(seed)], stdout=subprocess.DEVNULL)
    return d


def run(tag, size, seed, extra, budget=None):
    """Run chronos once; returns the parsed stats dict (or None) and stores raw files."""
    data = dataset(size, seed)
    out = os.path.join(RUNS, "%s_%s_s%d" % (tag, size, seed))
    os.makedirs(out, exist_ok=True)
    budget = budget or BUDGET_MS[size]
    cmd = [CHRONOS, "--dataset", data, "--out", out, "--time-ms", str(budget), "--mem-mb", str(MEM_MB[size]),
           "--seed", str(seed), "--quiet"] + extra
    log("run %-28s %-8s seed %d %s" % (tag, size, seed, " ".join(extra)))
    t0 = time.time()
    try:
        subprocess.run(cmd, check=True, timeout=max(60, 3 * budget / 1000.0), stdout=subprocess.DEVNULL)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        log("  FAILED: %s" % e)
        return None
    wall = time.time() - t0
    stats_path = os.path.join(out, "stats.json")
    if not os.path.exists(stats_path):
        return None
    with open(stats_path) as f:
        stats = json.load(f)
    record = {"experiment": tag, "size": size, "seed": seed, "args": extra, "budget_ms": budget,
              "wall_s": wall, "stats": stats}
    with open(os.path.join(out, "score.json")) as f:
        record["score"] = json.load(f)
    os.makedirs(RAW, exist_ok=True)
    with open(os.path.join(RAW, "%s_%s_s%d.json" % (tag, size, seed)), "w") as f:
        json.dump(record, f, indent=1)
    for name in COPY_FILES[2:]:
        src = os.path.join(out, name)
        if os.path.exists(src):
            shutil.copy(src, os.path.join(RAW, "%s_%s_s%d_%s" % (tag, size, seed, name)))
    return record


def bench(kind, size, seed, n=None):
    data = dataset(size, seed)
    out = os.path.join(RUNS, "bench_%s_%s_s%d" % (kind, size, seed))
    os.makedirs(out, exist_ok=True)
    cmd = [CHRONOS, "--dataset", data, "--out", out, "--bench", kind, "--seed", str(seed), "--quiet"]
    if n:
        cmd += ["--bench-n", str(n)]
    log("bench %-8s %-8s seed %d" % (kind, size, seed))
    try:
        subprocess.run(cmd, check=True, timeout=3600, stdout=subprocess.DEVNULL)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        log("  FAILED: %s" % e)
        return
    src = os.path.join(out, "bench_%s.json" % kind)
    if os.path.exists(src):
        os.makedirs(RAW, exist_ok=True)
        shutil.copy(src, os.path.join(RAW, "bench_%s_%s_s%d.json" % (kind, size, seed)))


def write_summary():
    rows = []
    if not os.path.isdir(RAW):
        return
    keys = ["total_score", "population_survival_rate", "resources_delivered", "resource_shortages", "resource_waste",
            "missions_completed", "missions_failed", "total_distance", "total_fuel", "total_transport_cost",
            "total_risk", "nodes_explored", "candidates_evaluated", "states_explored", "search_space_reduction_pct",
            "events_handled", "avg_replan_ms", "total_exec_ms", "peak_memory_mb"]
    for name in sorted(os.listdir(RAW)):
        if not name.endswith(".json") or name.startswith("bench_"):
            continue
        with open(os.path.join(RAW, name)) as f:
            r = json.load(f)
        row = {"experiment": r["experiment"], "size": r["size"], "seed": r["seed"], "args": " ".join(r["args"]),
               "budget_ms": r["budget_ms"]}
        for k in keys:
            row[k] = r["stats"].get(k)
        extra = r["stats"].get("extra", {})
        for k in ["naive_states", "max_replan_ms", "alns_iterations", "days_simulated", "routing_algorithm",
                  "missions_on_time", "missions_expired", "validation_hard_violations"]:
            row[k] = extra.get(k)
        rows.append(row)
    if not rows:
        return
    path = os.path.join(ROOT, "results", "summary.csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    log("summary written to %s (%d runs)" % (path, len(rows)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sizes", nargs="+", default=["small", "medium", "large"])
    ap.add_argument("--seeds", nargs="+", type=int, default=[1, 2, 3])
    ap.add_argument("--extreme", action="store_true", help="also run the extreme size (scaling only)")
    ap.add_argument("--only", nargs="+", default=["scaling", "algorithms", "ablation", "replan", "pruning", "bench"])
    ap.add_argument("--budget-scale", type=float, default=1.0, help="multiply every time budget")
    args = ap.parse_args()
    for k in BUDGET_MS:
        BUDGET_MS[k] = int(BUDGET_MS[k] * args.budget_scale)
    ensure_binaries()
    sizes = list(args.sizes) + (["extreme"] if args.extreme and "extreme" not in args.sizes else [])
    small_sizes = [s for s in args.sizes if s in ("small", "medium")]
    if "scaling" in args.only:
        for size in sizes:
            for seed in args.seeds:
                run("scaling", size, seed, [])
    if "algorithms" in args.only:
        for size in small_sizes:
            for seed in args.seeds:
                for sched in ["greedy", "alns", "ga", "sa"]:
                    run("sched_" + sched, size, seed, ["--sched", sched])
                run("alloc_greedy", size, seed, ["--alloc", "greedy"])
                run("alloc_mcf", size, seed, ["--alloc", "mcf"])
                for routing in ["dijkstra", "bidir"]:
                    run("routing_" + routing, size, seed, ["--routing", routing])
    if "ablation" in args.only:
        for size in small_sizes:
            for seed in args.seeds:
                for flag in ["--no-regret", "--no-alns", "--no-ls", "--no-repair"]:
                    run("ablation_" + flag.strip("-").replace("-", "_"), size, seed, ["--sched", "alns", flag])
                run("ablation_full_hybrid", size, seed, ["--sched", "alns"])
    if "replan" in args.only:
        for size in small_sizes:
            for seed in args.seeds:
                run("replan_incremental", size, seed, ["--replan", "incremental"])
                run("replan_full", size, seed, ["--replan", "full"])
    if "pruning" in args.only:
        for size in small_sizes:
            for seed in args.seeds:
                run("pruning_on", size, seed, [])
                run("pruning_off", size, seed, ["--no-pruning", "--routing", "dijkstra"])
    if "bench" in args.only:
        for size in args.sizes:
            seed = args.seeds[0]
            bench("routing", size, seed, 200)
            bench("flow", size, seed)
            bench("infra", size, seed, 10)
            if size == "small":
                bench("gap", size, seed, 15)
    write_summary()


if __name__ == "__main__":
    sys.exit(main())
