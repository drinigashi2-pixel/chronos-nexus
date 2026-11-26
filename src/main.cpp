// main.cpp - CLI entry point: full simulation or algorithm micro-benchmarks.
//   chronos --dataset <dir> --out <dir> --time-ms N --mem-mb N --seed N [options]
//   chronos --dataset <dir> --out <dir> --bench routing|flow|gap|infra [--bench-n N]
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>

#include "chronos/io.hpp"
#include "chronos/sim.hpp"

using namespace chronos;

namespace {

void usage() {
  std::printf(
      "Usage: chronos --dataset <dir> --out <dir> [--time-ms N] [--mem-mb N] [--seed N]\n"
      "  [--days N] [--replan incremental|full] [--sched auto|greedy|alns|local|fast|ga|sa|bnb]\n"
      "  [--alloc auto|mcf|greedy] [--routing alt|dijkstra|bidir]\n"
      "  [--no-regret] [--no-alns] [--no-ls] [--no-repair] [--no-pruning] [--quiet]\n"
      "  [--bench routing|flow|gap|infra] [--bench-n N]\n");
}

struct Cli {
  SimConfig cfg;
  std::string bench;
  int benchN = 0;
  bool ok = true;
};

Cli parse(int argc, char** argv) {
  Cli c;
  auto need = [&](int& i) -> const char* {
    if (i + 1 >= argc) {
      std::fprintf(stderr, "missing value for %s\n", argv[i]);
      c.ok = false;
      return "";
    }
    return argv[++i];
  };
  for (int i = 1; i < argc && c.ok; ++i) {
    std::string a = argv[i];
    if (a == "--dataset") c.cfg.datasetDir = need(i);
    else if (a == "--out") c.cfg.outDir = need(i);
    else if (a == "--time-ms") c.cfg.timeMs = std::atof(need(i));
    else if (a == "--mem-mb") c.cfg.memMB = std::atof(need(i));
    else if (a == "--seed") c.cfg.seed = static_cast<uint64_t>(std::strtoull(need(i), nullptr, 10));
    else if (a == "--days") c.cfg.days = std::atoi(need(i));
    else if (a == "--replan") c.cfg.replanMode = std::string(need(i)) == "full" ? ReplanMode::Full : ReplanMode::Incremental;
    else if (a == "--sched") c.cfg.schedForced = schedMethodFromString(need(i));
    else if (a == "--alloc") {
      std::string v = need(i);
      c.cfg.allocForced = v == "mcf" ? 0 : (v == "greedy" ? 1 : -1);
    } else if (a == "--routing") {
      std::string v = need(i);
      c.cfg.routing = v == "dijkstra" ? RouteAlgo::Dijkstra : (v == "bidir" ? RouteAlgo::Bidirectional : RouteAlgo::ALT);
    } else if (a == "--no-regret") c.cfg.useRegret = false;
    else if (a == "--no-alns") c.cfg.useALNS = false;
    else if (a == "--no-ls") c.cfg.useLocalSearch = false;
    else if (a == "--no-repair") c.cfg.useRepair = false;
    else if (a == "--no-pruning") c.cfg.usePruning = false;
    else if (a == "--quiet") c.cfg.quiet = true;
    else if (a == "--bench") c.bench = need(i);
    else if (a == "--bench-n") c.benchN = std::atoi(need(i));
    else if (a == "--help" || a == "-h") {
      usage();
      std::exit(0);
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
      c.ok = false;
    }
  }
  if (c.cfg.datasetDir.empty()) c.ok = false;
  c.cfg.timeMs = std::max(100.0, c.cfg.timeMs);
  c.cfg.memMB = std::max(16.0, c.cfg.memMB);
  return c;
}

// ---------------------------------------------------------------- benchmarks
std::string benchRouting(World& w, int n, uint64_t seed) {
  Rng rng(seed);
  std::ostringstream o;
  o << "{\n  \"benchmark\": \"routing\",\n  \"V\": " << w.graph.numNodes() << ", \"E\": " << w.graph.numEdges()
    << ",\n  \"results\": [\n";
  int V = w.graph.numNodes();
  if (V < 2) return "{}";
  std::vector<std::pair<int, int>> pairs;
  for (int i = 0; i < n; ++i) pairs.push_back({rng.uniformInt(V), rng.uniformInt(V)});
  Router base(w.graph);
  base.setCacheEnabled(false);
  base.setNaiveSampling(0);
  base.alt().setLandmarkCount(MetaSelector::landmarkCount(V, 4096));
  bool first = true;
  for (int prof = 0; prof < 4; ++prof) {
    Timer tb;
    base.setAlgo(RouteAlgo::ALT);
    base.ensureAlt(prof);
    double buildMs = tb.ms();
    std::vector<double> ref(pairs.size());
    for (int a = 0; a < 3; ++a) {
      RouteAlgo algo = static_cast<RouteAlgo>(a);
      base.setAlgo(algo);
      uint64_t settled0 = counters().routingSettled;
      Timer t;
      int mismatches = 0, found = 0;
      for (size_t i = 0; i < pairs.size(); ++i) {
        Path p = base.compute(pairs[i].first, pairs[i].second, prof);
        double c = p.found ? p.cost : INF;
        if (a == 0) ref[i] = c;
        else if (std::isfinite(ref[i]) != std::isfinite(c) ||
                 (std::isfinite(c) && std::fabs(c - ref[i]) > 1e-6 * std::max(1.0, ref[i])))
          ++mismatches;
        found += p.found;
      }
      double ms = t.ms();
      uint64_t settled = counters().routingSettled - settled0;
      o << (first ? "" : ",\n") << "    {\"profile\": \"" << costProfile(prof).name << "\", \"algorithm\": \""
        << routeAlgoName(algo) << "\", \"queries\": " << pairs.size() << ", \"found\": " << found
        << ", \"avg_settled\": " << fmtDouble(static_cast<double>(settled) / pairs.size(), 2)
        << ", \"avg_ms\": " << fmtDouble(ms / pairs.size(), 5) << ", \"cost_mismatches_vs_dijkstra\": " << mismatches
        << ", \"landmark_build_ms\": " << fmtDouble(a == 2 ? buildMs : 0.0, 3) << "}";
      first = false;
    }
  }
  o << "\n  ]\n}\n";
  return o.str();
}

std::string benchFlow(World& w) {
  Router router(w.graph);
  ResourceAllocator alloc(w, router);
  std::vector<double> zero(static_cast<size_t>(w.numPlanets()) * NUM_RESOURCES, 0.0);
  std::ostringstream o;
  o << "{\n  \"benchmark\": \"flow\",\n  \"results\": [\n";
  double fleet = 0;
  for (const auto& s : w.ships) fleet += s.cargoCap;
  for (int m = 0; m < 2; ++m) {
    AllocConfig ac;
    ac.method = static_cast<AllocMethod>(m);
    ac.fleetCapacity = 1.5 * fleet;
    uint64_t st0 = counters().statesExplored;
    AllocationStats st = alloc.allocate(ac, 0.0, zero, zero, Deadline::never());
    o << (m ? ",\n" : "") << "    {\"method\": \"" << (m ? "greedy" : "min_cost_flow")
      << "\", \"transport_cost\": " << fmtDouble(st.transportCost, 3) << ", \"shortage_penalty\": "
      << fmtDouble(st.shortagePenalty, 3) << ", \"objective\": " << fmtDouble(st.transportCost + st.shortagePenalty, 3)
      << ", \"shortage_units\": " << fmtDouble(st.shortageUnits, 2) << ", \"planned_units\": "
      << fmtDouble(st.plannedUnits, 2) << ", \"shipments\": " << alloc.shipments().size()
      << ", \"deficits\": " << st.deficits << ", \"supplies\": " << st.supplies << ", \"arcs\": " << st.arcs
      << ", \"phases\": " << st.phases << ", \"states\": " << (counters().statesExplored - st0)
      << ", \"ms\": " << fmtDouble(st.ms, 3) << "}";
  }
  o << "\n  ]\n}\n";
  return o.str();
}

std::string benchGap(const World& w0, int n, uint64_t seed) {
  Rng rng(seed);
  std::ostringstream o;
  o << "{\n  \"benchmark\": \"optimality_gap\",\n  \"instances\": [\n";
  std::vector<int> inputs;
  for (size_t m = 0; m < w0.missions.size(); ++m) inputs.push_back(static_cast<int>(m));
  if (inputs.size() < 4 || w0.ships.size() < 2) return "{}";
  World w = w0;
  Router router(w.graph);
  bool first = true;
  for (int inst = 0; inst < n; ++inst) {
    int k = 6 + rng.uniformInt(5);  // 6..10 missions
    std::vector<int> pick;
    for (int i = 0; i < k; ++i) pick.push_back(inputs[rng.uniformInt(static_cast<int>(inputs.size()))]);
    std::sort(pick.begin(), pick.end());
    pick.erase(std::unique(pick.begin(), pick.end()), pick.end());
    for (auto& m : w.missions) m.status = MissionStatus::Cancelled;
    for (int m : pick) {
      w.missions[m] = w0.missions[m];
      w.missions[m].status = MissionStatus::Pending;
      w.missions[m].release = 0;
    }
    // 3 ships closest to the first mission's origin that can carry its cargo type
    std::vector<std::pair<double, int>> sd;
    const Planet& o0 = w.planets[w.missions[pick[0]].origin];
    for (size_t s = 0; s < w.ships.size(); ++s) {
      const Planet& p = w.planets[w.ships[s].location];
      sd.push_back({std::hypot(p.x - o0.x, p.y - o0.y), static_cast<int>(s)});
    }
    std::sort(sd.begin(), sd.end());
    std::vector<int> ships;
    for (size_t i = 0; i < sd.size() && ships.size() < 3; ++i) ships.push_back(sd[i].second);
    const char* names[5] = {"bnb", "regret_alns", "greedy", "ga", "sa"};
    double cost[5] = {0, 0, 0, 0, 0}, ms[5] = {0, 0, 0, 0, 0};
    bool proved = false;
    for (int a = 0; a < 5; ++a) {
      SchedulerConfig sc;
      Scheduler sch(w, router, sc, seed + static_cast<uint64_t>(inst));
      sch.restrictShips(ships);
      sch.setNow(0);
      sch.syncWithWorld();
      sch.buildCandidates(sch.activeMissions());
      Timer t;
      Deadline dl = Deadline::inMs(20000);
      std::vector<int> un = sch.unassigned();
      if (a == 0) proved = sch.solveBranchAndBound(20000000, dl);
      else if (a == 1) {
        sch.insertRegret(un, 2, dl);
        sch.runALNS(400, dl, nullptr, nullptr);
        sch.localSearch(dl, nullptr);
      } else if (a == 2) sch.insertGreedy(un, dl);
      else if (a == 3) sch.solveGA(60, dl, nullptr);
      else sch.solveSA(4000, dl, nullptr);
      ms[a] = t.ms();
      cost[a] = sch.recomputeTotal();
    }
    o << (first ? "" : ",\n") << "    {\"missions\": " << pick.size() << ", \"ships\": " << ships.size()
      << ", \"bnb_proved_optimal\": " << (proved ? "true" : "false");
    for (int a = 0; a < 5; ++a)
      o << ", \"" << names[a] << "_cost\": " << fmtDouble(cost[a], 4) << ", \"" << names[a]
        << "_ms\": " << fmtDouble(ms[a], 3) << ", \"" << names[a] << "_gap_pct\": "
        << fmtDouble(cost[0] > 0 ? 100.0 * (cost[a] - cost[0]) / cost[0] : 0.0, 4);
    o << "}";
    first = false;
  }
  o << "\n  ]\n}\n";
  return o.str();
}

std::string benchInfra(World& w, int n, uint64_t seed) {
  Rng rng(seed);
  InfraPlanner planner(w);
  std::ostringstream o;
  o << "{\n  \"benchmark\": \"infrastructure\",\n  \"results\": [\n";
  bool first = true;
  int total = static_cast<int>(w.facilities.size());
  for (int inst = 0; inst < n && total > 0; ++inst) {
    int k = std::min(total, inst % 2 == 0 ? 30 : 400);
    std::vector<InfraItem> items;
    std::vector<int> grp;
    for (int i = 0; i < k; ++i) {
      int idx = rng.uniformInt(total);
      const auto& f = w.facilities[idx];
      InfraItem it;
      it.id = idx;
      it.group = static_cast<int>(f.planet) * NUM_FACILITY_TYPES + f.type;
      it.cost = std::round(f.cost);
      it.value = planner.optionValue(f, w.days);
      items.push_back(it);
    }
    double sum = 0;
    for (const auto& it : items) sum += it.cost;
    double budget = std::round(0.3 * sum);
    auto pr = InfraPlanner::prune(items, budget);
    InfraResult dp = InfraPlanner::solveDP(pr, budget), gr = InfraPlanner::solveGreedy(pr, budget, true);
    InfraResult bb;
    bool runBb = pr.size() <= 60;
    if (runBb) bb = InfraPlanner::solveBnB(pr, budget);
    o << (first ? "" : ",\n") << "    {\"items\": " << items.size() << ", \"after_pruning\": " << pr.size()
      << ", \"budget\": " << fmtDouble(budget, 1) << ", \"dp_value\": " << fmtDouble(dp.value, 3)
      << ", \"dp_exact\": " << (dp.exact ? "true" : "false") << ", \"dp_ms\": " << fmtDouble(dp.ms, 3)
      << ", \"greedy_value\": " << fmtDouble(gr.value, 3) << ", \"greedy_ms\": " << fmtDouble(gr.ms, 3)
      << ", \"greedy_ratio\": " << fmtDouble(dp.value > 0 ? gr.value / dp.value : 1.0, 5);
    if (runBb)
      o << ", \"bnb_value\": " << fmtDouble(bb.value, 3) << ", \"bnb_nodes\": " << bb.nodes
        << ", \"bnb_ms\": " << fmtDouble(bb.ms, 3);
    o << "}";
    first = false;
  }
  o << "\n  ]\n}\n";
  return o.str();
}

}  // namespace

int main(int argc, char** argv) {
  Cli cli = parse(argc, argv);
  if (!cli.ok) {
    usage();
    return 2;
  }
  Timer total;
  World world;
  LoadReport rep;
  if (!loadDataset(cli.cfg.datasetDir, world, rep)) {
    std::fprintf(stderr, "error: could not load planets from %s\n", cli.cfg.datasetDir.c_str());
    for (const auto& w : rep.warnings) std::fprintf(stderr, "  warning: %s\n", w.c_str());
    return 1;
  }
  if (!cli.cfg.quiet) {
    std::fprintf(stderr,
                 "loaded %d planets, %d route rows (%d directed edges, %d duplicates), %d ships, %d missions, "
                 "%d facility options, %d events in %.1f ms\n",
                 rep.planets, rep.routeRows, rep.edges, rep.duplicateEdges, rep.ships, rep.missions, rep.facilities,
                 rep.events, rep.ms);
    for (const auto& w : rep.warnings) std::fprintf(stderr, "  warning: %s\n", w.c_str());
    if (rep.unknownRefs) std::fprintf(stderr, "  warning: %d rows referenced unknown objects and were skipped\n", rep.unknownRefs);
  }
  ensureDirectory(cli.cfg.outDir);
  if (!cli.bench.empty()) {
    int n = cli.benchN > 0 ? cli.benchN : (cli.bench == "routing" ? 200 : 10);
    std::string json;
    if (cli.bench == "routing") json = benchRouting(world, n, cli.cfg.seed);
    else if (cli.bench == "flow") json = benchFlow(world);
    else if (cli.bench == "gap") json = benchGap(world, n, cli.cfg.seed);
    else if (cli.bench == "infra") json = benchInfra(world, n, cli.cfg.seed);
    else {
      std::fprintf(stderr, "unknown benchmark %s\n", cli.bench.c_str());
      return 2;
    }
    writeTextFile(joinPath(cli.cfg.outDir, "bench_" + cli.bench + ".json"), json);
    std::printf("%s", json.c_str());
    return 0;
  }
  cli.cfg.loadMs = total.ms();
  Simulator sim(world, cli.cfg);
  Statistics st = sim.run();
  if (cli.cfg.writeFiles) sim.writeOutputs(cli.cfg.outDir);
  std::printf("score=%.3f survival=%.4f completed=%d failed=%d events=%d avg_replan_ms=%.3f states=%llu "
              "reduction=%.2f%% time_ms=%.1f peak_mb=%.1f\n",
              st.totalScore, st.populationSurvivalRate, st.missionsCompleted, st.missionsFailed, st.eventsHandled,
              st.avgReplanMs, static_cast<unsigned long long>(st.statesExplored), st.searchSpaceReductionPct,
              total.ms(), st.peakMemoryMB);
  return 0;
}
