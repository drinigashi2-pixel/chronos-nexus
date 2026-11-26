// test_scheduler.cpp - validator on every solver output, B&B == exhaustive search,
// ALNS optimality gap vs exact B&B on <= 12 missions, hard constraints.
#include "chronos/scheduler.hpp"
#include "test_util.hpp"

using namespace chronos;

namespace {
// Exhaustive canonical enumeration (each mission inserted at any position of any ship, or rejected).
double exhaustive(Scheduler& s, const World& w, const std::vector<int>& missions, const std::vector<int>& ships) {
  Solution base = s.solution();
  for (int m : missions) base.shipOf[m] = -1;
  for (int sh : ships) base.seq[sh].clear();
  double best = INF;
  std::function<void(size_t, Solution&)> rec = [&](size_t k, Solution& cur) {
    if (k == missions.size()) {
      s.restore(cur);
      best = std::min(best, s.recomputeTotal());
      return;
    }
    int m = missions[k];
    rec(k + 1, cur);  // reject
    for (int sh : ships) {
      const Ship& S = w.ships[sh];
      const Mission& M = w.missions[m];
      if ((S.caps & M.requiredCaps) != M.requiredCaps || S.cargoCap < M.amount) continue;
      for (size_t p = 0; p <= cur.seq[sh].size(); ++p) {
        cur.seq[sh].insert(cur.seq[sh].begin() + static_cast<long>(p), m);
        cur.shipOf[m] = sh;
        rec(k + 1, cur);
        cur.seq[sh].erase(cur.seq[sh].begin() + static_cast<long>(p));
        cur.shipOf[m] = -1;
      }
    }
  };
  rec(0, base);
  return best;
}

World metricWorld(uint64_t seed, int missions, int ships) {
  World w = testing::makeWorld(40, 120, seed, true);
  testing::addShips(w, ships, seed + 1);
  testing::addMissions(w, missions, seed + 2, 3);
  return w;
}
}  // namespace

TEST(scheduler_bnb_equals_exhaustive) {
  for (uint64_t seed = 1; seed <= 6; ++seed) {
    World w = metricWorld(seed, 5, 2);
    Router r(w.graph);
    r.setBoundsEnabled(true);
    Scheduler s(w, r, SchedulerConfig(), seed);
    s.syncWithWorld();
    s.buildCandidates(s.activeMissions());
    bool proved = s.solveBranchAndBound(50000000, Deadline::never());
    CHECK(proved);
    double bnb = s.recomputeTotal();
    Solution keep = s.solution();
    double ex = exhaustive(s, w, s.activeMissions(), {0, 1});
    CHECK_NEAR(bnb, ex, 1e-6);
    s.restore(keep);
    CHECK(s.validate().ok());
  }
}

TEST(scheduler_alns_gap_vs_exact_bnb) {
  double worst = 0;
  for (uint64_t seed = 10; seed < 16; ++seed) {
    World w = metricWorld(seed, 8 + static_cast<int>(seed % 5), 3);  // 8..12 missions
    Router r(w.graph);
    r.setBoundsEnabled(true);
    Scheduler exact(w, r, SchedulerConfig(), seed);
    exact.syncWithWorld();
    exact.buildCandidates(exact.activeMissions());
    bool proved = exact.solveBranchAndBound(200000000, Deadline::inMs(20000));
    double opt = exact.recomputeTotal();
    Scheduler h(w, r, SchedulerConfig(), seed);
    h.syncWithWorld();
    h.buildCandidates(h.activeMissions());
    h.insertRegret(h.unassigned(), 2, Deadline::never());
    h.runALNS(600, Deadline::never(), nullptr, nullptr);
    h.localSearch(Deadline::never(), nullptr);
    double alns = h.recomputeTotal();
    CHECK(h.validate().ok());
    if (proved) {
      CHECK(alns + 1e-6 >= opt);  // B&B is exact on metric instances
      worst = std::max(worst, (alns - opt) / std::max(1e-9, opt));
    }
  }
  std::printf("    worst ALNS gap vs B&B: %.3f%%\n", 100 * worst);
  CHECK(worst <= 0.15);
}

TEST(scheduler_all_methods_produce_valid_plans) {
  World w = testing::makeWorld(120, 500, 91);
  testing::addShips(w, 12, 5, 800, 120);  // small tanks: refuelling and fuel-constrained routes kick in
  testing::addMissions(w, 60, 7, 5);
  Router r(w.graph);
  r.setBoundsEnabled(true);
  for (int method = 0; method < 5; ++method) {
    Scheduler s(w, r, SchedulerConfig(), 11);
    s.syncWithWorld();
    s.buildCandidates(s.activeMissions());
    Deadline dl = Deadline::inMs(10000);
    auto un = s.unassigned();
    if (method == 0) s.insertGreedy(un, dl);
    else if (method == 1) {
      s.insertRegret(un, 2, dl);
      s.runALNS(200, dl, nullptr, nullptr);
      s.localSearch(dl, nullptr);
    } else if (method == 2) s.solveGA(15, dl, nullptr);
    else if (method == 3) s.solveSA(3000, dl, nullptr);
    else {
      s.insertRegret(un, 3, dl);
    }
    s.repairInvalid(dl);
    ValidationReport rep = s.validate();
    for (const auto& msg : rep.messages) std::printf("    %s\n", msg.c_str());
    CHECK(rep.ok());
    CHECK(s.assignedCount() > 0);
    // incremental total equals a full recomputation
    double inc = s.total();
    CHECK_NEAR(inc, s.recomputeTotal(), 1e-6);
  }
}

TEST(scheduler_respects_cargo_capacity_and_capabilities) {
  World w = metricWorld(3, 6, 2);
  w.missions[0].amount = 5000;          // bigger than every ship
  w.missions[1].requiredCaps |= CAP_ARMORED;  // nobody has armour
  Router r(w.graph);
  Scheduler s(w, r, SchedulerConfig(), 1);
  s.syncWithWorld();
  s.buildCandidates(s.activeMissions());
  s.insertRegret(s.unassigned(), 2, Deadline::never());
  s.runALNS(100, Deadline::never(), nullptr, nullptr);
  CHECK(s.solution().shipOf[0] < 0);
  CHECK(s.solution().shipOf[1] < 0);
  CHECK(s.validate().ok());
}

TEST(scheduler_pruning_reduces_candidates) {
  World w = testing::makeWorld(150, 600, 13);
  testing::addShips(w, 60, 2);
  testing::addMissions(w, 40, 3);
  Router r(w.graph);
  r.setBoundsEnabled(true);
  SchedulerConfig on, off;
  off.usePruning = false;
  Scheduler a(w, r, on, 1), b(w, r, off, 1);
  a.syncWithWorld();
  b.syncWithWorld();
  a.buildCandidates(a.activeMissions());
  b.buildCandidates(b.activeMissions());
  size_t ca = 0, cb = 0;
  for (int m : a.activeMissions()) ca += a.candidates(m).size();
  for (int m : b.activeMissions()) cb += b.candidates(m).size();
  CHECK(ca < cb);
  CHECK(ca <= static_cast<size_t>(on.kNearest) * a.activeMissions().size());
}
