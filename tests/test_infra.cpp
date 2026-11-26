// test_infra.cpp - multiple-choice knapsack: DP == brute force == B&B, greedy within 2-approx,
// pruning keeps the optimum, planner integration.
#include "chronos/infra.hpp"
#include "test_util.hpp"

using namespace chronos;

namespace {
std::vector<InfraItem> randomItems(Rng& rng, int n, int groups) {
  std::vector<InfraItem> v;
  for (int i = 0; i < n; ++i) {
    InfraItem it;
    it.id = i;
    it.group = groups > 0 ? rng.uniformInt(groups) : i;
    it.cost = 1 + rng.uniformInt(30);
    it.value = 1 + rng.uniformInt(100);
    v.push_back(it);
  }
  return v;
}

double bruteForce(const std::vector<InfraItem>& items, double budget) {
  int n = static_cast<int>(items.size());
  double best = 0;
  for (int mask = 0; mask < (1 << n); ++mask) {
    double c = 0, val = 0;
    bool ok = true;
    std::vector<int> used;
    for (int i = 0; i < n && ok; ++i) {
      if (!(mask >> i & 1)) continue;
      if (std::find(used.begin(), used.end(), items[i].group) != used.end()) ok = false;
      used.push_back(items[i].group);
      c += items[i].cost;
      val += items[i].value;
    }
    if (ok && c <= budget + 1e-9) best = std::max(best, val);
  }
  return best;
}

bool feasible(const std::vector<InfraItem>& items, const InfraResult& r, double budget) {
  double c = 0;
  std::vector<int> groups;
  for (int i : r.chosen) {
    c += items[i].cost;
    if (std::find(groups.begin(), groups.end(), items[i].group) != groups.end()) return false;
    groups.push_back(items[i].group);
  }
  return c <= budget + 1e-9;
}
}  // namespace

TEST(infra_dp_equals_brute_force) {
  Rng rng(71);
  for (int k = 0; k < 200; ++k) {
    int n = 1 + rng.uniformInt(12);
    auto items = randomItems(rng, n, 1 + rng.uniformInt(n));
    double budget = 5 + rng.uniformInt(80);
    InfraResult dp = InfraPlanner::solveDP(items, budget);
    CHECK(dp.exact);
    CHECK(feasible(items, dp, budget));
    CHECK_NEAR(dp.value, bruteForce(items, budget), 1e-9);
  }
}

TEST(infra_bnb_equals_dp) {
  Rng rng(73);
  for (int k = 0; k < 200; ++k) {
    int n = 1 + rng.uniformInt(14);
    auto items = randomItems(rng, n, 1 + rng.uniformInt(n));
    double budget = 5 + rng.uniformInt(100);
    InfraResult dp = InfraPlanner::solveDP(items, budget), bb = InfraPlanner::solveBnB(items, budget);
    CHECK(bb.exact);
    CHECK(feasible(items, bb, budget));
    CHECK_NEAR(bb.value, dp.value, 1e-9);
  }
}

TEST(infra_greedy_within_two_approximation) {
  Rng rng(79);
  for (int k = 0; k < 300; ++k) {
    int n = 1 + rng.uniformInt(40);
    bool singleLevel = k % 2 == 0;
    auto items = randomItems(rng, n, singleLevel ? 0 : 1 + rng.uniformInt(n));
    double budget = 5 + rng.uniformInt(150);
    auto pr = InfraPlanner::prune(items, budget);
    InfraResult dp = InfraPlanner::solveDP(pr, budget), gr = InfraPlanner::solveGreedy(pr, budget, true);
    CHECK(feasible(pr, gr, budget));
    CHECK(gr.value <= dp.value + 1e-9);
    CHECK(2.0 * gr.value + 1e-9 >= dp.value);  // hull greedy + best single item: >= OPT/2
  }
}

TEST(infra_pruning_keeps_optimum) {
  Rng rng(83);
  for (int k = 0; k < 200; ++k) {
    int n = 1 + rng.uniformInt(12);
    auto items = randomItems(rng, n, 1 + rng.uniformInt(4));
    double budget = 5 + rng.uniformInt(60);
    auto pr = InfraPlanner::prune(items, budget);
    CHECK(pr.size() <= items.size());
    CHECK_NEAR(InfraPlanner::solveDP(pr, budget).value, InfraPlanner::solveDP(items, budget).value, 1e-9);
  }
}

TEST(infra_planner_respects_budget_and_groups) {
  World w = testing::makeWorld(40, 100, 5);
  Rng rng(3);
  for (int p = 0; p < w.numPlanets(); ++p)
    for (int t = 0; t < 2; ++t)
      for (int lvl = 1; lvl <= 3; ++lvl) {
        FacilityOption f;
        f.extId = static_cast<int64_t>(w.facilities.size());
        f.planet = p;
        f.type = (p + t) % NUM_FACILITY_TYPES;
        f.level = lvl;
        f.cost = 50.0 * lvl * rng.uniform(0.8, 1.2);
        f.buildDays = 2 + lvl;
        f.resourceImpact[rng.uniformInt(NUM_RESOURCES)] = 10.0 * lvl;
        f.strategicValue = lvl;
        w.facilities.push_back(f);
      }
  InfraPlanner planner(w);
  std::vector<int> ids;
  for (size_t i = 0; i < w.facilities.size(); ++i) ids.push_back(static_cast<int>(i));
  for (InfraMethod m : {InfraMethod::DP, InfraMethod::Greedy, InfraMethod::Auto}) {
    InfraResult r = planner.plan(ids, 1000.0, 14.0, m, Deadline::never());
    double cost = 0;
    std::vector<std::pair<int, int>> groups;
    for (int o : r.chosen) {
      cost += w.facilities[o].cost;
      auto g = std::make_pair(static_cast<int>(w.facilities[o].planet), w.facilities[o].type);
      CHECK(std::find(groups.begin(), groups.end(), g) == groups.end());
      groups.push_back(g);
    }
    CHECK(cost <= 1000.0 + 1e-9);
    CHECK(r.value > 0);
  }
}
