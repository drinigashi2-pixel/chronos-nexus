// infra.hpp - budgeted infrastructure selection (multiple-choice knapsack, MCKP).
//
// Groups = (planet, facility type); options in a group (levels) are mutually exclusive.
// Value = marginal score gain pro-rated by max(0, H - buildTime)/H.
//   prune:  remove non-positive / over-budget options and dominated options in a group
//           (cost >=, value <=). O(n log n)
//   DP:     exact over integer-scaled budget (<= 10^4 buckets). O(n * B) time, O(G * B) bytes
//   Greedy: lazy greedy on incremental efficiency over each group's upper convex hull
//           + best-single-item fix (>= OPT/2) + swap local search. O(n log n + LS)
//   B&B:    DFS with fractional (LP) bound for small-instance certification. Exponential worst case.
#pragma once
#include <vector>

#include "chronos/graph.hpp"

namespace chronos {

enum class InfraMethod : uint8_t { Auto = 0, DP, Greedy, BranchAndBound };
const char* infraMethodName(InfraMethod m);

struct InfraItem {
  int id = -1;     // caller's identifier (option index)
  int group = 0;   // mutually exclusive group
  double cost = 0;
  double value = 0;
};

struct InfraResult {
  std::vector<int> chosen;  // indices into the item vector given to the solver (plan(): option indices)
  double value = 0, cost = 0;
  InfraMethod method = InfraMethod::Auto;
  int candidates = 0, afterPruning = 0;
  uint64_t nodes = 0;
  bool exact = false;
  double ms = 0;
};

class InfraPlanner {
 public:
  explicit InfraPlanner(World& w) : w_(w) {}
  // Score references (start-of-run production value and tech, run length in days). With them set,
  // optionValue() also credits the economic-growth and scientific-progress score terms directly,
  // converted into the planner's value units through the resource-security exchange rate.
  void setScoreReference(double prodValueStart, double techStart, double days) {
    prodStart_ = prodValueStart;
    techStart_ = techStart;
    days_ = days;
  }
  // Marginal value of building option f, horizon in days.
  double optionValue(const FacilityOption& f, double horizonDays) const;
  // Select among optionIds (indices into world.facilities) under budget.
  InfraResult plan(const std::vector<int>& optionIds, double budget, double horizonDays, InfraMethod m,
                   const Deadline& dl);
  // Generic solvers (also exercised by the tests).
  static std::vector<InfraItem> prune(const std::vector<InfraItem>& items, double budget);
  static InfraResult solveDP(const std::vector<InfraItem>& items, double budget, int maxBuckets = 10000);
  static InfraResult solveGreedy(const std::vector<InfraItem>& items, double budget, bool localSearch = true);
  static InfraResult solveBnB(const std::vector<InfraItem>& items, double budget, uint64_t nodeLimit = 20000000);

 private:
  void refreshScoreState();  // headroom of the capped growth terms + value-per-point rate
  World& w_;
  double prodStart_ = 0, techStart_ = 0, days_ = 0;
  double egHeadroom_ = 1, spHeadroom_ = 1;  // fraction of each capped term still attainable
  double valuePerPoint_ = 0;                // planner value units per score point
};

}  // namespace chronos
