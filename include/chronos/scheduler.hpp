// scheduler.hpp - mission scheduling: which ship, departure time, route, delay/priority.
//
// Model: each ship executes an ordered sequence of missions; a mission is an empty
// leg (ship location -> origin), load, loaded leg (origin -> destination), unload.
// Cargo of one mission at a time (missions larger than a ship are chunked upstream).
//
// Main solver: ALNS with simulated-annealing acceptance.
//   construction: regret-2 insertion (lazy max-heap of regrets)
//   destroy: random | worst-cost | Shaw-related | deadline-urgent | dependency-guided
//   repair: greedy | regret-2 | regret-3 (beam-limited insertion positions)
//   adaptive roulette-wheel operator weights (segment-based scores)
//   polish: relocate, swap, 2-opt* (tail exchange)
// Per-iteration cost: O(q * K * P * L) evaluations (q removed, K candidate ships,
// P beam positions, L sequence length); each evaluation reuses cached routes.
// Baselines: greedy-only, plain GA (permutation + decoder), SA-only, exact B&B (<= ~12 missions).
#pragma once
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "chronos/routing.hpp"

namespace chronos {

enum class SchedMethod : uint8_t { Auto = 0, Greedy, RegretALNS, LocalRepair, GreedyPruneALNS, GA, SA, BranchAndBound };
const char* schedMethodName(SchedMethod m);
SchedMethod schedMethodFromString(const std::string& s);  // Auto if unknown

struct SchedulerConfig {
  int kNearest = 12;         // candidate ships per mission
  int beam = 2;              // insertion positions around the deadline rank (+ append)
  double maxLateH = 72;      // ALT deadline pruning allowance
  bool usePruning = true;    // K-nearest + LB + dominance pruning (false = all ship-mission pairs)
  bool appendOnly = false;   // very large instances: only try appending to a ship's sequence
  int regretBatch = 400;     // regret insertion processes missions in deadline-ordered batches (anytime)
  double wLate = 2.0;        // cost per hour late, times priority*importance
  double wChance = 4.0;      // chance-constraint violation cost per hour of missing slack
  double rejectFactor = 4.0; // reject penalty = value * rejectFactor
  double zOnTime = 1.2816;   // P(on time) >= 0.9 for priority >= 4
  double loadH = 1, unloadH = 1, refuelH = 2, refuelCost = 20;
  double emptyLegFactor = 0.5;  // empty legs are cheaper than loaded legs
  // Planning-only weight on the empty leg (the executed cost still uses emptyLegFactor). Empty
  // repositioning costs fuel and ship-hours that the transport-cost and energy score terms count
  // against a dedicated round trip; weighting it 3x when choosing ships/positions measurably
  // shortens empty legs (tuned on 3 seeds x small/medium/large, mean +0.1 to +0.6 points).
  double emptyLegPlanWeight = 3.0;
  int gaPopulation = 24;
  int destroyMin = 2, destroyMax = 40;
  double destroyFrac = 0.15;
  int segment = 40;
  double reaction = 0.25;
};

struct MissionEval {
  int mission = -1;
  double start = 0, arriveOrigin = 0, depart = 0, arrive = 0, sigma = 0, pOnTime = 1;
  double cost = 0, travelCost = 0, latePen = 0, chancePen = 0, failPen = 0, failP = 0;
  bool feasible = true;
  int profile = 0;
  std::vector<EdgeId> emptyPath, loadedPath;  // filled only when details are requested
  std::vector<NodeId> refuels;  // refuel stops in travel order
  int refuelSplit = 0;          // refuels[0..split) belong to the empty leg
  double dist = 0, fuel = 0, risk = 0, reliability = 1;
};
struct ShipEval {
  double cost = 0;
  bool feasible = true;
  std::vector<MissionEval> items;
};

struct Solution {
  std::vector<std::vector<int>> seq;  // per ship: global mission ids in execution order
  std::vector<int> shipOf;            // per mission: ship or -1
  std::vector<double> shipCost;       // evaluated cost per ship
  double total = 0;                   // sum shipCost + reject penalties of unassigned active missions
};

struct ValidationReport {
  int checked = 0, hardViolations = 0, lateMissions = 0;
  std::vector<std::string> messages;
  bool ok() const { return hardViolations == 0; }
};

struct ConvergencePoint {
  double ms;
  int64_t iter;
  double best, current;
};

class Scheduler {
 public:
  Scheduler(World& w, Router& r, const SchedulerConfig& cfg, uint64_t seed);
  SchedulerConfig& config() { return cfg_; }

  void setNow(double now) { now_ = now; }
  double now() const { return now_; }

  // ---- active set: missions with Pending/Assigned status that are schedulable now
  void syncWithWorld();                                // O(M)
  void restrictShips(const std::vector<int>& ships);   // empty = all alive ships
  void activate(const std::vector<int>& missions);     // new missions become schedulable, O(k)
  void deactivate(const std::vector<int>& missions);   // cancelled missions leave the pool, O(k + active)
  void refreshShip(int s);                             // re-evaluate one ship after an external change
  const std::vector<int>& activeMissions() const { return active_; }
  bool isActive(int m) const { return m >= 0 && m < static_cast<int>(isActive_.size()) && isActive_[m]; }
  std::vector<int> unassigned() const;
  bool shipEnabled(int s) const;

  // ---- candidate generation (K-nearest ships by spatial grid, capability/capacity/fuel
  //      filters, ALT deadline lower bound pruning, ship dominance)
  void buildCandidates(const std::vector<int>& missions);
  // Fuel-constrained leg, first attempt: the best route a -> b with the tank filled at the current
  // node whenever the next edge needs more fuel than is left (refuel nodes only). Feasible by
  // construction or not found; the exact label search (Router::routeFuel) is the fallback.
  Path refuelAlongRoute(NodeId a, NodeId b, int profile, double minCap, double fuelStart, double fuelCap);
  const std::vector<int>& candidates(int m) const;
  // Candidate epochs. Eager (default): buildCandidates() is called for every mission that needs a
  // list. Lazy: lists are built on first use in bestInsertion() within the epoch, so a cycle that
  // can only insert a few thousand of 40k waiting missions does not route all 40k (extreme).
  void beginCandidateEpoch(bool lazy);
  void ensureCandidates(int m);
  void clearCandidates(int m) {
    if (m >= 0 && m < static_cast<int>(cand_.size())) cand_[m].clear();
  }

  // ---- construction heuristics
  void insertGreedy(const std::vector<int>& missions, const Deadline& dl);
  void insertRegret(const std::vector<int>& missions, int k, const Deadline& dl);

  // ---- improvement
  // scope != nullptr restricts destroy operators to that mission set (incremental re-planning).
  int64_t runALNS(int64_t iterations, const Deadline& dl, const std::vector<int>* scope,
                  std::vector<ConvergencePoint>* trace);
  int localSearch(const Deadline& dl, const std::vector<int>* scope);
  void solveGA(int generations, const Deadline& dl, std::vector<ConvergencePoint>* trace);
  void solveSA(int64_t iterations, const Deadline& dl, std::vector<ConvergencePoint>* trace);
  bool solveBranchAndBound(uint64_t nodeLimit, const Deadline& dl);  // true = proved optimal
  int repairInvalid(const Deadline& dl);  // remove hard-infeasible missions and re-insert

  // ---- modification (re-planning)
  std::vector<int> remove(const std::vector<int>& missions);  // returns the ones that were assigned
  void unassignAll();

  // ---- evaluation / output
  double evaluateShip(int s, const std::vector<int>& seq, ShipEval* out);
  double rejectPenalty(int m) const;
  double total() const { return sol_.total; }
  double recomputeTotal();
  ValidationReport validate();
  void extractPlans(const std::vector<int>* ships);  // write MissionPlan for ships (null = all)
  int profileFor(int m) const;
  std::vector<int> related(const std::vector<int>& seeds, int count, const std::vector<int>* scope);
  const Solution& solution() const { return sol_; }
  void restore(const Solution& s);
  int assignedCount() const;
  int64_t iterationsForBudget(double ms, size_t scope) const;
  void invalidateFuelCache() {
    fuelIndex_.clear();
    fuelPaths_.clear();
  }
  // Deterministic work cap (search states counted from now on); 0 = unlimited.
  void setWorkCap(double states) {
    workCap_ = states;
    workStart_ = static_cast<double>(counters().statesExplored);
  }
  bool workExceeded(double frac = 1.0) const {
    return workCap_ > 0 && static_cast<double>(counters().statesExplored) - workStart_ > frac * workCap_;
  }
  double workProgress() const {
    return workCap_ > 0 ? (static_cast<double>(counters().statesExplored) - workStart_) / workCap_ : 0.0;
  }
  uint64_t evaluations() const { return evals_; }
  double slots() const { return slots_; }
  std::string lastOperatorStats() const;

 private:
  struct Ins {
    double delta = INF;
    int ship = -1, pos = -1;
  };
  struct LegInfo {
    bool found = false;
    double cost = 0, time = 0, var = 0, fuel = 0, rel = 1, risk = 0, dist = 0, bottleneck = INF;
  };
  void ensureSizes();
  bool capable(int s, int m) const;
  LegInfo leg(NodeId a, NodeId b, int profile, double minCap, std::vector<EdgeId>* edges);
  bool fuelLeg(int s, NodeId a, NodeId b, int profile, double minCap, double& fuel, LegInfo& li,
               std::vector<EdgeId>* edges, std::vector<NodeId>* refuels);
  std::vector<int> positionsFor(int m, int s) const;
  Ins bestInsertion(int m, std::vector<double>* perShipBest, bool appendOnly = false);
  double insertionDelta(int m, int s, int pos);
  void applyInsert(int m, int s, int pos);
  void applyRemove(int m);
  void touch(int s);
  void beginUndo();
  void rollback();
  void setSequence(int s, const std::vector<int>& seq);  // pairs must preserve the assigned set
  void recomputeSlots();
  double relatedness(int a, int b) const;
  void startSnapshots();
  void captureBest();
  void restoreBest();
  void stopSnapshots();
  // ALNS operators
  std::vector<int> destroy(int op, int q, const std::vector<int>& pool);
  void repairOp(int op, std::vector<int>& removed);
  double missionContribution(int m);
  void bnbRecurse(size_t k);

  World& w_;
  Router& router_;
  SchedulerConfig cfg_;
  Rng rng_;
  double now_ = 0;
  Solution sol_;
  std::vector<int> active_;
  std::vector<char> isActive_;
  std::vector<std::vector<int>> cand_;
  std::vector<uint32_t> candStamp_;  // epoch in which cand_[m] was last built
  uint32_t candEpoch_ = 1;
  bool lazyCand_ = false;
  SpatialGrid2D grid_;
  bool gridValid_ = false;
  std::vector<int> candOne_;
  std::vector<char> shipMask_;
  bool restricted_ = false;
  uint64_t evals_ = 0;
  // undo log
  std::vector<std::pair<int, std::vector<int>>> undoSeq_;
  std::vector<std::pair<int, double>> undoCost_;
  std::vector<std::pair<int, int>> undoShipOf_;
  std::vector<uint32_t> touchStamp_;
  uint32_t stamp_ = 1;
  double undoTotal_ = 0;
  bool recording_ = false;
  // ALNS adaptive weights
  std::vector<double> dW_, rW_, dS_, rS_;
  std::vector<int> dN_, rN_;
  std::vector<uint8_t> refuelMask_;
  FlatIndex fuelIndex_;          // fuel-leg memo: key -> slot in fuelPaths_
  std::vector<Path> fuelPaths_;
  std::vector<int> tmp_;
  double workCap_ = 0, workStart_ = 0;
  std::vector<int> unassignedLog_;    // missions unassigned since the last plan extraction
  double slots_ = 0, undoSlots_ = 0;  // sum over enabled ships of (len+1): naive insertion slots
  double scale_ = 1;                  // galaxy extent (Shaw relatedness normalisation)
  std::vector<char> poolFlag_;
  // best-solution snapshots (copy-on-write per touched ship)
  bool snapOn_ = false;
  double bestTotal_ = INF;
  std::unordered_map<int, std::pair<std::vector<int>, double>> snap_;
  std::vector<int> dirty_;
  std::vector<char> dirtyFlag_;
  // B&B state
  std::vector<int> bnbOrder_, bnbShips_;
  std::vector<double> bnbMin_;
  std::vector<double> bnbSuffix_, bnbSR_;
  double bnbBest_ = INF;
  Solution bnbBestSol_;
  uint64_t bnbNodes_ = 0, bnbLimit_ = 0;
  Deadline bnbDl_;
  bool bnbAborted_ = false;
};

}  // namespace chronos
