// meta.hpp - meta-optimization layer: budget manager + rule-based algorithm selector.
// All decisions are O(1) given the feature vector.
#pragma once
#include <string>

#include "chronos/flow.hpp"
#include "chronos/infra.hpp"
#include "chronos/scheduler.hpp"

namespace chronos {

struct Features {
  int64_t V = 0, E = 0;
  int activeMissions = 0, ships = 0, newMissions = 0;
  double minSlackH = INF;        // smallest (deadline - now) among active missions
  double remainingMs = 0, totalMs = 0;
  double memUsedMB = 0, memBudgetMB = 0;
  double eventSeverity = 0;      // max severity of events handled this cycle
  double routeVariance = 0;      // mean (sigma/mean) of edge travel times
  int cyclesLeft = 1;
  int cyclesTotal = 1;
};

struct Strategy {
  SchedMethod sched = SchedMethod::RegretALNS;
  AllocMethod alloc = AllocMethod::MinCostFlow;
  InfraMethod infra = InfraMethod::Auto;
  RouteAlgo routing = RouteAlgo::ALT;
  int kNearest = 12, beam = 2, allocK = 12;
  int64_t alnsIterations = 200;
  double schedBudgetMs = 100, allocBudgetMs = 50, replanBoxMs = 20;
  bool lowBudget = false;
  bool appendOnly = false;
  std::string reason;
};

// Time split (plan): ~10% setup, 15% landmarks + flow, 60% scheduling, 15% slack for events and
// overruns; the last 5% of the wall-clock budget is never planned and is kept for writing outputs.
class BudgetManager {
 public:
  // totalMs: wall-clock budget still available; nominalMs: the configured budget (planning only)
  BudgetManager(double totalMs, double memMB, double nominalMs = -1)
      : total_(totalMs), mem_(memMB), nominal_(nominalMs > 0 ? nominalMs : totalMs) {}
  double totalMs() const { return total_; }
  double elapsedMs() const { return clock_.ms(); }
  double remainingMs() const { return total_ - clock_.ms(); }
  double reserveMs() const { return 0.05 * total_; }  // kept for writing the outputs
  // Deadline leaving the reserve untouched (used as the hard guard for optimisation).
  Deadline hard() const { return Deadline::inMs(std::max(0.0, remainingMs() - reserveMs())); }
  double setupBudgetMs() const { return 0.10 * total_; }
  double landmarkFlowBudgetMs() const { return 0.15 * total_; }
  double schedulingBudgetMs() const { return 0.60 * total_; }
  // Share of what is left (minus reserve) for one of `cyclesLeft` remaining cycles (measured).
  double cycleBudgetMs(int cyclesLeft) const;
  // Deterministic plan: 75% of the budget (after 10% setup and 15% reserve) split evenly over the
  // cycles. Decisions use this so that runs are reproducible; the measured clock is only a guard.
  double plannedCycleMs(int cycles) const { return 0.75 * nominal_ / std::max(1, cycles); }
  double memBudgetMB() const { return mem_; }
  bool memoryPressure() const { return currentMemoryMB() > 0.85 * mem_; }

 private:
  double total_, mem_, nominal_;
  Timer clock_;
};

class MetaSelector {
 public:
  // forced != Auto overrides the scheduling choice (experiments / ablations).
  Strategy choose(const Features& f, const BudgetManager& b, SchedMethod forced) const;
  static int landmarkCount(int64_t V, double memBudgetMB);
};

}  // namespace chronos
