// meta.cpp - rule-based algorithm selection and budget split. O(1) per decision.
#include "chronos/meta.hpp"

#include <sstream>

namespace chronos {

double BudgetManager::cycleBudgetMs(int cyclesLeft) const {
  double avail = remainingMs() - reserveMs();
  return std::max(0.0, avail) / std::max(1, cyclesLeft);
}

int MetaSelector::landmarkCount(int64_t V, double memBudgetMB) {
  int L = V <= 20000 ? 16 : (V <= 200000 ? 8 : 4);
  // up to 4 profiles x (forward + reverse) x L x V floats must stay below 20% of the memory budget
  while (L > 2 && 4.0 * 2.0 * L * static_cast<double>(V) * 4.0 / 1e6 > 0.2 * memBudgetMB) L /= 2;
  return L;
}

Strategy MetaSelector::choose(const Features& f, const BudgetManager& b, SchedMethod forced) const {
  Strategy s;
  double cyc = b.plannedCycleMs(f.cyclesTotal);
  // behind schedule by more than 2x (measured): switch to the cheap regime (guard only)
  bool behind = b.cycleBudgetMs(f.cyclesLeft) < 0.5 * cyc;
  // per-cycle split of the non-reserved time: 15/75 allocation, 60/75 scheduling
  s.allocBudgetMs = 0.20 * cyc;
  s.schedBudgetMs = 0.80 * cyc;
  s.replanBoxMs = std::min(200.0, std::max(2.0, 0.02 * cyc));
  bool memPressure = f.memBudgetMB > 0 && f.memUsedMB > 0.85 * f.memBudgetMB;
  s.lowBudget = cyc < 50.0 || memPressure || behind;
  s.alloc = s.lowBudget && f.V > 20000 ? AllocMethod::Greedy : AllocMethod::MinCostFlow;
  s.routing = memPressure ? RouteAlgo::Bidirectional : RouteAlgo::ALT;
  s.kNearest = f.V > 50000 ? 8 : 12;
  s.beam = f.activeMissions > 20000 ? 1 : 2;
  s.allocK = f.V > 50000 ? 10 : 15;
  std::ostringstream why;
  if (forced != SchedMethod::Auto) {
    s.sched = forced;
    why << "forced by configuration";
  } else if (f.activeMissions <= 12 && f.ships <= 40 && cyc > 200.0) {
    s.sched = SchedMethod::BranchAndBound;
    why << "<=12 active missions, <=40 ships and ample budget: exact branch and bound";
  } else if ((f.eventSeverity >= 0.8 && f.minSlackH < 12.0) || s.lowBudget) {
    s.sched = SchedMethod::LocalRepair;
    why << (s.lowBudget ? "low time/memory budget" : "urgent high-severity event") << ": local repair only";
  } else if (f.V > 5000 || f.activeMissions > 3000) {
    s.sched = SchedMethod::GreedyPruneALNS;
    s.kNearest = 6;
    s.beam = 1;
    s.appendOnly = true;
    why << "very large instance: greedy (append-only, K=6) + pruning + short ALNS";
  } else {
    s.sched = SchedMethod::RegretALNS;
    why << "normal regime: regret-2 construction + time-boxed ALNS";
  }
  why << " | V=" << f.V << " E=" << f.E << " active=" << f.activeMissions << " ships=" << f.ships
      << " cycleBudgetMs=" << fmtDouble(cyc, 1) << " minSlackH=" << fmtDouble(f.minSlackH, 1)
      << " mem=" << fmtDouble(f.memUsedMB, 1) << "/" << fmtDouble(f.memBudgetMB, 0)
      << " routeVar=" << fmtDouble(f.routeVariance, 3);
  s.reason = why.str();
  return s;
}

}  // namespace chronos
