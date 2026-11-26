// score.hpp - civilization score. Every term is normalized to [0,1]. O(1).
//
// Score = 100*[0.30 PS + 0.20 RS + 0.18 MS + 0.12 EG + 0.10 SP + 0.10 TE]
//       - 100*[0.10 MF + 0.08 TC + 0.08 RK + 0.05 WS + 0.04 EN + 0.03 CC]
//
//  PS PopulationSurvival   = surviving population / exposed population (initial + added).
//                            Population on planets in critical (Food/Water/Medicine) shortage
//                            longer than the grace period loses 2% per day (counted as lost).
//  RS ResourceSecurity     = 1 - sum(importance * shortage unit-days) / sum(importance * demand unit-days)
//  MS MissionSuccess       = sum(w * on-time delivered fraction) / sum(w), w = priority*importance,
//                            over missions that reached a terminal state or whose deadline passed.
//  EG EconomicGrowth       = clamp((V_end - V_start) / (0.25 V_start)), V = importance-weighted production
//  SP ScientificProgress   = clamp(delta(sum tech*strategic) / (0.10 * sum tech*strategic at start))
//  TE TerritorialExpansion = established colonies (alive, no critical shortage at end) / colonies
//  MF MissionFailures      = sum(w over failed/expired) / sum(w)
//  TC TransportCost        = clamp(actual transport cost / naive baseline cost); naive = dedicated
//                            round trip per delivered mission on the Balanced shortest path
//  RK Risk                 = mean expected failure probability of executed missions
//  WS Waste                = sum(importance * wasted units) / sum(importance * produced units)
//  EN Energy               = clamp(fuel consumed / naive baseline fuel)
//  CC ComputationalCost    = 0.5 * clamp(time / time budget) + 0.5 * clamp(peak memory / memory budget)
#pragma once
#include <string>

namespace chronos {

struct ScoreInputs {
  double popExposed = 0, popSurvived = 0;
  double shortageWeighted = 0, demandWeighted = 0;
  double missionWeightTotal = 0, missionWeightOnTime = 0, missionWeightFailed = 0;
  double productionValueStart = 0, productionValueEnd = 0;
  double techStart = 0, techEnd = 0;
  int colonies = 0, coloniesEstablished = 0;
  double transportCost = 0, naiveTransportCost = 0;
  double riskSum = 0;
  int riskCount = 0;
  double wasteWeighted = 0, producedWeighted = 0;
  double fuelUsed = 0, naiveFuel = 0;
  double execMs = 0, budgetMs = 1, peakMB = 0, memBudgetMB = 1;
};

struct ScoreBreakdown {
  double populationSurvival = 0, resourceSecurity = 0, missionSuccess = 0, economicGrowth = 0,
         scientificProgress = 0, territorialExpansion = 0;
  double missionFailures = 0, transportCost = 0, risk = 0, waste = 0, energy = 0, computationalCost = 0;
  double positive = 0, negative = 0, total = 0;
};

ScoreBreakdown computeScore(const ScoreInputs& in);
std::string scoreToJson(const ScoreBreakdown& s);

}  // namespace chronos
