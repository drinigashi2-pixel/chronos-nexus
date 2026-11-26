// score.cpp - civilization score (definitions in score.hpp). O(1).
#include "chronos/score.hpp"

#include <sstream>

#include "chronos/common.hpp"

namespace chronos {

ScoreBreakdown computeScore(const ScoreInputs& in) {
  ScoreBreakdown s;
  auto ratio = [](double a, double b, double ifZero) { return b > 0 ? a / b : ifZero; };
  s.populationSurvival = clamp01(ratio(in.popSurvived, in.popExposed, 1.0));
  s.resourceSecurity = clamp01(1.0 - ratio(in.shortageWeighted, in.demandWeighted, 0.0));
  s.missionSuccess = clamp01(ratio(in.missionWeightOnTime, in.missionWeightTotal, 1.0));
  s.economicGrowth = clamp01(ratio(in.productionValueEnd - in.productionValueStart, 0.25 * in.productionValueStart, 0.0));
  s.scientificProgress = clamp01(ratio(in.techEnd - in.techStart, 0.10 * in.techStart, 0.0));
  s.territorialExpansion = clamp01(in.colonies > 0 ? static_cast<double>(in.coloniesEstablished) / in.colonies : 1.0);
  s.missionFailures = clamp01(ratio(in.missionWeightFailed, in.missionWeightTotal, 0.0));
  s.transportCost = clamp01(ratio(in.transportCost, in.naiveTransportCost, 0.0));
  s.risk = clamp01(in.riskCount > 0 ? in.riskSum / in.riskCount : 0.0);
  s.waste = clamp01(ratio(in.wasteWeighted, in.producedWeighted, 0.0));
  s.energy = clamp01(ratio(in.fuelUsed, in.naiveFuel, 0.0));
  s.computationalCost =
      0.5 * clamp01(ratio(in.execMs, in.budgetMs, 0.0)) + 0.5 * clamp01(ratio(in.peakMB, in.memBudgetMB, 0.0));
  s.positive = 100.0 * (0.30 * s.populationSurvival + 0.20 * s.resourceSecurity + 0.18 * s.missionSuccess +
                        0.12 * s.economicGrowth + 0.10 * s.scientificProgress + 0.10 * s.territorialExpansion);
  s.negative = 100.0 * (0.10 * s.missionFailures + 0.08 * s.transportCost + 0.08 * s.risk + 0.05 * s.waste +
                        0.04 * s.energy + 0.03 * s.computationalCost);
  s.total = s.positive - s.negative;
  return s;
}

std::string scoreToJson(const ScoreBreakdown& s) {
  std::ostringstream o;
  o << "{\n"
    << "  \"total_score\": " << fmtDouble(s.total, 4) << ",\n"
    << "  \"positive_part\": " << fmtDouble(s.positive, 4) << ",\n"
    << "  \"negative_part\": " << fmtDouble(s.negative, 4) << ",\n"
    << "  \"terms\": {\n"
    << "    \"population_survival\": " << fmtDouble(s.populationSurvival, 6) << ",\n"
    << "    \"resource_security\": " << fmtDouble(s.resourceSecurity, 6) << ",\n"
    << "    \"mission_success\": " << fmtDouble(s.missionSuccess, 6) << ",\n"
    << "    \"economic_growth\": " << fmtDouble(s.economicGrowth, 6) << ",\n"
    << "    \"scientific_progress\": " << fmtDouble(s.scientificProgress, 6) << ",\n"
    << "    \"territorial_expansion\": " << fmtDouble(s.territorialExpansion, 6) << ",\n"
    << "    \"mission_failures\": " << fmtDouble(s.missionFailures, 6) << ",\n"
    << "    \"transport_cost\": " << fmtDouble(s.transportCost, 6) << ",\n"
    << "    \"risk\": " << fmtDouble(s.risk, 6) << ",\n"
    << "    \"waste\": " << fmtDouble(s.waste, 6) << ",\n"
    << "    \"energy\": " << fmtDouble(s.energy, 6) << ",\n"
    << "    \"computational_cost\": " << fmtDouble(s.computationalCost, 6) << "\n"
    << "  },\n"
    << "  \"weights\": {\"population_survival\": 0.30, \"resource_security\": 0.20, \"mission_success\": 0.18, "
       "\"economic_growth\": 0.12, \"scientific_progress\": 0.10, \"territorial_expansion\": 0.10, "
       "\"mission_failures\": 0.10, \"transport_cost\": 0.08, \"risk\": 0.08, \"waste\": 0.05, \"energy\": 0.04, "
       "\"computational_cost\": 0.03}\n"
    << "}\n";
  return o.str();
}

}  // namespace chronos
