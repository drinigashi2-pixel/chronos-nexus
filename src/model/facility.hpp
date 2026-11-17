#pragma once
#include "model/types.hpp"

namespace chronos {

enum class FacilityType : int {
    ENERGY_PLANT = 0, FARM, HOSPITAL, FACTORY, DEFENSE_SYSTEM,
    RESEARCH_STATION, SPACEPORT, MINING_FACILITY, NUM_TYPES
};

// One buildable facility type. A decision is a (planet, FacilitySpec) pair.
struct FacilitySpec {
    FacilityType type = FacilityType::ENERGY_PLANT;
    double cost = 0;
    double pop_impact = 0;       // fractional population growth capacity
    double energy_impact = 0;    // net energy per day
    double strategic_value = 0;
    double build_time = 0;       // days
    ResVec resource_impact{};    // change in production per day
};

}  // namespace chronos
