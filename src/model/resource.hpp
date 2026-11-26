#pragma once
#include "model/types.hpp"

namespace chronos {

// Global per-resource parameters used by scoring / allocation.
struct ResourceInfo {
    const char* name;
    double importance;         // weight in the CivilizationScore
    double shortage_penalty;   // cost per unit of unmet demand (min-cost-flow penalty arc)
    double volume_per_unit;    // cargo space one unit takes
};

inline const ResourceInfo& resource_info(int r) {
    // PLACEHOLDER weights: justify / tune these in the report.
    static const ResourceInfo t[NUM_RES] = {
        {"Food",                  1.0, 10.0, 1.0},
        {"Water",                 1.0, 10.0, 1.0},
        {"Energy",                0.8,  6.0, 1.0},
        {"Fuel",                  0.7,  5.0, 1.0},
        {"Medicine",              1.5, 25.0, 0.5},
        {"RareMinerals",          0.5,  3.0, 0.5},
        {"ConstructionMaterials", 0.4,  2.0, 1.5},
    };
    return t[r];
}

}  // namespace chronos
