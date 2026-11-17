#pragma once
#include <array>
#include <cstdint>
#include <limits>

namespace chronos {

using Id = int32_t;
constexpr Id NONE = -1;
constexpr double INF = std::numeric_limits<double>::infinity();

enum Resource : int {
    FOOD = 0, WATER, ENERGY, FUEL, MEDICINE, RARE_MINERALS, CONSTRUCTION,
    NUM_RESOURCES
};
constexpr int NUM_RES = NUM_RESOURCES;
using ResVec = std::array<double, NUM_RES>;

inline const char* resource_name(int r) {
    static const char* n[NUM_RES] = {"Food", "Water", "Energy", "Fuel",
                                     "Medicine", "RareMinerals", "ConstructionMaterials"};
    return (r >= 0 && r < NUM_RES) ? n[r] : "?";
}

// Spacecraft capability bitmask / mission requirement bitmask.
enum Capability : uint32_t {
    CAP_CARGO = 1, CAP_MEDICAL = 2, CAP_ARMED = 4, CAP_HEAVY = 8
};

}  // namespace chronos
