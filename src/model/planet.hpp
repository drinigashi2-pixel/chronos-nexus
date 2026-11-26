#pragma once
#include "model/types.hpp"

namespace chronos {

enum class LocationType : uint8_t { PLANET = 0, COLONY = 1, STATION = 2 };

// All production / consumption values are per DAY. Planet id == graph vertex id.
struct Planet {
    Id id = NONE;
    LocationType type = LocationType::PLANET;
    double x = 0, y = 0, z = 0;
    double population = 0;
    int tech = 1;              // technology level 1..10
    double defense = 0;        // defense level
    double strategic = 0;      // strategic importance 0..1
    ResVec production{};       // units/day
    ResVec consumption{};      // units/day
    ResVec stock{};            // current stored units
    ResVec storage_cap{};      // max storable units
    bool active = true;

    double net_daily(int r) const { return production[r] - consumption[r]; }

    // Days until stock hits zero at the current net rate (INF if not depleting).
    double days_to_shortage(int r) const {
        double n = net_daily(r);
        return n >= 0 ? INF : stock[r] / -n;
    }
};

}  // namespace chronos
