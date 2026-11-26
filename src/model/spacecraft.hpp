#pragma once
#include "model/types.hpp"

namespace chronos {

// Travel time on an edge = edge.time / speed  (edge.time is for a speed-1.0 ship).
struct Spacecraft {
    Id id = NONE;
    Id location = NONE;
    double cargo_cap = 0;
    double fuel_cap = 0;
    double speed = 1.0;
    double fuel = 0;
    double avail_time = 0;     // hour at which the ship becomes free
    double reliability = 1.0;  // probability of completing a leg without breakdown
    double op_cost = 0;        // cost per hour
    uint32_t caps = CAP_CARGO; // Capability bitmask
    bool alive = true;
};

}  // namespace chronos
