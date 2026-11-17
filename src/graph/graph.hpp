#pragma once
#include "model/types.hpp"

namespace chronos {

struct Point { double x = 0, y = 0, z = 0; };

// Directed route. float keeps 1M+ edges around ~50 MB.
// Failure probability = 1 - reliability. Expected time uses [time_min, time_max].
// NOTE: wormholes (is_wormhole) can be SHORTER than the euclidean distance, so a plain
// euclidean A* heuristic is not admissible across them (use ALT/landmarks or scale it).
struct Edge {
    Id from = NONE, to = NONE;
    float distance = 0;
    float time = 0;        // expected hours for a speed-1.0 ship
    float time_min = 0;
    float time_max = 0;
    float fuel = 0;
    float risk = 0;        // 0..1
    float capacity = 0;    // max cargo per ship / per period
    float reliability = 1; // 1 - failure probability
    uint8_t available = 1;
    uint8_t is_wormhole = 0;
};

// Mission-specific weights for Cost = w1*Dist + w2*Fuel + w3*Time + w4*Risk.
struct CostWeights { double w_dist = 1, w_fuel = 1, w_time = 1, w_risk = 1; };

inline double edge_cost(const Edge& e, const CostWeights& w) {
    return w.w_dist * e.distance + w.w_fuel * e.fuel + w.w_time * e.time + w.w_risk * e.risk;
}

}  // namespace chronos
