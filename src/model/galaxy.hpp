#pragma once
#include <vector>
#include "graph/dynamic_graph.hpp"
#include "model/event.hpp"
#include "model/facility.hpp"
#include "model/mission.hpp"
#include "model/planet.hpp"
#include "model/spacecraft.hpp"

namespace chronos {

// Complete simulation state. planets[i] corresponds to graph vertex i.
struct Galaxy {
    DynamicGraph graph;
    std::vector<Planet> planets;
    std::vector<Spacecraft> spacecraft;
    std::vector<Mission> missions;
    std::vector<FacilitySpec> facility_catalog;
    double budget = 0;
    std::vector<Event> events;  // sorted by time
};

}  // namespace chronos
