#pragma once
#include "model/types.hpp"

namespace chronos {

// Matches the 12 example events in the spec (order is part of the file format).
enum class EventType : int {
    WORMHOLE_COLLAPSE = 0,      // target = edge id
    ROUTE_UNAVAILABLE,          // target = edge id
    PIRATE_BLOCKADE,            // target = edge id
    SPACECRAFT_DESTROYED,       // target = spacecraft id
    EMERGENCY_MEDICAL_REQUEST,  // target = planet id, p1 = resource, p2 = amount
    PRODUCTION_DROP,            // target = planet id, severity = fraction lost, p1 = resource
    NEW_COLONY,                 // target = NONE, p1 = x, p2 = y
    POPULATION_SURGE,           // target = planet id, severity = growth fraction
    RESOURCE_UNAVAILABLE,       // target = planet id, p1 = resource, p2 = duration (hours)
    NEW_PRIORITY_MISSION,       // target = destination planet, p1 = resource, p2 = amount
    ROUTE_DANGER_INCREASE,      // target = edge id, severity = risk increase
    SPACECRAFT_FAILURE,         // target = spacecraft id, p2 = downtime (hours)
    NUM_TYPES
};

inline const char* event_name(EventType t) {
    static const char* n[] = {
        "WormholeCollapse", "RouteUnavailable", "PirateBlockade", "SpacecraftDestroyed",
        "EmergencyMedicalRequest", "ProductionDrop", "NewColony", "PopulationSurge",
        "ResourceUnavailable", "NewPriorityMission", "RouteDangerIncrease", "SpacecraftFailure"};
    int i = static_cast<int>(t);
    return (i >= 0 && i < static_cast<int>(EventType::NUM_TYPES)) ? n[i] : "?";
}

struct Event {
    double time = 0;  // hours
    EventType type = EventType::ROUTE_UNAVAILABLE;
    Id target = NONE;
    double severity = 0;
    double p1 = 0, p2 = 0;
};

}  // namespace chronos
