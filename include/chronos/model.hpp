// model.hpp - domain entities: planets, spacecraft, missions (+ plans),
// infrastructure options and dynamic events. Plain data, O(1) helpers.
#pragma once
#include <algorithm>
#include <array>
#include <string>
#include <vector>

#include "chronos/common.hpp"

namespace chronos {

enum class PlanetType : uint8_t { Planet = 0, Colony = 1, Station = 2 };

struct Planet {
  int64_t extId = 0;
  PlanetType type = PlanetType::Planet;
  double x = 0, y = 0, z = 0;
  double population = 0, initialPopulation = 0;
  std::array<double, NUM_RESOURCES> production{};   // units per day
  std::array<double, NUM_RESOURCES> consumption{};  // units per day
  std::array<double, NUM_RESOURCES> capacity{};     // storage limit
  std::array<double, NUM_RESOURCES> stock{};        // current stock
  double tech = 1, defense = 1, strategic = 1;      // strategic importance > 0
  bool alive = true;
  bool discovered = false;                          // created by a NewColony event
  std::array<int, NUM_RESOURCES> shortageDays{};    // consecutive days in shortage (critical resources)
  double netRate(int r) const { return production[r] - consumption[r]; }
};

struct Ship {
  int64_t extId = 0;
  NodeId location = 0;          // location once available (after current commitments)
  double cargoCap = 0, fuelCap = 0, fuel = 0, speed = 1;
  double availableAt = 0;       // hours
  double reliability = 0.99, opCost = 1;  // opCost per travel hour
  uint32_t caps = CAP_CARGO;
  bool alive = true;
  // statistics
  double distanceTraveled = 0, fuelUsed = 0;
};

enum class MissionStatus : uint8_t { Pending, Assigned, InFlight, Completed, Failed, Expired, Cancelled };
enum class MissionSource : uint8_t { Input, Proactive, Emergency, EventMission, Construction };

// Planned execution of one mission (output of the scheduler).
struct MissionPlan {
  int ship = -1;
  int seqIndex = -1;
  double plannedStart = 0, arriveOrigin = 0, depart = 0, arrive = 0;
  double sigma = 0, pOnTime = 1, cost = 0;
  int profile = 0;
  std::vector<EdgeId> emptyPath, loadedPath;  // directed edges
  std::vector<NodeId> refuelNodes;
  uint32_t version = 0;                       // bumped on every re-plan (dependency index staleness)
  bool valid = false;
};

struct Mission {
  int64_t extId = 0;
  NodeId origin = 0, dest = 0;
  int resource = 0;
  double amount = 0;
  double release = 0, deadline = 0;   // hours
  int priority = 1;                   // 1..5
  double importance = 0.5;            // 0..1
  uint32_t requiredCaps = CAP_CARGO;
  MissionSource source = MissionSource::Input;
  MissionStatus status = MissionStatus::Pending;
  int64_t parent = -1;                // chunk parent ext id (-1 none)
  int flowKey = -1;                   // allocator shipment id that generated it (-1 none)
  MissionPlan plan;
  std::vector<EdgeId> fixedPath;      // Pareto-selected loaded path (emergency missions)
  // execution results
  int executedBy = -1;
  double actualDepart = 0, actualArrive = 0, delivered = 0;
  bool late = false;
  double weight() const { return priority * importance; }
  double value() const { return 100.0 * priority * importance; }
  bool active() const { return status == MissionStatus::Pending || status == MissionStatus::Assigned; }
};

enum class FacilityType : uint8_t {
  EnergyPlant = 0, Farm, Hospital, Factory, DefenseSystem, ResearchStation, Spaceport, MiningFacility
};
constexpr int NUM_FACILITY_TYPES = 8;

struct FacilityOption {
  int64_t extId = 0;
  NodeId planet = 0;
  int type = 0;
  int level = 1;                      // options of one (planet,type) group are mutually exclusive
  double cost = 0;
  double buildDays = 0;
  std::array<double, NUM_RESOURCES> resourceImpact{};  // production delta per day
  double populationImpact = 0, energyImpact = 0, strategicValue = 0;
  // runtime
  bool built = false, started = false;
  double completeAt = -1;
};

enum class EventType : uint8_t {
  WormholeCollapse = 0, RouteUnavailable, PirateBlockade, ShipDestroyed, EmergencyMedical, ProductionDrop,
  NewColony, PopulationSurge, ResourceUnavailable, NewHighPriorityMission, RouteDanger, ShipFailure,
  RouteRestore  // internal follow-up of timed route closures (not in input files)
};
constexpr int NUM_EVENT_TYPES = 12;

// Generic event record; meaning of a..d depends on type (see docs/FORMAT.md).
struct Event {
  double time = 0;
  EventType type = EventType::RouteDanger;
  int64_t target = -1;   // external id of route / ship / planet (-1 = none/global)
  double severity = 0;
  double a = 0, b = 0, c = 0, d = 0;
  int index = -1;        // order in the input file
};

// ---- inline name tables (header-only so no extra translation unit is needed)
inline const char* missionStatusName(MissionStatus s) {
  static const char* n[] = {"pending", "assigned", "in_flight", "completed", "failed", "expired", "cancelled"};
  return n[static_cast<int>(s)];
}
inline const char* missionSourceName(MissionSource s) {
  static const char* n[] = {"input", "proactive", "emergency", "event", "construction"};
  return n[static_cast<int>(s)];
}
inline const char* facilityTypeName(int t) {
  static const char* n[] = {"EnergyPlant", "Farm", "Hospital", "Factory", "DefenseSystem",
                            "ResearchStation", "Spaceport", "MiningFacility"};
  return (t >= 0 && t < NUM_FACILITY_TYPES) ? n[t] : "Unknown";
}
inline int facilityTypeFromString(const std::string& s) {
  if (!s.empty() && std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; })) {
    int v = std::atoi(s.c_str());
    return (v >= 0 && v < NUM_FACILITY_TYPES) ? v : -1;
  }
  for (int t = 0; t < NUM_FACILITY_TYPES; ++t) {
    std::string a = facilityTypeName(t), b = s;
    for (auto& ch : a) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    for (auto& ch : b) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    b.erase(std::remove(b.begin(), b.end(), '_'), b.end());
    b.erase(std::remove(b.begin(), b.end(), ' '), b.end());
    if (a == b) return t;
  }
  return -1;
}
inline const char* eventTypeName(EventType t) {
  static const char* n[] = {"WormholeCollapse", "RouteUnavailable", "PirateBlockade", "ShipDestroyed",
                            "EmergencyMedical", "ProductionDrop", "NewColony", "PopulationSurge",
                            "ResourceUnavailable", "NewHighPriorityMission", "RouteDanger", "ShipFailure",
                            "RouteRestore"};
  return n[static_cast<int>(t)];
}
inline int eventTypeFromString(const std::string& s) {
  if (!s.empty() && std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; })) {
    int v = std::atoi(s.c_str());
    return (v >= 0 && v < NUM_EVENT_TYPES) ? v : -1;
  }
  std::string b = s;
  for (auto& ch : b) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  b.erase(std::remove(b.begin(), b.end(), '_'), b.end());
  for (int t = 0; t < NUM_EVENT_TYPES; ++t) {
    std::string a = eventTypeName(static_cast<EventType>(t));
    for (auto& ch : a) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    if (a == b) return t;
  }
  return -1;
}

}  // namespace chronos
