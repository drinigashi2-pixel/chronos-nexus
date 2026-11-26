// replan.hpp - Dependency-Guided Incremental ALNS (our algorithm) and event handling.
//
// DependencyIndex: edge -> missions, ship -> missions, planet -> missions, with lazy
// version stamps (an entry is live iff the mission's plan version still matches).
// An event's invalid set is produced in O(|index entries touched|) instead of O(M * |path|).
// The invalid set (plus Shaw-related neighbours) seeds the ALNS destroy step; repair and a
// short time-boxed ALNS operate only inside that neighbourhood; every other assignment is
// left untouched. If the invalid set exceeds an escalation fraction (15%) of the active
// missions a wider ALNS over all missions is run. Mode Full re-solves everything per event.
#pragma once
#include <string>
#include <unordered_map>
#include <vector>

#include "chronos/flow.hpp"
#include "chronos/predict.hpp"
#include "chronos/scheduler.hpp"

namespace chronos {

enum class ReplanMode : uint8_t { Incremental = 0, Full = 1 };
const char* replanModeName(ReplanMode m);

class DependencyIndex {
 public:
  void clear();
  void indexMission(const World& w, int m);  // O(|plan path|)
  void indexAll(const World& w);
  std::vector<int> missionsOnEdge(const World& w, EdgeId e) const;
  std::vector<int> missionsOfShip(const World& w, int ship) const;
  std::vector<int> missionsFromPlanet(const World& w, NodeId p, int resource) const;  // resource -1 = any
  // Missions made invalid by an (already applied) event.
  std::vector<int> invalidSet(const World& w, const Event& ev) const;
  size_t entries() const { return entries_; }
  size_t memoryBytes() const;

 private:
  using Ref = std::pair<int, uint32_t>;  // mission, plan version
  static bool live(const World& w, const Ref& r);
  std::unordered_map<EdgeId, std::vector<Ref>> edge_;
  std::unordered_map<int, std::vector<Ref>> ship_;
  std::unordered_map<NodeId, std::vector<Ref>> planet_;
  size_t entries_ = 0;
};

// Reference implementation used by the tests: scans every active mission.
std::vector<int> bruteForceInvalidSet(const World& w, const Event& ev);

struct ReplanRecord {
  int eventIndex = -1;
  double time = 0;
  EventType type = EventType::RouteDanger;
  int64_t target = -1;
  ReplanMode mode = ReplanMode::Incremental;
  int invalid = 0, created = 0, cancelled = 0, removed = 0, reinserted = 0, unchanged = 0, active = 0;
  double ms = 0, costBefore = 0, costAfter = 0;
  bool escalated = false;
  std::string note;
};

// Creates missions (chunked to chunkCap) for a shipment; returns their indices.
std::vector<int> materializeShipment(World& w, const Shipment& sh, double now, double chunkCap);
// Bounded epsilon-dominance Pareto route for an emergency / high-importance mission.
bool assignParetoPath(World& w, Router& r, int m, double z);

class Replanner {
 public:
  Replanner(World& w, Router& r, Scheduler& s, ResourceAllocator& a, Predictor& p, DependencyIndex& d);
  void setMode(ReplanMode m) { mode_ = m; }
  ReplanMode mode() const { return mode_; }
  void setTimeBoxMs(double ms) { boxMs_ = ms; }
  void setEscalation(double frac) { escalation_ = frac; }
  void setShawExtra(int n) { shawExtra_ = n; }
  void setChunkCap(double c) { chunkCap_ = c; }
  void setFullIterations(int64_t it) { fullIters_ = it; }

  // Mutates the world for the event. Missions created are returned; timed closures push
  // RouteRestore follow-ups. changedPlanets/changedEdges feed the flow repair.
  std::vector<int> applyEvent(const Event& ev, double now, std::vector<Event>& followUps,
                              std::vector<NodeId>& changedPlanets, std::vector<EdgeId>& changedEdges);
  // Apply + compute invalid set + repair (incremental or full). Hard deadline guards the work.
  ReplanRecord handle(const Event& ev, double now, std::vector<Event>& followUps, const Deadline& hard);

  const std::vector<int>& lastInvalid() const { return lastInvalid_; }
  const std::vector<int>& lastRemoved() const { return lastRemoved_; }
  const std::vector<NodeId>& infraTriggers() const { return infraTriggers_; }
  void clearInfraTriggers() { infraTriggers_.clear(); }
  void reindex(const std::vector<int>* missions);
  // Shipment id (current allocation) -> missions materialised from it.
  void registerShipmentMissions(int shipmentId, const std::vector<int>& missions);
  void clearShipmentMissions() { shipMissions_.clear(); }

 private:
  std::vector<int> missionsOfShipments(const std::vector<int>& shipmentIds) const;
  World& w_;
  Router& router_;
  Scheduler& sched_;
  ResourceAllocator& alloc_;
  Predictor& pred_;
  DependencyIndex& dep_;
  ReplanMode mode_ = ReplanMode::Incremental;
  double boxMs_ = 30, escalation_ = 0.15, chunkCap_ = 500;
  int shawExtra_ = 8;
  int64_t fullIters_ = 300;
  std::vector<int> lastInvalid_, lastRemoved_;
  std::vector<NodeId> infraTriggers_;
  std::unordered_map<int, std::vector<int>> shipMissions_;
};

}  // namespace chronos
