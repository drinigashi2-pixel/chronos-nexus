// graph.hpp - dynamic weighted directed galaxy graph + the World state.
//
// Layout: CSR forward and reverse adjacency holding edge ids; per-edge
// attributes are structure-of-arrays (float32) and mutable in O(1).
// New edges go to a linked overflow buffer (O(1) insert) and are merged into
// the CSR when the buffer exceeds a threshold (O(V+E) rebuild, amortised).
// Every mutation bumps a version counter and is appended to a change log.
// Space: O(V + E).
#pragma once
#include <string>
#include <unordered_map>
#include <vector>

#include "chronos/model.hpp"

namespace chronos {

struct EdgeAttr {
  NodeId from = 0, to = 0;
  float dist = 0, tmean = 0, tmin = 0, tmax = 0, fuel = 0, risk = 0, cap = 0, rel = 1;
  bool available = true, wormhole = false;
  int64_t routeId = -1;
};

enum class EdgeChangeKind : uint8_t { Increase, Decrease, Added, Disabled, Enabled };
struct EdgeChange {
  EdgeId edge;
  uint64_t version;
  EdgeChangeKind kind;
};

class DynamicGraph {
 public:
  // Build from scratch. O(V + E).
  void build(int numNodes, const std::vector<EdgeAttr>& edges);
  int numNodes() const { return n_; }
  int numEdges() const { return static_cast<int>(from_.size()); }

  NodeId addNode();                          // O(1) amortised
  EdgeId addEdge(const EdgeAttr& a);         // O(1) (overflow buffer)
  void compact();                            // merge overflow into CSR, O(V + E)
  size_t overflowSize() const { return ovEdgeOut_.size(); }

  // Iterate outgoing edges: f(EdgeId e, NodeId head). O(deg).
  template <class F>
  void forEachOut(NodeId u, F&& f) const {
    for (int i = rowOut_[u]; i < rowOut_[u + 1]; ++i) f(adjOut_[i], to_[adjOut_[i]]);
    for (int i = ovHeadOut_[u]; i >= 0; i = ovNextOut_[i]) f(ovEdgeOut_[i], to_[ovEdgeOut_[i]]);
  }
  // Iterate incoming edges: f(EdgeId e, NodeId tail). O(deg).
  template <class F>
  void forEachIn(NodeId v, F&& f) const {
    for (int i = rowIn_[v]; i < rowIn_[v + 1]; ++i) f(adjIn_[i], from_[adjIn_[i]]);
    for (int i = ovHeadIn_[v]; i >= 0; i = ovNextIn_[i]) f(ovEdgeIn_[i], from_[ovEdgeIn_[i]]);
  }
  int outDegree(NodeId u) const;

  // Attribute access, O(1).
  NodeId from(EdgeId e) const { return from_[e]; }
  NodeId to(EdgeId e) const { return to_[e]; }
  float dist(EdgeId e) const { return dist_[e]; }
  float tmean(EdgeId e) const { return tmean_[e]; }
  float tmin(EdgeId e) const { return tmin_[e]; }
  float tmax(EdgeId e) const { return tmax_[e]; }
  float sigma(EdgeId e) const { return (tmax_[e] - tmin_[e]) * 0.25f; }
  float fuel(EdgeId e) const { return fuel_[e]; }
  float risk(EdgeId e) const { return risk_[e]; }
  float cap(EdgeId e) const { return cap_[e]; }
  float rel(EdgeId e) const { return rel_[e]; }
  bool available(EdgeId e) const { return avail_[e] != 0; }
  bool wormhole(EdgeId e) const { return worm_[e] != 0; }
  int64_t routeId(EdgeId e) const { return route_[e]; }
  EdgeAttr attr(EdgeId e) const;

  // Mutations, O(1) each; logged with a new version.
  void setAvailable(EdgeId e, bool on);
  void setRisk(EdgeId e, float risk);
  void setReliability(EdgeId e, float rel);
  void setTimes(EdgeId e, float tmin, float tmean, float tmax);

  uint64_t version() const { return version_; }
  const std::vector<EdgeChange>& changeLog() const { return log_; }
  size_t memoryBytes() const;

 private:
  void log(EdgeId e, EdgeChangeKind k) { log_.push_back({e, ++version_, k}); }
  void rebuildCsr();
  int n_ = 0;
  std::vector<int> rowOut_{0}, adjOut_, rowIn_{0}, adjIn_;
  std::vector<int> ovHeadOut_, ovNextOut_, ovEdgeOut_, ovHeadIn_, ovNextIn_, ovEdgeIn_;
  std::vector<NodeId> from_, to_;
  std::vector<float> dist_, tmean_, tmin_, tmax_, fuel_, risk_, cap_, rel_;
  std::vector<uint8_t> avail_, worm_;
  std::vector<int64_t> route_;
  uint64_t version_ = 0;
  std::vector<EdgeChange> log_;
};

// Complete mutable state of the civilization. Planet index == graph node id.
struct World {
  DynamicGraph graph;
  std::vector<Planet> planets;
  std::vector<Ship> ships;
  std::vector<Mission> missions;
  std::vector<FacilityOption> facilities;
  std::vector<Event> events;
  std::unordered_map<int64_t, int> planetIdx, shipIdx;
  std::unordered_map<int64_t, std::vector<EdgeId>> routeEdges;  // route id -> directed edges
  std::vector<double> frozenUntil;   // planet*R -> supply frozen until (hours)
  std::vector<double> extraDemand;   // planet*R -> pending construction demand (units)
  double budget = 0;                 // construction budget
  int days = 14;                     // simulated days
  double now = 0;                    // hours
  int64_t nextMissionExt = 1, nextPlanetExt = 1, nextRouteExt = 1;

  int numPlanets() const { return static_cast<int>(planets.size()); }
  int planetOf(int64_t ext) const;
  int shipOf(int64_t ext) const;
  const std::vector<EdgeId>* edgesOfRoute(int64_t ext) const;
  bool supplyFrozen(int p, int r) const { return frozenUntil[static_cast<size_t>(p) * NUM_RESOURCES + r] > now; }
  bool isRefuelNode(NodeId v) const;  // stations, or planets holding fuel stock
  NodeId addPlanet(const Planet& p);  // also adds the graph vertex
  int addMission(Mission m);          // assigns ext id when <= 0; returns index
  void finalizeIndexes();             // (re)size per-planet arrays
  std::vector<uint8_t> refuelMask() const;
};

}  // namespace chronos
