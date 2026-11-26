// flow.hpp - resource allocation by minimum-cost flow on a REDUCED transportation network.
//
// MinCostFlow: primal-dual successive shortest paths with Johnson potentials.
//   Each phase: binary-heap Dijkstra on reduced costs (O((V+E) log V)), potential update,
//   then a Dinic-style blocking flow on zero-reduced-cost arcs (augments all shortest
//   paths of that length). Integer costs/capacities. Phases <= #distinct path lengths.
//   Warm start: flow and potentials are kept; capacity changes cancel flow by path
//   decomposition and potentials are repaired with SPFA on reduced costs + negative-cycle
//   cancelling (near-linear when few arcs are violated; O(V E) worst case per cycle).
//
// ResourceAllocator: per resource (criticality order) builds
//   S -> [F fleet bottleneck] -> surplus i -> deficit j -> T,  plus shortage arcs S -> j
//   (cost = shortage penalty). Each deficit connects to its K cheapest surplus sources
//   found by ONE reverse bounded Dijkstra per deficit planet shared by all resources (Cargo
//   profile; paths read off the search tree). Network size O(D*K + S).
#pragma once
#include <array>
#include <unordered_map>
#include <vector>

#include "chronos/routing.hpp"

namespace chronos {

class MinCostFlow {
 public:
  using Cap = int64_t;
  using Cost = int64_t;
  struct Result {
    Cap flow = 0;
    Cost cost = 0;
    int phases = 0;
    bool timedOut = false;
  };

  explicit MinCostFlow(int n = 0) { reset(n); }
  void reset(int n);
  int addNode();
  int addArc(int u, int v, Cap cap, Cost cost);  // forward arc id (even); residual twin = id ^ 1
  int numNodes() const { return static_cast<int>(head_.size()); }
  int numArcs() const { return static_cast<int>(to_.size()) / 2; }

  // Augment from s to t until maxFlow or t unreachable (warm-startable).
  Result solve(int s, int t, Cap maxFlow, const Deadline& dl);
  // Push `amount` along a given list of forward arcs (no optimality guarantee).
  Cap pushAlong(const std::vector<int>& arcs, Cap amount);
  // Change capacity; reducing below current flow cancels flow through the arc.
  void setCapacity(int arc, Cap cap, int s, int t);
  // Cancel up to `amount` units of s-t flow passing through `arc`. Returns cancelled amount.
  Cap cancelThrough(int arc, Cap amount, int s, int t);
  // Recompute feasible potentials (SPFA on reduced costs); cancels negative cycles if present.
  // Returns #cycles cancelled, or -1 if the deadline expired (potentials stay dirty).
  int restorePotentials(const Deadline* dl = nullptr);

  Cap flow(int arc) const { return cap0_[arc] - cap_[arc]; }
  Cap capacity(int arc) const { return cap0_[arc]; }
  Cost cost(int arc) const { return cost_[arc]; }
  int from(int arc) const { return to_[arc ^ 1]; }
  int to(int arc) const { return to_[arc]; }
  Cost totalCost() const;
  bool checkConservation(int s, int t) const;  // capacity and conservation constraints
  int64_t settled() const { return settled_; }

 private:
  bool dijkstra(int s, int t);
  Cap blockingFlow(int s, int t, Cap limit);
  Cap dfs(int v, int t, Cap f);
  std::vector<int> head_, next_, to_;
  std::vector<Cap> cap_, cap0_;
  std::vector<Cost> cost_;
  std::vector<Cost> pot_, dist_;
  std::vector<int> level_, it_;
  bool dirty_ = false;
  int64_t settled_ = 0;
};

struct Shipment {
  int id = -1;
  NodeId from = 0, to = 0;
  int resource = 0;
  double amount = 0;
  double unitCost = 0;
  double deadline = INF;             // hours: latest useful arrival
  double pathTime = 0, pathSigma = 0;
  std::vector<EdgeId> path;          // Cargo-profile route used to price the arc
  int arc = -1;                      // transport arc in that resource's network
  bool active = true;                // false once cancelled by repair
};

enum class AllocMethod : uint8_t { MinCostFlow = 0, Greedy = 1 };

struct AllocConfig {
  AllocMethod method = AllocMethod::MinCostFlow;
  int K = 12;                 // candidate sources per deficit
  int settleLimit = 3000;     // reverse search budget per deficit
  double horizonDays = 7;     // look-ahead for deficits
  double coverDays = 10;      // batching: deliver this many days of need at once
  double reserveDays = 3;     // stock kept by suppliers
  double minShipment = 10;    // smaller flows are dropped
  double refShipCap = 500;    // amortisation of trip cost per unit
  double fleetCapacity = 1e18;  // units the fleet can move per cycle (all resources)
  double penaltyScale = 40;   // shortage penalty per unit (times importance*strategic*pop weight)
  int maxShipments = 1000000000;  // trip budget: at most this many shipments per cycle
};

struct AllocationStats {
  double transportCost = 0, shortagePenalty = 0, shortageUnits = 0, wasteUnits = 0, plannedUnits = 0;
  int deficits = 0, supplies = 0, arcs = 0, phases = 0;
  double ms = 0;
  bool timedOut = false;
};

class ResourceAllocator {
 public:
  ResourceAllocator(World& w, Router& r);
  // incoming/outgoing: planet*R units already scheduled to arrive / leave.
  AllocationStats allocate(const AllocConfig& cfg, double now, const std::vector<double>& incoming,
                           const std::vector<double>& outgoing, const Deadline& dl);
  struct RepairResult {
    std::vector<int> cancelled, added;  // shipment ids
    double ms = 0;
    int resourcesTouched = 0;
  };
  // Warm-start repair after events: arcs using changed edges are re-priced or closed,
  // changed planets get updated supply/demand; then re-augment.
  RepairResult repair(const std::vector<EdgeId>& changedEdges, const std::vector<NodeId>& changedPlanets,
                      double now, const Deadline& dl);

  const std::vector<Shipment>& shipments() const { return ship_; }
  const std::vector<double>& shortage() const { return shortage_; }  // planet*R unmet planned demand
  const std::vector<double>& waste() const { return waste_; }        // planet*R projected overflow
  const AllocConfig& config() const { return cfg_; }
  double totalObjective() const;  // transport + shortage penalty of the current plan (all resources)
  size_t memoryBytes() const;

 private:
  struct ArcInfo {
    int kind = 0;  // 0 supply, 1 transport, 2 shortage, 3 demand, 4 fleet
    NodeId i = -1, j = -1;
    std::vector<int> shipments;     // active shipments materialised from this arc
    std::vector<EdgeId> path;       // priced route (read off the reverse search tree)
    double unitCost = 0, pathTime = 0, pathSigma = 0;
    double shipped = 0, dropped = 0;  // units turned into shipments / dropped by the trip cap
  };
  struct Net {
    MinCostFlow mcf;
    int S = -1, T = -1, F = -1;
    std::vector<int> nodeOf;               // planet -> network node (-1)
    std::vector<int> supplyArc, supplyArc2, demandArc, shortArc;  // per planet (-1)
    std::vector<int> transport;            // transport arc ids
    std::vector<ArcInfo> info;             // per forward arc index (arc/2)
    std::vector<double> penalty;           // per planet unit penalty
    bool built = false;
  };
  void balance(NodeId p, int r, double incoming, double outgoing, double& supply, double& demand,
               double& overflow) const;
  double unitPenalty(NodeId j, int r) const;
  double deadlineFor(NodeId j, int r, double now, double incomingUnits, double leadH) const;
  int newArc(Net& N, int u, int v, double cap, double cost, const ArcInfo& info);
  void addSupplyNode(int r, NodeId i, double supply, double overflow);
  void addDeficitNode(int r, NodeId j, double demand);
  int boundCapacities(int r);  // lazy routing of flow-carrying arcs + bottleneck tightening
  void solveResource(int r, const Deadline& dl, AllocationStats* st);
  void collectShipments(int r, double now, std::vector<int>* added, std::vector<int>* cancelled);
  void greedyAllocate(int r, double now);
  void refreshShortageWaste(int r);
  void enforceTripBudget();
  void dropShipment(Shipment& sh);

  World& w_;
  Router& router_;
  AllocConfig cfg_;
  std::array<Net, NUM_RESOURCES> net_;
  std::vector<Shipment> ship_;
  std::vector<double> shortage_, waste_;
  std::vector<double> lastIncoming_, lastOutgoing_;
  std::unordered_map<EdgeId, double> edgeUsed_;
  std::unordered_map<EdgeId, std::vector<std::pair<int, int>>> arcByEdge_;  // edge -> (resource, arc)
  std::array<std::vector<double>, NUM_RESOURCES> supply_, demand_, overflow_;  // per planet
  // Candidate sources per deficit planet, shared by all resources: ONE reverse search per deficit
  // planet collects up to K suppliers of every resource it lacks (with their paths).
  struct SourceCand {
    NodeId v;
    double cost;
    std::vector<EdgeId> path;
  };
  std::unordered_map<NodeId, std::vector<SourceCand>> cands_;
  void precomputeSources(const Deadline& dl);
  double fleetLeft_ = 0;
  double now_ = 0;
  bool inAllocate_ = false;
  Deadline allocDl_ = Deadline::never();
};

}  // namespace chronos
