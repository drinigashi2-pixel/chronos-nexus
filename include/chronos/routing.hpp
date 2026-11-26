// routing.hpp - multi-objective route planning.
//
//  * Dijkstra (baseline)                 O((V+E) log V) time, O(V) space
//  * Bidirectional Dijkstra (baseline)   same worst case, ~sqrt fewer settles on geometric graphs
//  * A* with ALT landmarks               same worst case; typically 5-50x fewer settles
//      tables: O(L * V) floats per cost profile, built with 2L full Dijkstras
//  * Fuel-constrained label search over (node, fuel bucket) with dominance:
//                                         O(B (V+E) log(BV)), B = fuel buckets
//  * Bounded epsilon-dominance Pareto labels (time, risk): O(Lmax (V+E) log(Lmax V))
//  * Route cache keyed (src,dst,profile,capacity class) + inverted index
//    edge -> cached keys for O(#affected) targeted invalidation.
//
// Edge cost for profile p:
//   c(e) = w1*dist + w2*fuel + w3*(mean + k*sigma) + w4*(risk + relPenalty*(-ln rel)),
//   sigma = (max - min)/4. Unavailable edges (or capacity below the request) cost INF.
#pragma once
#include <array>
#include <functional>
#include <unordered_map>
#include <vector>

#include "chronos/graph.hpp"

namespace chronos {

enum ProfileId : int { PROFILE_BALANCED = 0, PROFILE_EMERGENCY = 1, PROFILE_CARGO = 2, PROFILE_SAFE = 3,
                       PROFILE_TIME = 4 };
// 4 mission profiles + 1 internal pure-travel-time profile (used for deadline lower bounds).
constexpr int NUM_PROFILES = 5;

struct CostProfile {
  double wDist, wFuel, wTime, wRisk, kSigma, relPenalty;
  const char* name;
};
const CostProfile& costProfile(int p);
double edgeCost(const DynamicGraph& g, EdgeId e, int profile);  // INF when unavailable

struct Path {
  bool found = false;
  std::vector<EdgeId> edges;
  double cost = INF;
  double dist = 0, fuel = 0, timeMean = 0, timeVar = 0;
  double risk = 0;          // 1 - prod(1 - risk_e)
  double reliability = 1;   // prod(rel_e)
  double bottleneck = INF;  // min edge capacity
  std::vector<NodeId> refuelNodes;  // fuel-constrained paths only
  double fuelAtArrival = -1;        // fuel-constrained paths only
};
void fillPathMetrics(const DynamicGraph& g, Path& p, int profile);  // O(|path|)
std::vector<NodeId> pathNodes(const DynamicGraph& g, const std::vector<EdgeId>& edges, NodeId src);
bool pathUsable(const DynamicGraph& g, const std::vector<EdgeId>& edges, double minCap);

// Reusable search buffers with O(1) reset via generation stamps.
class SearchWorkspace {
 public:
  void ensure(int n);
  void reset();
  double dist(int v) const { return stamp_[v] == cur_ ? dist_[v] : INF; }
  EdgeId pred(int v) const { return stamp_[v] == cur_ ? pred_[v] : -1; }
  void set(int v, double d, EdgeId p) {
    stamp_[v] = cur_;
    dist_[v] = d;
    pred_[v] = p;
    closed_[v] = 0;
  }
  bool closed(int v) const { return stamp_[v] == cur_ && closed_[v]; }
  void close(int v) { closed_[v] = 1; }
  size_t memoryBytes() const;

 private:
  std::vector<double> dist_;
  std::vector<EdgeId> pred_;
  std::vector<uint32_t> stamp_;
  std::vector<uint8_t> closed_;
  uint32_t cur_ = 0;
};

// ALT (A*, Landmarks, Triangle inequality) tables, one per cost profile, built lazily.
// Admissibility under dynamic costs: a per-profile slack = sum over edges of
// max(0, cost_at_build - cost_now) is subtracted from the bound (provably admissible);
// edges unavailable at build time that become usable, and new edges, make the table stale
// (bound 0 = Dijkstra) until rebuilt. Rebuild is requested after K decreases or staleness.
class AltIndex {
 public:
  void setLandmarkCount(int L) { L_ = std::max(1, L); }
  int landmarkCount() const { return L_; }
  void setRebuildThreshold(int k) { rebuildAfter_ = std::max(1, k); }
  void build(const DynamicGraph& g, int profile, SearchWorkspace& ws);
  bool built(int profile) const { return t_[profile].built; }
  bool stale(int profile) const { return t_[profile].built && t_[profile].stale; }
  bool needsRebuild(int profile) const {
    return t_[profile].built && (t_[profile].stale || t_[profile].decreases >= rebuildAfter_);
  }
  double lowerBound(int profile, NodeId v, NodeId t) const;
  double slack(int profile) const { return t_[profile].slack; }
  // Per-query bound evaluator: the target rows are fetched once, so each node costs one pass
  // over its own two rows. Same value as lowerBound(profile, v, t).
  class Bound {
   public:
    Bound() = default;
    Bound(const AltIndex& a, int profile, NodeId t);
    double operator()(NodeId v) const;
    bool active() const { return on_; }
   private:
    const float *fwd_ = nullptr, *rev_ = nullptr, *ft_ = nullptr, *rt_ = nullptr;
    size_t stride_ = 0, n_ = 0;
    int L_ = 0;
    NodeId t_ = -1;
    double slack_ = 0;
    bool on_ = false;
  };
  // With g given, an edge that was unusable at build time and becomes usable is repaired in place
  // (see repairEdge) instead of making the table stale.
  void onEdgeChange(EdgeId e, const std::array<double, NUM_PROFILES>& oldCost,
                    const std::array<double, NUM_PROFILES>& newCost, const DynamicGraph* g = nullptr);
  void onEdgeAdded();  // new edge: all tables stale
  // New edge (or new node + edges): repair every built table in place. The tables stay feasible
  // potentials of the build-time costs (fwd[v] <= fwd[u] + c_B(u,v), rev[u] <= c_B(u,v) + rev[v]),
  // which is all the ALT bound needs; a decrease-only Dijkstra per landmark and direction touches
  // only the nodes whose landmark distance actually drops. Rows for new nodes are appended.
  void onEdgeAdded(const DynamicGraph& g, EdgeId e);
  void dropAll();
  size_t memoryBytes() const;
  const std::vector<NodeId>& landmarks(int profile) const { return t_[profile].lm; }

 private:
  struct Table;
  double buildCost(const Table& T, const DynamicGraph& g, EdgeId e, int profile) const;
  void repairEdge(Table& T, const DynamicGraph& g, int profile, EdgeId e, double c);
  struct Table {
    std::vector<NodeId> lm;
    std::vector<float> fwd, rev;  // [v * L + l]: d(L_l, v) and d(v, L_l)
    size_t n = 0;
    int L = 0;
    bool built = false, stale = false;
    int decreases = 0;
    double slack = 0;
    std::unordered_map<EdgeId, std::pair<double, double>> changed;  // edge -> (cost at build, deficit)
  };
  int L_ = 8, rebuildAfter_ = 8;
  std::array<Table, NUM_PROFILES> t_;
};

// Route memo keyed (src,dst,profile,capacity class) with inverted edge index.
class RouteCache {
 public:
  explicit RouteCache(size_t maxEntries = 200000) : max_(maxEntries) {}
  static uint64_t key(NodeId s, NodeId t, int profile, int capClass) {
    return (static_cast<uint64_t>(s) << 36) | (static_cast<uint64_t>(t) << 8) |
           (static_cast<uint64_t>(profile) << 5) | static_cast<uint64_t>(capClass);
  }
  const Path* find(uint64_t key) const {
    int32_t i = index_.find(key);
    return i < 0 ? nullptr : &paths_[static_cast<size_t>(i)];
  }
  void insert(uint64_t key, const Path& p);
  int invalidateEdge(EdgeId e);  // returns #entries removed
  // Drops every entry for which drop(key, path) is true; returns #entries removed.
  template <class F>
  int eraseIf(F&& drop) {
    std::vector<uint64_t> gone;
    index_.forEach([&](uint64_t k, int32_t i) {
      if (drop(k, paths_[static_cast<size_t>(i)])) gone.push_back(k);
    });
    for (uint64_t k : gone) {
      int32_t i = index_.erase(k);
      if (i >= 0) {
        paths_[static_cast<size_t>(i)] = Path();
        free_.push_back(i);
      }
    }
    return static_cast<int>(gone.size());
  }
  static NodeId keySource(uint64_t k) { return static_cast<NodeId>(k >> 36); }
  static NodeId keyTarget(uint64_t k) { return static_cast<NodeId>((k >> 8) & 0xFFFFFFFULL); }
  static int keyProfile(uint64_t k) { return static_cast<int>((k >> 5) & 7ULL); }
  void clear();
  void setCapacity(size_t n) { max_ = n; }
  size_t size() const { return index_.size(); }
  size_t memoryBytes() const;

 private:
  // key -> slot in paths_ (flat open addressing); freed slots are recycled
  FlatIndex index_;
  std::vector<Path> paths_;
  std::vector<int32_t> free_;
  std::unordered_map<EdgeId, std::vector<uint64_t>> byEdge_;
  size_t max_;
  size_t edgeRefs_ = 0;
};

enum class RouteAlgo : uint8_t { Dijkstra = 0, Bidirectional = 1, ALT = 2 };
const char* routeAlgoName(RouteAlgo a);

class Router {
 public:
  explicit Router(const DynamicGraph& g);

  void setAlgo(RouteAlgo a) { algo_ = a; }
  RouteAlgo algo() const { return algo_; }
  void setCacheEnabled(bool on) { cacheOn_ = on; }
  void setRebuildAllowed(bool on) { rebuildAllowed_ = on; }
  void setNaiveSampling(int every) { sampleEvery_ = every; }  // 0 = off
  // ALT tables are also used for lower bounds when paths are computed by another algorithm.
  void setBoundsEnabled(bool on) { boundsOn_ = on; }
  AltIndex& alt() { return alt_; }
  RouteCache& cache() { return cache_; }

  // Capacity class: smallest power of two >= minCap (class 0 = no capacity filter).
  static int capClass(double minCap);
  static double capOfClass(int c) { return c <= 0 ? 0.0 : static_cast<double>(1ULL << (c - 1)); }

  // Best path under `profile` using edges with capacity >= capOfClass(capClass(minCap)). Cached.
  // The returned reference stays valid until the next non-const Router call.
  const Path& route(NodeId s, NodeId t, int profile, double minCap = 0);
  // Uncached dispatch to the configured algorithm; `bound` prunes paths costing more.
  Path compute(NodeId s, NodeId t, int profile, double minCap = 0, double bound = INF);

  Path dijkstra(NodeId s, NodeId t, int profile, double minCap = 0, double bound = INF);
  Path bidirectional(NodeId s, NodeId t, int profile, double minCap = 0);
  Path astar(NodeId s, NodeId t, int profile, double minCap = 0, double bound = INF);
  // Admissible lower bound on the profile cost s->t (0 if ALT unavailable).
  double lowerBound(NodeId s, NodeId t, int profile);
  // Full single-source distances (forward or reverse graph); returns #settled.
  int fullDijkstra(NodeId s, int profile, bool reverse, std::vector<double>& out);

  // Fuel-constrained label search: start with fuelStart (<= fuelCap); refuel to fuelCap at
  // nodes with refuel[v] != 0 (cost refuelPenalty each). Labels (node, fuel bucket) with
  // dominance (cost <=, fuel >=) and duplicate-state elimination.
  Path routeFuel(NodeId s, NodeId t, int profile, double fuelCap, double fuelStart,
                 const std::vector<uint8_t>& refuel, double minCap = 0, int buckets = 16,
                 double refuelPenalty = 5.0);

  // Bounded epsilon-dominance Pareto front on (time mean + 2 sigma, -ln reliability + risk).
  std::vector<Path> paretoPaths(NodeId s, NodeId t, double minCap = 0, int maxLabelsPerNode = 6,
                                double eps = 0.05, int maxTotalLabels = 30000);

  // Reverse bounded Dijkstra from t: up to k nodes v (v != t) with isSource(v), cheapest first,
  // stopping after settleLimit settles. out = (source, cost source->t). If paths != nullptr the
  // shortest path source->t of each result is read off the search tree (no extra searches).
  void nearestSources(NodeId t, int profile, int k, int settleLimit, const std::function<bool(NodeId)>& isSource,
                      std::vector<std::pair<NodeId, double>>& out,
                      std::vector<std::vector<EdgeId>>* paths = nullptr, double minCap = 0);

  // Dynamic updates. Call edgeCosts(e) BEFORE mutating, then onEdgeChanged(e, old) after.
  std::array<double, NUM_PROFILES> edgeCosts(EdgeId e) const;
  int onEdgeChanged(EdgeId e, const std::array<double, NUM_PROFILES>& oldCost);  // returns #cache entries dropped
  void onEdgeAdded(EdgeId e);
  void onNodeAdded();
  // After edge e became cheaper (or was added): drop only the cached paths that e could improve.
  // A cached s->t path of cost C stays optimal unless LB(s,from e) + c(e) + LB(to e,t) < C, with
  // ALT lower bounds; profiles without a usable table are dropped wholesale. Returns #dropped.
  int invalidateForCheaperEdge(EdgeId e);
  void maintain();  // rebuild ALT tables that requested it (if allowed); called at cycle boundaries
  void ensureAlt(int profile);
  size_t memoryBytes() const;

 private:
  // Per-profile edge cost arrays (lazily filled, appended for new edges, updated on change).
  double cost(EdgeId e, int p) {
    auto& v = costs_[p];
    if (static_cast<int>(v.size()) <= e) {
      for (int x = static_cast<int>(v.size()); x < g_.numEdges(); ++x) v.push_back(edgeCost(g_, x, p));
    }
    return v[e];
  }
  Path extract(NodeId s, NodeId t, const SearchWorkspace& ws, int profile) const;
  void sampleNaive(NodeId s, NodeId t, int profile, double minCap, uint64_t actualSettled);
  const DynamicGraph& g_;
  RouteAlgo algo_ = RouteAlgo::ALT;
  AltIndex alt_;
  RouteCache cache_;
  SearchWorkspace ws_, ws2_;
  // fuel-label search buffers (reused across calls; O(1) reset by stamp)
  std::vector<double> fuelMin_;     // [v*B + b]: min settled cost over buckets >= b
  std::vector<uint32_t> fuelStamp_;  // per node
  std::vector<double> hMemo_;       // per node heuristic memo for the current query
  std::vector<uint32_t> hStamp_;
  std::vector<uint32_t> hClosed_;   // fuel search: nodes settled by the bounded reverse sweep
  uint32_t fuelCur_ = 0;
  int fuelB_ = 0;
  struct FuelLabel {
    double cost, fuel;
    int node, pred;
    EdgeId edge;  // -1 for a refuel step
  };
  std::vector<FuelLabel> fuelLabels_;
  std::vector<std::vector<int>> paretoPerm_;  // Pareto search: node -> permanent labels
  std::vector<int> paretoTouched_;
  std::vector<std::pair<double, int>> fuelHeap_;
  Path scratch_;
  std::array<std::vector<double>, NUM_PROFILES> costs_;
  bool cacheOn_ = true, rebuildAllowed_ = true, boundsOn_ = false;
  int sampleEvery_ = 64;
  uint64_t maxSamples_ = 300;
  uint64_t uncached_ = 0;
};

}  // namespace chronos
