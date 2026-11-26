// routing.cpp - Dijkstra, bidirectional Dijkstra, ALT A*, fuel-constrained labels,
// bounded epsilon-dominance Pareto labels, route cache. See routing.hpp for complexities.
#include <cstdlib>
#include "chronos/routing.hpp"

#include <algorithm>
#include <queue>
#include <unordered_map>

namespace chronos {

namespace {
// Weights chosen so each term has a comparable magnitude on generated data
// (dist ~10-100, fuel ~10-80, time ~5-60 h, risk 0-0.3): see README "Route cost".
const CostProfile PROFILES[NUM_PROFILES] = {
    {0.40, 0.30, 0.30, 40.0, 1.0, 1.0, "balanced"},
    {0.05, 0.05, 1.00, 60.0, 2.0, 2.0, "emergency"},
    {0.30, 0.80, 0.10, 15.0, 0.5, 1.0, "cargo"},
    {0.20, 0.20, 0.20, 200.0, 1.5, 3.0, "safe"},
    {0.00, 0.00, 1.00, 0.0, 0.0, 0.0, "time"},
};
using QItem = std::pair<double, int>;
using MinHeap = std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>>;
}  // namespace

const CostProfile& costProfile(int p) { return PROFILES[(p >= 0 && p < NUM_PROFILES) ? p : 0]; }

const char* routeAlgoName(RouteAlgo a) {
  switch (a) {
    case RouteAlgo::Dijkstra: return "dijkstra";
    case RouteAlgo::Bidirectional: return "bidirectional";
    default: return "alt_astar";
  }
}

double edgeCost(const DynamicGraph& g, EdgeId e, int profile) {
  if (!g.available(e)) return INF;
  const CostProfile& c = PROFILES[profile];
  double rel = g.rel(e);
  double relTerm = rel >= 1.0 ? 0.0 : -std::log(rel);
  return c.wDist * g.dist(e) + c.wFuel * g.fuel(e) + c.wTime * (g.tmean(e) + c.kSigma * g.sigma(e)) +
         c.wRisk * (g.risk(e) + c.relPenalty * relTerm);
}

void fillPathMetrics(const DynamicGraph& g, Path& p, int profile) {
  p.dist = p.fuel = p.timeMean = p.timeVar = 0;
  p.reliability = 1;
  p.bottleneck = INF;
  double safe = 1;
  double cost = 0;
  for (EdgeId e : p.edges) {
    p.dist += g.dist(e);
    p.fuel += g.fuel(e);
    p.timeMean += g.tmean(e);
    double s = g.sigma(e);
    p.timeVar += s * s;
    safe *= (1.0 - g.risk(e));
    p.reliability *= g.rel(e);
    p.bottleneck = std::min(p.bottleneck, static_cast<double>(g.cap(e)));
    cost += edgeCost(g, e, profile);
  }
  p.risk = 1.0 - safe;
  p.cost = cost;
}

std::vector<NodeId> pathNodes(const DynamicGraph& g, const std::vector<EdgeId>& edges, NodeId src) {
  std::vector<NodeId> nodes;
  nodes.reserve(edges.size() + 1);
  nodes.push_back(src);
  for (EdgeId e : edges) nodes.push_back(g.to(e));
  return nodes;
}

bool pathUsable(const DynamicGraph& g, const std::vector<EdgeId>& edges, double minCap) {
  for (EdgeId e : edges)
    if (e < 0 || e >= g.numEdges() || !g.available(e) || g.cap(e) < minCap) return false;
  return true;
}

// ------------------------------------------------------------------ workspace
void SearchWorkspace::ensure(int n) {
  if (static_cast<int>(dist_.size()) >= n) return;
  dist_.resize(n, INF);
  pred_.resize(n, -1);
  stamp_.resize(n, 0);
  closed_.resize(n, 0);
}

void SearchWorkspace::reset() {
  if (++cur_ == 0) {
    std::fill(stamp_.begin(), stamp_.end(), 0u);
    cur_ = 1;
  }
}

size_t SearchWorkspace::memoryBytes() const {
  return dist_.size() * (sizeof(double) + sizeof(EdgeId) + sizeof(uint32_t) + 1);
}

// ------------------------------------------------------------------ ALT
void AltIndex::build(const DynamicGraph& g, int profile, SearchWorkspace& ws) {
  (void)ws;
  Table& T = t_[profile];
  int n = g.numNodes();
  T = Table();
  T.n = static_cast<size_t>(n);
  T.L = std::min(L_, std::max(1, n));
  if (n == 0) {
    T.built = true;
    return;
  }
  T.fwd.assign(static_cast<size_t>(n) * T.L, std::numeric_limits<float>::infinity());
  T.rev.assign(static_cast<size_t>(n) * T.L, std::numeric_limits<float>::infinity());
  std::vector<double> d(n), minD(n, INF);
  std::vector<char> chosen(n, 0);
  // Dijkstra over forward or reverse adjacency; O((V+E) log V).
  auto full = [&](NodeId s, bool reverse) {
    std::fill(d.begin(), d.end(), INF);
    MinHeap pq;
    d[s] = 0;
    pq.push({0, s});
    uint64_t settled = 0;
    while (!pq.empty()) {
      double du = pq.top().first;
      int u = pq.top().second;
      pq.pop();
      if (du > d[u]) continue;
      ++settled;
      auto relax = [&](EdgeId e, NodeId v) {
        double c = edgeCost(g, e, profile);
        if (!std::isfinite(c)) return;
        if (du + c < d[v]) {
          d[v] = du + c;
          pq.push({d[v], v});
        }
      };
      if (reverse) g.forEachIn(u, relax);
      else g.forEachOut(u, relax);
    }
    counters().nodesExplored += settled;
    counters().statesExplored += settled;
  };
  // Farthest-point selection: start from the node farthest from vertex 0.
  full(0, false);
  NodeId cur = 0;
  double far = -1;
  for (int v = 0; v < n; ++v)
    if (std::isfinite(d[v]) && d[v] > far) { far = d[v]; cur = v; }
  for (int l = 0; l < T.L; ++l) {
    T.lm.push_back(cur);
    chosen[cur] = 1;
    full(cur, false);
    for (int v = 0; v < n; ++v) {
      T.fwd[static_cast<size_t>(v) * T.L + l] = static_cast<float>(d[v]);
      minD[v] = std::min(minD[v], d[v]);
    }
    full(cur, true);
    for (int v = 0; v < n; ++v) T.rev[static_cast<size_t>(v) * T.L + l] = static_cast<float>(d[v]);
    // next landmark: maximise distance to the chosen set (unreached components first)
    NodeId best = -1;
    double bestD = -1;
    for (int v = 0; v < n; ++v) {
      if (chosen[v]) continue;
      double s = std::isfinite(minD[v]) ? minD[v] : 1e300;
      if (s > bestD) { bestD = s; best = v; }
    }
    if (best < 0) {
      T.L = l + 1;
      break;
    }
    cur = best;
  }
  if (static_cast<int>(T.lm.size()) < T.L) T.L = static_cast<int>(T.lm.size());
  T.built = true;
  counters().altRebuilds++;
}

double AltIndex::lowerBound(int profile, NodeId v, NodeId t) const {
  const Table& T = t_[profile];
  if (!T.built || T.stale || v == t) return 0;
  if (static_cast<size_t>(v) >= T.n || static_cast<size_t>(t) >= T.n) return 0;
  const int L = T.L;
  // stride of stored rows is the L used at allocation time (T.fwd.size()/T.n)
  const size_t stride = T.n ? T.fwd.size() / T.n : 0;
  const float* fv = &T.fwd[static_cast<size_t>(v) * stride];
  const float* ft = &T.fwd[static_cast<size_t>(t) * stride];
  const float* rv = &T.rev[static_cast<size_t>(v) * stride];
  const float* rt = &T.rev[static_cast<size_t>(t) * stride];
  double best = 0;
  for (int l = 0; l < L; ++l) {
    double a = ft[l], b = fv[l];
    if (std::isfinite(a) && std::isfinite(b)) {
      // float storage error guard: |err| <= 2^-24 (a+b); subtract 4x that
      double h = (a - b) - (a + b) * 2.4e-7;
      if (h > best) best = h;
    }
    double c = rv[l], d = rt[l];
    if (std::isfinite(c) && std::isfinite(d)) {
      double h = (c - d) - (c + d) * 2.4e-7;
      if (h > best) best = h;
    }
  }
  best -= T.slack;
  return best > 0 ? best : 0;
}

AltIndex::Bound::Bound(const AltIndex& a, int profile, NodeId t) {
  const Table& T = a.t_[profile];
  if (!T.built || T.stale || T.n == 0 || static_cast<size_t>(t) >= T.n) return;
  stride_ = T.fwd.size() / T.n;
  n_ = T.n;
  L_ = T.L;
  t_ = t;
  slack_ = T.slack;
  fwd_ = T.fwd.data();
  rev_ = T.rev.data();
  ft_ = fwd_ + static_cast<size_t>(t) * stride_;
  rt_ = rev_ + static_cast<size_t>(t) * stride_;
  on_ = true;
}

double AltIndex::Bound::operator()(NodeId v) const {
  if (!on_ || v == t_ || static_cast<size_t>(v) >= n_) return 0;
  double best = 0;
  const float* fv = fwd_ + static_cast<size_t>(v) * stride_;
  const float* rv = rev_ + static_cast<size_t>(v) * stride_;
  for (int l = 0; l < L_; ++l) {
    double a = ft_[l], b = fv[l];
    if (std::isfinite(a) && std::isfinite(b)) {
      double h = (a - b) - (a + b) * 2.4e-7;
      if (h > best) best = h;
    }
    double c = rv[l], d = rt_[l];
    if (std::isfinite(c) && std::isfinite(d)) {
      double h = (c - d) - (c + d) * 2.4e-7;
      if (h > best) best = h;
    }
  }
  best -= slack_;
  return best > 0 ? best : 0;
}

void AltIndex::onEdgeChange(EdgeId e, const std::array<double, NUM_PROFILES>& oldCost,
                            const std::array<double, NUM_PROFILES>& newCost, const DynamicGraph* g) {
  for (int p = 0; p < NUM_PROFILES; ++p) {
    Table& T = t_[p];
    if (!T.built) continue;
    auto it = T.changed.find(e);
    double base = it == T.changed.end() ? oldCost[p] : it->second.first;
    double oldDef = it == T.changed.end() ? 0.0 : it->second.second;
    double newDef = 0;
    if (!std::isfinite(base)) {
      if (std::isfinite(newCost[p])) {  // edge unusable at build time became usable
        if (g && !T.stale) {
          // adopt the current cost as its build cost and repair the potentials around it
          T.changed[e] = {newCost[p], 0.0};
          repairEdge(T, *g, p, e, newCost[p]);
          continue;
        }
        T.stale = true;
      }
    } else if (std::isfinite(newCost[p])) {
      newDef = std::max(0.0, base - newCost[p]);
    }
    T.slack += newDef - oldDef;
    if (T.slack < 0) T.slack = 0;
    if (newDef > oldDef + 1e-12) T.decreases++;
    T.changed[e] = {base, newDef};
  }
}

void AltIndex::onEdgeAdded() {
  for (auto& T : t_)
    if (T.built) T.stale = true;
}

double AltIndex::buildCost(const Table& T, const DynamicGraph& g, EdgeId e, int profile) const {
  auto it = T.changed.find(e);
  return it != T.changed.end() ? it->second.first : edgeCost(g, e, profile);
}

void AltIndex::repairEdge(Table& T, const DynamicGraph& g, int profile, EdgeId e, double c) {
  if (!T.built || T.stale || T.n == 0 || !std::isfinite(c)) return;
  const size_t stride = T.fwd.size() / T.n;
  const size_t n = static_cast<size_t>(g.numNodes());
  if (n > T.n) {  // rows for new nodes: +inf is a feasible potential until repaired below
    T.fwd.resize(n * stride, std::numeric_limits<float>::infinity());
    T.rev.resize(n * stride, std::numeric_limits<float>::infinity());
    T.n = n;
  }
  const NodeId u = g.from(e), v = g.to(e);
  uint64_t settled = 0;
  std::vector<std::pair<double, int>> heap;
  auto cmp = std::greater<std::pair<double, int>>();
  for (int l = 0; l < T.L; ++l) {
    // forward: d(L, .) may drop for nodes reachable from v
    double fu = T.fwd[static_cast<size_t>(u) * stride + l];
    if (std::isfinite(fu) && static_cast<float>(fu + c) < T.fwd[static_cast<size_t>(v) * stride + l]) {
      heap.clear();
      T.fwd[static_cast<size_t>(v) * stride + l] = static_cast<float>(fu + c);
      heap.push_back({static_cast<float>(fu + c), v});
      while (!heap.empty()) {
        std::pop_heap(heap.begin(), heap.end(), cmp);
        auto [d, x] = heap.back();
        heap.pop_back();
        if (d > T.fwd[static_cast<size_t>(x) * stride + l]) continue;  // stale entry
        ++settled;
        g.forEachOut(x, [&](EdgeId f, NodeId y) {
          double cf = buildCost(T, g, f, profile);
          if (!std::isfinite(cf)) return;
          // compare in storage precision: every accepted update strictly lowers the stored
          // float, so the propagation terminates even where float rounding is coarse (|d| ~ 1e12)
          float& fy = T.fwd[static_cast<size_t>(y) * stride + l];
          const float nf = static_cast<float>(d + cf);
          if (nf < fy) {
            fy = nf;
            heap.push_back({nf, y});
            std::push_heap(heap.begin(), heap.end(), cmp);
          }
        });
      }
    }
    // reverse: d(., L) may drop for nodes that reach u
    double rv = T.rev[static_cast<size_t>(v) * stride + l];
    if (std::isfinite(rv) && static_cast<float>(c + rv) < T.rev[static_cast<size_t>(u) * stride + l]) {
      heap.clear();
      T.rev[static_cast<size_t>(u) * stride + l] = static_cast<float>(c + rv);
      heap.push_back({static_cast<float>(c + rv), u});
      while (!heap.empty()) {
        std::pop_heap(heap.begin(), heap.end(), cmp);
        auto [d, x] = heap.back();
        heap.pop_back();
        if (d > T.rev[static_cast<size_t>(x) * stride + l]) continue;  // stale entry
        ++settled;
        g.forEachIn(x, [&](EdgeId f, NodeId y) {
          double cf = buildCost(T, g, f, profile);
          if (!std::isfinite(cf)) return;
          float& ry = T.rev[static_cast<size_t>(y) * stride + l];
          const float nr = static_cast<float>(cf + d);
          if (nr < ry) {
            ry = nr;
            heap.push_back({nr, y});
            std::push_heap(heap.begin(), heap.end(), cmp);
          }
        });
      }
    }
  }
  counters().nodesExplored += settled;
  counters().statesExplored += settled;
}

void AltIndex::onEdgeAdded(const DynamicGraph& g, EdgeId e) {
  for (int p = 0; p < NUM_PROFILES; ++p) {
    Table& T = t_[p];
    if (!T.built || T.stale) continue;
    double c = edgeCost(g, e, p);
    if (!std::isfinite(c)) {
      // unusable now: remember build cost INF so a later availability change triggers repair
      T.changed[e] = {INF, 0.0};
      continue;
    }
    repairEdge(T, g, p, e, c);
  }
}

void AltIndex::dropAll() {
  for (auto& T : t_) T = Table();
}

size_t AltIndex::memoryBytes() const {
  size_t b = 0;
  for (const auto& T : t_) b += (T.fwd.size() + T.rev.size()) * sizeof(float) + T.changed.size() * 40;
  return b;
}

// ------------------------------------------------------------------ cache
void RouteCache::insert(uint64_t key, const Path& p) {
  if (max_ == 0) return;
  if (index_.size() >= max_) clear();  // simple full eviction keeps memory bounded
  int32_t i = index_.find(key);
  if (i < 0) {
    if (!free_.empty()) {
      i = free_.back();
      free_.pop_back();
    } else {
      i = static_cast<int32_t>(paths_.size());
      paths_.emplace_back();
    }
    index_.put(key, i);
  }
  paths_[static_cast<size_t>(i)] = p;
  for (EdgeId e : p.edges) byEdge_[e].push_back(key);
  edgeRefs_ += p.edges.size();
}

int RouteCache::invalidateEdge(EdgeId e) {
  auto it = byEdge_.find(e);
  if (it == byEdge_.end()) return 0;
  int removed = 0;
  for (uint64_t k : it->second) {
    int32_t i = index_.erase(k);
    if (i >= 0) {
      ++removed;
      Path& slot = paths_[static_cast<size_t>(i)];
      slot = Path();  // release the edge vectors now
      free_.push_back(i);
    }
  }
  edgeRefs_ -= std::min(edgeRefs_, it->second.size());
  byEdge_.erase(it);
  counters().cacheInvalidations += removed;
  return removed;
}

void RouteCache::clear() {
  index_.clear();
  paths_.clear();
  free_.clear();
  byEdge_.clear();
  edgeRefs_ = 0;
}

size_t RouteCache::memoryBytes() const {
  size_t b = index_.size() * (sizeof(Path) + 32) + byEdge_.size() * 48 + edgeRefs_ * (sizeof(uint64_t) + sizeof(EdgeId));
  return b;
}

// ------------------------------------------------------------------ router
Router::Router(const DynamicGraph& g) : g_(g) {}

int Router::capClass(double minCap) {
  if (!(minCap > 0)) return 0;
  int c = 1;
  while (c < 31 && capOfClass(c) < minCap) ++c;
  return c;
}

std::array<double, NUM_PROFILES> Router::edgeCosts(EdgeId e) const {
  std::array<double, NUM_PROFILES> c{};
  for (int p = 0; p < NUM_PROFILES; ++p) c[p] = edgeCost(g_, e, p);
  return c;
}

void Router::ensureAlt(int profile) {
  // first build is lazy; rebuilds after decreases/staleness happen in maintain() (cycle boundary)
  if (algo_ != RouteAlgo::ALT && !boundsOn_) return;
  if (!alt_.built(profile)) {
    ws_.ensure(g_.numNodes());
    alt_.build(g_, profile, ws_);
  }
}

const Path& Router::route(NodeId s, NodeId t, int profile, double minCap) {
  counters().routeQueries++;
  int cc = capClass(minCap);
  double effCap = capOfClass(cc);
  if (s == t) {
    scratch_ = Path();
    scratch_.found = true;
    scratch_.cost = 0;
    return scratch_;
  }
  uint64_t key = RouteCache::key(s, t, profile, cc);
  if (cacheOn_) {
    if (const Path* p = cache_.find(key)) {
      counters().cacheHits++;
      return *p;
    }
  }
  counters().cacheMisses++;
  if (algo_ == RouteAlgo::ALT) ensureAlt(profile);
  uint64_t before = counters().routingSettled;
  Path p = compute(s, t, profile, effCap);
  uint64_t actual = counters().routingSettled - before;
  ++uncached_;
  if (sampleEvery_ > 0 && counters().samples < maxSamples_ && uncached_ % static_cast<uint64_t>(sampleEvery_) == 0)
    sampleNaive(s, t, profile, effCap, actual);
  if (cacheOn_ && cache_.size() < 100000000) {
    cache_.insert(key, p);
    if (const Path* q = cache_.find(key)) return *q;
  }
  scratch_ = std::move(p);
  return scratch_;
}

void Router::sampleNaive(NodeId s, NodeId t, int profile, double minCap, uint64_t actualSettled) {
  // Measure what a plain Dijkstra would have explored on this exact query.
  Counters saved = counters();
  uint64_t before = counters().routingSettled;
  (void)dijkstra(s, t, profile, minCap, INF);
  uint64_t naive = counters().routingSettled - before;
  Counters& c = counters();
  c.nodesExplored = saved.nodesExplored;
  c.statesExplored = saved.statesExplored;
  c.routingSettled = saved.routingSettled;
  c.sampleNaive += static_cast<double>(naive);
  c.sampleActual += static_cast<double>(std::max<uint64_t>(1, actualSettled));
  c.samples++;
}

Path Router::compute(NodeId s, NodeId t, int profile, double minCap, double bound) {
  switch (algo_) {
    case RouteAlgo::Dijkstra: return dijkstra(s, t, profile, minCap, bound);
    case RouteAlgo::Bidirectional: return bidirectional(s, t, profile, minCap);
    default:
      // a stale table gives no guidance until the next rebuild: bidirectional search is exact and faster
      if (alt_.stale(profile) && !std::isfinite(bound)) return bidirectional(s, t, profile, minCap);
      return astar(s, t, profile, minCap, bound);
  }
}

Path Router::extract(NodeId s, NodeId t, const SearchWorkspace& ws, int profile) const {
  Path p;
  if (!std::isfinite(ws.dist(t))) return p;
  p.found = true;
  for (NodeId v = t; v != s;) {
    EdgeId e = ws.pred(v);
    if (e < 0) break;
    p.edges.push_back(e);
    v = g_.from(e);
  }
  std::reverse(p.edges.begin(), p.edges.end());
  fillPathMetrics(g_, p, profile);
  return p;
}

Path Router::dijkstra(NodeId s, NodeId t, int profile, double minCap, double bound) {
  ws_.ensure(g_.numNodes());
  ws_.reset();
  MinHeap pq;
  ws_.set(s, 0, -1);
  pq.push({0, s});
  uint64_t settled = 0;
  while (!pq.empty()) {
    double d = pq.top().first;
    int u = pq.top().second;
    pq.pop();
    if (d > ws_.dist(u) || ws_.closed(u)) continue;
    ws_.close(u);
    ++settled;
    if (u == t || d > bound) break;
    g_.forEachOut(u, [&](EdgeId e, NodeId v) {
      if (g_.cap(e) < minCap) return;
      double c = cost(e, profile);
      if (!std::isfinite(c)) return;
      double nd = d + c;
      if (nd < ws_.dist(v)) {
        ws_.set(v, nd, e);
        pq.push({nd, v});
      }
    });
  }
  Counters& C = counters();
  C.nodesExplored += settled;
  C.statesExplored += settled;
  C.routingSettled += settled;
  if (!ws_.closed(t) || ws_.dist(t) > bound) return Path();
  return extract(s, t, ws_, profile);
}

Path Router::bidirectional(NodeId s, NodeId t, int profile, double minCap) {
  int n = g_.numNodes();
  ws_.ensure(n);
  ws2_.ensure(n);
  ws_.reset();
  ws2_.reset();
  MinHeap qf, qb;
  ws_.set(s, 0, -1);
  ws2_.set(t, 0, -1);
  qf.push({0, s});
  qb.push({0, t});
  double mu = INF;
  EdgeId meet = -1;
  uint64_t settled = 0;
  while (!qf.empty() && !qb.empty()) {
    if (qf.top().first + qb.top().first >= mu) break;
    bool forward = qf.top().first <= qb.top().first;
    MinHeap& q = forward ? qf : qb;
    SearchWorkspace& a = forward ? ws_ : ws2_;
    SearchWorkspace& b = forward ? ws2_ : ws_;
    double d = q.top().first;
    int u = q.top().second;
    q.pop();
    if (d > a.dist(u) || a.closed(u)) continue;
    a.close(u);
    ++settled;
    auto relax = [&](EdgeId e, NodeId v) {
      if (g_.cap(e) < minCap) return;
      double c = cost(e, profile);
      if (!std::isfinite(c)) return;
      double nd = d + c;
      if (nd < a.dist(v)) {
        a.set(v, nd, e);
        q.push({nd, v});
      }
      double other = b.dist(v);
      if (std::isfinite(other) && nd + other < mu) {
        mu = nd + other;
        meet = e;
      }
    };
    if (forward) g_.forEachOut(u, relax);
    else g_.forEachIn(u, relax);
  }
  Counters& C = counters();
  C.nodesExplored += settled;
  C.statesExplored += settled;
  C.routingSettled += settled;
  Path p;
  if (s == t) {
    p.found = true;
    p.cost = 0;
    return p;
  }
  if (meet < 0 || !std::isfinite(mu)) return p;
  // forward part s -> from(meet), then meet edge, then backward part to(meet) -> t
  NodeId u = g_.from(meet), v = g_.to(meet);
  std::vector<EdgeId> front;
  for (NodeId x = u; x != s;) {
    EdgeId e = ws_.pred(x);
    if (e < 0) break;
    front.push_back(e);
    x = g_.from(e);
  }
  std::reverse(front.begin(), front.end());
  p.edges = front;
  p.edges.push_back(meet);
  for (NodeId x = v; x != t;) {
    EdgeId e = ws2_.pred(x);
    if (e < 0) break;
    p.edges.push_back(e);
    x = g_.to(e);
  }
  p.found = true;
  fillPathMetrics(g_, p, profile);
  return p;
}

Path Router::astar(NodeId s, NodeId t, int profile, double minCap, double bound) {
  const int n = g_.numNodes();
  ws_.ensure(n);
  ws_.reset();
  if (static_cast<int>(hStamp_.size()) < n) {
    hStamp_.resize(n, 0);
    hMemo_.resize(n, 0);
  }
  if (++fuelCur_ == 0) {  // stamp wrap-around: clear both stamp arrays
    std::fill(hStamp_.begin(), hStamp_.end(), 0);
    std::fill(hClosed_.begin(), hClosed_.end(), 0);
    std::fill(fuelStamp_.begin(), fuelStamp_.end(), 0);
    fuelCur_ = 1;
  }
  const uint32_t hs = fuelCur_;
  const AltIndex::Bound H(alt_, profile, t);
  // h(v) is a pure function of (v, t, tables) during one query: memoise it per node
  auto h = [&](NodeId v) {
    if (hStamp_[v] != hs) {
      hStamp_[v] = hs;
      hMemo_[v] = H(v);
    }
    return hMemo_[v];
  };
  struct Item {
    double f, g;
    int v;
    bool operator>(const Item& o) const { return f > o.f || (f == o.f && g < o.g); }
  };
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
  ws_.set(s, 0, -1);
  pq.push({h(s), 0, s});
  uint64_t settled = 0;
  while (!pq.empty()) {
    Item it = pq.top();
    pq.pop();
    if (it.g > ws_.dist(it.v) || ws_.closed(it.v)) continue;
    ws_.close(it.v);
    ++settled;
    if (it.v == t || it.f > bound) break;
    double d = it.g;
    g_.forEachOut(it.v, [&](EdgeId e, NodeId v) {
      if (g_.cap(e) < minCap) return;
      double c = cost(e, profile);
      if (!std::isfinite(c)) return;
      double nd = d + c;
      if (nd < ws_.dist(v)) {
        ws_.set(v, nd, e);  // also reopens v if it was closed (inconsistent bound after slack)
        double hv = h(v);
        if (nd + hv <= bound) pq.push({nd + hv, nd, v});
      }
    });
  }
  Counters& C = counters();
  C.nodesExplored += settled;
  C.statesExplored += settled;
  C.routingSettled += settled;
  if (!ws_.closed(t) || ws_.dist(t) > bound) return Path();
  return extract(s, t, ws_, profile);
}

double Router::lowerBound(NodeId s, NodeId t, int profile) {
  if (algo_ != RouteAlgo::ALT && !boundsOn_) return 0;
  ensureAlt(profile);
  return alt_.lowerBound(profile, s, t);
}

int Router::fullDijkstra(NodeId s, int profile, bool reverse, std::vector<double>& out) {
  int n = g_.numNodes();
  out.assign(n, INF);
  if (s < 0 || s >= n) return 0;
  MinHeap pq;
  out[s] = 0;
  pq.push({0, s});
  int settled = 0;
  while (!pq.empty()) {
    double d = pq.top().first;
    int u = pq.top().second;
    pq.pop();
    if (d > out[u]) continue;
    ++settled;
    auto relax = [&](EdgeId e, NodeId v) {
      double c = cost(e, profile);
      if (!std::isfinite(c)) return;
      if (d + c < out[v]) {
        out[v] = d + c;
        pq.push({out[v], v});
      }
    };
    if (reverse) g_.forEachIn(u, relax);
    else g_.forEachOut(u, relax);
  }
  counters().nodesExplored += settled;
  counters().statesExplored += settled;
  return settled;
}

// ---------------------------------------------------------- fuel-constrained labels
Path Router::routeFuel(NodeId s, NodeId t, int profile, double fuelCap, double fuelStart,
                       const std::vector<uint8_t>& refuel, double minCap, int buckets, double refuelPenalty) {
  using Label = FuelLabel;
  const int B = std::max(2, buckets);
  const int n = g_.numNodes();
  // Label budget per search. Every label counts against the scheduler's work cap, and on 100k-node
  // galaxies an uncapped search averaged ~190k labels: a handful of fuel legs used up a whole
  // cycle. A leg that needs more than 50k labels is reported infeasible for this ship, so the
  // mission goes to another candidate (extreme: 23k -> 63k on-time missions; small/medium/large
  // unchanged).
  const size_t maxLabels = 50000;
  // flat (node, bucket) table: fuelMin_[v*B+b] = min settled cost over buckets >= b of v, so the
  // dominance test (some settled label with fuel bucket >= ours and cost <= ours) is one lookup
  if (fuelB_ != B || fuelStamp_.size() < static_cast<size_t>(n)) {
    fuelB_ = B;
    fuelStamp_.assign(n, 0);
    fuelMin_.assign(static_cast<size_t>(n) * B, INF);
  }
  if (hStamp_.size() < static_cast<size_t>(n)) {
    hStamp_.resize(n, 0);
    hMemo_.resize(n, 0);
  }
  if (++fuelCur_ == 0) {
    std::fill(hStamp_.begin(), hStamp_.end(), 0);
    std::fill(hClosed_.begin(), hClosed_.end(), 0);
    std::fill(fuelStamp_.begin(), fuelStamp_.end(), 0);
    fuelCur_ = 1;
  }
  const uint32_t cur = fuelCur_;
  auto row = [&](int v) -> double* {
    double* r = &fuelMin_[static_cast<size_t>(v) * B];
    if (fuelStamp_[v] != cur) {
      fuelStamp_[v] = cur;
      std::fill(r, r + B, INF);
    }
    return r;
  };
  // Goal direction: ALT bound on the profile cost. Refuel steps add a non-negative cost at the
  // same node and the capacity/fuel filters only remove edges, so the bound stays admissible and
  // consistent whenever the table is exact for the current costs (no slack). Otherwise h = 0.
  if (algo_ == RouteAlgo::ALT || boundsOn_) ensureAlt(profile);
  // ALT only with a dense landmark set: with 4-8 landmarks (large/extreme galaxies) its bound is
  // far below the true distance and the label search wanders (extreme: ~86 % of fuel searches hit
  // the label limit). The bounded exact sweep below is never weaker and its cost is a ball around t.
  const bool useH = alt_.landmarkCount() >= 16 && alt_.built(profile) && !alt_.stale(profile) &&
                    alt_.slack(profile) <= 0;
  const AltIndex::Bound H = useH ? AltIndex::Bound(alt_, profile, t) : AltIndex::Bound();
  // Without a usable ALT table, one reverse Dijkstra from t on the current costs (capacity filter
  // applied, fuel ignored) gives the exact fuel-free distance: a consistent bound that is never
  // weaker than ALT. It costs O((V+E) log V) once, against label searches that otherwise grow to
  // ~B times the reachable graph. It also proves early that t is unreachable.
  // Bounded: the sweep stops at radius R = 2 d(s,t) + 4 refuel fees once s is settled. Nodes it
  // settled get their exact distance, all others the radius at which it stopped (their true
  // distance is at least that), i.e. h = min(d, R): still consistent, and the sweep covers a ball
  // around t instead of the whole graph (100k-node galaxies).
  double hCap = INF;   // value for nodes the sweep did not settle (INF only after a complete sweep)
  if (!H.active()) {
    if (hClosed_.size() < static_cast<size_t>(n)) hClosed_.resize(n, 0);
    MinHeap rq;
    hStamp_[t] = cur;
    hMemo_[t] = 0;
    rq.push({0, t});
    uint64_t rs = 0;
    double R = INF, radius = 0;
    bool complete = true;
    while (!rq.empty()) {
      double d = rq.top().first;
      int u = rq.top().second;
      if (d > R) {
        complete = false;
        radius = d;
        break;
      }
      rq.pop();
      if (d > hMemo_[u] || hClosed_[u] == cur) continue;
      hClosed_[u] = cur;
      radius = d;
      ++rs;
      if (u == s) R = 2.0 * d + 4.0 * refuelPenalty;
      g_.forEachIn(u, [&](EdgeId e, NodeId w) {
        if (g_.cap(e) < minCap || g_.fuel(e) > fuelCap + 1e-9) return;  // never traversable
        double c = cost(e, profile);
        if (!std::isfinite(c)) return;
        double nd = d + c;
        if (hStamp_[w] != cur || nd < hMemo_[w]) {
          hStamp_[w] = cur;
          hMemo_[w] = nd;
          rq.push({nd, w});
        }
      });
    }
    counters().nodesExplored += rs;
    counters().statesExplored += rs;
    counters().routingSettled += rs;
    if (complete && hClosed_[s] != cur) return Path();  // t unreachable from s even with unlimited fuel
    hCap = complete ? INF : radius;
  }
  auto h = [&](int v) {
    if (!H.active()) {
      if (hClosed_[v] == cur) return hMemo_[v];  // exact (settled)
      return hCap;  // not settled: true distance >= the sweep radius (INF: cannot reach t)
    }
    if (hStamp_[v] != cur) {
      hStamp_[v] = cur;
      hMemo_[v] = H(v);
    }
    return hMemo_[v];
  };
  auto bucketOf = [&](double f) {
    int b = static_cast<int>(f / std::max(1e-9, fuelCap) * B);
    return std::min(B - 1, std::max(0, b));
  };
  auto dominated = [&](int v, double cost, double fuel) {
    if (fuelStamp_[v] != cur) return false;
    return fuelMin_[static_cast<size_t>(v) * B + bucketOf(fuel)] <= cost;
  };
  auto settle = [&](int v, double cost, double fuel) {
    double* r = row(v);
    for (int b = bucketOf(fuel); b >= 0; --b) {
      if (r[b] <= cost) break;  // lower buckets already hold something at least as good
      r[b] = cost;
    }
  };
  std::vector<Label>& labels = fuelLabels_;
  labels.clear();
  using LItem = std::pair<double, int>;  // (cost + h, label)
  std::vector<LItem>& heap = fuelHeap_;
  heap.clear();
  auto push = [&](double f, int li) {
    heap.push_back({f, li});
    std::push_heap(heap.begin(), heap.end(), std::greater<LItem>());
  };
  fuelStart = std::min(fuelStart, fuelCap);
  labels.push_back({0, fuelStart, s, -1, -1});
  push(h(s), 0);
  int found = -1;
  uint64_t expanded = 0;
  while (!heap.empty() && labels.size() < maxLabels) {
    std::pop_heap(heap.begin(), heap.end(), std::greater<LItem>());
    int li = heap.back().second;
    heap.pop_back();
    Label L = labels[li];
    if (dominated(L.node, L.cost, L.fuel)) {
      counters().labelsPruned++;
      continue;
    }
    settle(L.node, L.cost, L.fuel);  // duplicate-state elimination
    ++expanded;
    if (L.node == t) {
      found = li;
      break;
    }
    if (L.node < static_cast<int>(refuel.size()) && refuel[L.node] && L.fuel < fuelCap - 1e-9) {
      labels.push_back({L.cost + refuelPenalty, fuelCap, L.node, li, -1});
      push(L.cost + refuelPenalty + h(L.node), static_cast<int>(labels.size()) - 1);
      counters().labelsCreated++;
    }
    g_.forEachOut(L.node, [&](EdgeId e, NodeId v) {
      if (g_.cap(e) < minCap || g_.fuel(e) > L.fuel + 1e-9) return;
      double ec = cost(e, profile);
      if (!std::isfinite(ec)) return;
      double nc = L.cost + ec, nf = L.fuel - g_.fuel(e);
      if (dominated(v, nc, nf)) {
        counters().labelsPruned++;
        return;
      }
      double hv = h(v);
      if (!std::isfinite(hv)) return;
      labels.push_back({nc, nf, v, li, e});
      push(nc + hv, static_cast<int>(labels.size()) - 1);
      counters().labelsCreated++;
    });
  }
  counters().nodesExplored += expanded;
  counters().statesExplored += labels.size();
  counters().routingSettled += expanded;
  counters().fuelSearches++;
  counters().fuelSearchesFound += found >= 0;
  counters().fuelSearchesCapped += labels.size() >= maxLabels;
  counters().fuelLabels += labels.size();
  Path p;
  if (found < 0) return p;
  p.found = true;
  p.fuelAtArrival = labels[found].fuel;
  for (int li = found; li >= 0 && labels[li].pred >= 0; li = labels[li].pred) {
    if (labels[li].edge >= 0) p.edges.push_back(labels[li].edge);
    else p.refuelNodes.push_back(labels[li].node);
  }
  std::reverse(p.edges.begin(), p.edges.end());
  std::reverse(p.refuelNodes.begin(), p.refuelNodes.end());
  fillPathMetrics(g_, p, profile);
  p.cost += refuelPenalty * static_cast<double>(p.refuelNodes.size());
  return p;
}

// ---------------------------------------------------------- Pareto labels
std::vector<Path> Router::paretoPaths(NodeId s, NodeId t, double minCap, int maxLabelsPerNode, double eps,
                                      int maxTotalLabels) {
  struct Label {
    double time, risk;
    int node, pred;
    EdgeId edge;
  };
  const int n = g_.numNodes();
  std::vector<Label> labels;
  labels.reserve(1024);
  // node -> permanent labels, flat and reused across calls (only touched nodes are cleared)
  if (static_cast<int>(paretoPerm_.size()) < n) paretoPerm_.resize(n);
  for (int v : paretoTouched_) paretoPerm_[v].clear();
  paretoTouched_.clear();
  auto& perm = paretoPerm_;
  std::vector<int> targetLabels;
  // Completion-time bound for pruning. Preferred: the ALT bound on mean travel time (cheap per
  // node, memoised). Without a usable table: one reverse Dijkstra from t on the label time metric
  // (mean + 2 sigma), which is exact and also marks nodes that cannot reach t at all.
  if (static_cast<int>(hStamp_.size()) < n) {
    hStamp_.resize(n, 0);
    hMemo_.resize(n, 0);
  }
  if (++fuelCur_ == 0) {
    std::fill(hStamp_.begin(), hStamp_.end(), 0);
    std::fill(hClosed_.begin(), hClosed_.end(), 0);
    std::fill(fuelStamp_.begin(), fuelStamp_.end(), 0);
    fuelCur_ = 1;
  }
  const uint32_t cur = fuelCur_;
  if (algo_ == RouteAlgo::ALT || boundsOn_) ensureAlt(PROFILE_TIME);
  const bool useAlt = alt_.built(PROFILE_TIME) && !alt_.stale(PROFILE_TIME);
  const AltIndex::Bound H = useAlt ? AltIndex::Bound(alt_, PROFILE_TIME, t) : AltIndex::Bound();
  if (!useAlt) {
    MinHeap rq;
    hStamp_[t] = cur;
    hMemo_[t] = 0;
    rq.push({0, t});
    uint64_t rs = 0;
    while (!rq.empty()) {
      double d = rq.top().first;
      int u = rq.top().second;
      rq.pop();
      if (d > hMemo_[u]) continue;
      ++rs;
      g_.forEachIn(u, [&](EdgeId e, NodeId w) {
        if (!g_.available(e) || g_.cap(e) < minCap) return;
        double nd = d + g_.tmean(e) + 2.0 * g_.sigma(e);
        if (hStamp_[w] != cur || nd < hMemo_[w]) {
          hStamp_[w] = cur;
          hMemo_[w] = nd;
          rq.push({nd, w});
        }
      });
    }
    counters().nodesExplored += rs;
    counters().statesExplored += rs;
    counters().routingSettled += rs;
    if (hStamp_[s] != cur) return {};
  }
  auto lbOf = [&](int v) {
    if (hStamp_[v] != cur) {
      if (!useAlt) return INF;  // not reverse-reachable
      hStamp_[v] = cur;
      hMemo_[v] = H(v);
    }
    return hMemo_[v];
  };
  using LItem = std::pair<double, int>;
  std::priority_queue<LItem, std::vector<LItem>, std::greater<LItem>> pq;
  auto epsDom = [&](const Label& a, const Label& b) {
    return a.time <= (1 + eps) * b.time + 1e-12 && a.risk <= (1 + eps) * b.risk + 1e-12;
  };
  labels.push_back({0, 0, s, -1, -1});
  pq.push({0, 0});
  uint64_t expanded = 0;
  while (!pq.empty() && static_cast<int>(labels.size()) < maxTotalLabels) {
    int li = pq.top().second;
    pq.pop();
    const Label L = labels[li];
    bool dom = false;
    for (int o : targetLabels)
      if (epsDom(labels[o], L)) { dom = true; break; }
    auto& lst = perm[L.node];
    if (!dom)
      for (int o : lst)
        if (epsDom(labels[o], L)) { dom = true; break; }
    if (dom) {
      counters().labelsPruned++;
      continue;
    }
    if (static_cast<int>(lst.size()) >= maxLabelsPerNode) {  // bounded label sets
      counters().labelsPruned++;
      continue;
    }
    if (lst.empty()) paretoTouched_.push_back(L.node);
    lst.push_back(li);
    ++expanded;
    if (L.node == t) {
      targetLabels.push_back(li);
      continue;
    }
    g_.forEachOut(L.node, [&](EdgeId e, NodeId v) {
      if (!g_.available(e) || g_.cap(e) < minCap) return;
      double lb = lbOf(v);
      if (!std::isfinite(lb)) return;  // v cannot reach t
      double rel = g_.rel(e);
      Label nl{L.time + g_.tmean(e) + 2.0 * g_.sigma(e), L.risk + g_.risk(e) + (rel >= 1 ? 0 : -std::log(rel)),
               v, li, e};
      // completion bound: time >= time + LB_time(v,t), risk >= current risk
      for (int o : targetLabels)
        if (labels[o].time <= (1 + eps) * (nl.time + lb) + 1e-12 && labels[o].risk <= (1 + eps) * nl.risk + 1e-12) {
          counters().labelsPruned++;
          return;
        }
      for (int o : perm[v])
        if (epsDom(labels[o], nl)) { counters().labelsPruned++; return; }
      labels.push_back(nl);
      pq.push({nl.time, static_cast<int>(labels.size()) - 1});
      counters().labelsCreated++;
    });
  }
  counters().nodesExplored += expanded;
  counters().statesExplored += labels.size();
  counters().routingSettled += expanded;
  counters().paretoSearches++;
  counters().paretoLabels += labels.size();
  std::vector<Path> out;
  for (int li : targetLabels) {
    Path p;
    p.found = true;
    for (int x = li; x >= 0 && labels[x].pred >= 0; x = labels[x].pred) p.edges.push_back(labels[x].edge);
    std::reverse(p.edges.begin(), p.edges.end());
    fillPathMetrics(g_, p, PROFILE_EMERGENCY);
    out.push_back(std::move(p));
  }
  return out;
}

void Router::nearestSources(NodeId t, int profile, int k, int settleLimit, const std::function<bool(NodeId)>& isSource,
                            std::vector<std::pair<NodeId, double>>& out, std::vector<std::vector<EdgeId>>* paths,
                            double minCap) {
  out.clear();
  if (paths) paths->clear();
  ws2_.ensure(g_.numNodes());
  ws2_.reset();
  MinHeap pq;
  ws2_.set(t, 0, -1);
  pq.push({0, t});
  int settled = 0;
  while (!pq.empty() && static_cast<int>(out.size()) < k && settled < settleLimit) {
    double d = pq.top().first;
    int u = pq.top().second;
    pq.pop();
    if (d > ws2_.dist(u) || ws2_.closed(u)) continue;
    ws2_.close(u);
    ++settled;
    if (u != t && isSource(u)) {
      out.emplace_back(u, d);
      if (paths) {  // follow the reverse-search tree: pred(x) is the edge x -> next towards t
        std::vector<EdgeId> p;
        for (NodeId x = u; x != t;) {
          EdgeId e = ws2_.pred(x);
          if (e < 0) break;
          p.push_back(e);
          x = g_.to(e);
        }
        paths->push_back(std::move(p));
      }
    }
    g_.forEachIn(u, [&](EdgeId e, NodeId v) {
      if (g_.cap(e) < minCap) return;
      double c = cost(e, profile);
      if (!std::isfinite(c)) return;
      double nd = d + c;
      if (nd < ws2_.dist(v)) {
        ws2_.set(v, nd, e);
        pq.push({nd, v});
      }
    });
  }
  counters().nodesExplored += settled;
  counters().statesExplored += settled;
}

int Router::onEdgeChanged(EdgeId e, const std::array<double, NUM_PROFILES>& oldCost) {
  auto nc = edgeCosts(e);
  for (int p = 0; p < NUM_PROFILES; ++p)
    if (e < static_cast<EdgeId>(costs_[p].size())) costs_[p][e] = nc[p];
  alt_.onEdgeChange(e, oldCost, nc, &g_);
  bool increased = false, decreased = false;
  for (int p = 0; p < NUM_PROFILES; ++p) {
    if (nc[p] > oldCost[p] + 1e-12) increased = true;
    if (nc[p] < oldCost[p] - 1e-12) decreased = true;
  }
  int dropped = 0;
  if (increased) dropped += cache_.invalidateEdge(e);  // targeted invalidation
  if (decreased) dropped += invalidateForCheaperEdge(e);  // cached paths it may improve
  return dropped;
}

int Router::invalidateForCheaperEdge(EdgeId e) {
  if (cache_.size() == 0) return 0;
  const NodeId u = g_.from(e), v = g_.to(e);
  std::array<bool, NUM_PROFILES> usable{};
  std::array<double, NUM_PROFILES> ce{};
  bool any = false;
  for (int p = 0; p < NUM_PROFILES; ++p) {
    ce[p] = edgeCost(g_, e, p);
    usable[p] = alt_.built(p) && !alt_.stale(p);
    any = any || usable[p];
  }
  int dropped;
  if (!any) {
    dropped = static_cast<int>(cache_.size());
    cache_.clear();
  } else {
    dropped = cache_.eraseIf([&](uint64_t k, const Path& P) {
      int p = RouteCache::keyProfile(k);
      if (p >= NUM_PROFILES || !usable[p]) return true;
      if (!std::isfinite(ce[p])) return false;  // e unusable for this profile: cannot help
      if (!P.found) return true;                // "no path" may now be wrong
      NodeId s = RouteCache::keySource(k), t = RouteCache::keyTarget(k);
      double lb = alt_.lowerBound(p, s, u) + ce[p] + alt_.lowerBound(p, v, t);
      return lb < P.cost - 1e-9 * std::max(1.0, P.cost);
    });
  }
  counters().cacheInvalidations += dropped;
  return dropped;
}

void Router::onEdgeAdded(EdgeId e) {
  alt_.onEdgeAdded(g_, e);  // in-place landmark repair (no full rebuild needed)
  invalidateForCheaperEdge(e);
}

void Router::onNodeAdded() {
  ws_.ensure(g_.numNodes());
  ws2_.ensure(g_.numNodes());
}

void Router::maintain() {
  if ((algo_ != RouteAlgo::ALT && !boundsOn_) || !rebuildAllowed_) return;
  for (int p = 0; p < NUM_PROFILES; ++p)
    if (alt_.needsRebuild(p)) {
      ws_.ensure(g_.numNodes());
      alt_.build(g_, p, ws_);
    }
}

size_t Router::memoryBytes() const {
  return alt_.memoryBytes() + cache_.memoryBytes() + ws_.memoryBytes() + ws2_.memoryBytes();
}

}  // namespace chronos
