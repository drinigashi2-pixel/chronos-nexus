// flow.cpp - primal-dual min-cost flow and the reduced-network resource allocator.
// MinCostFlow::solve: O(phases * ((V+E) log V + blocking flow)); restorePotentials: SPFA on reduced costs.
// Allocation per resource: D reverse bounded searches + one warm-startable MCF solve.
#include "chronos/flow.hpp"

#include <queue>

namespace chronos {

namespace {
constexpr double COST_SCALE = 100.0;  // fixed-point scaling of per-unit costs
constexpr MinCostFlow::Cost CINF = std::numeric_limits<MinCostFlow::Cost>::max() / 4;
MinCostFlow::Cap toCap(double v) {
  if (!(v > 0)) return 0;
  if (v > 1e15) return static_cast<MinCostFlow::Cap>(1e15);
  return static_cast<MinCostFlow::Cap>(std::floor(v + 1e-9));
}
MinCostFlow::Cost toCost(double v) {
  if (!std::isfinite(v)) return CINF / 1024;
  return static_cast<MinCostFlow::Cost>(std::llround(v * COST_SCALE));
}
}  // namespace

// ================================================================ MinCostFlow
void MinCostFlow::reset(int n) {
  head_.assign(std::max(0, n), -1);
  next_.clear(); to_.clear(); cap_.clear(); cap0_.clear(); cost_.clear();
  pot_.assign(std::max(0, n), 0);
  dirty_ = false;
  settled_ = 0;
}

int MinCostFlow::addNode() {
  head_.push_back(-1);
  pot_.push_back(0);
  return static_cast<int>(head_.size()) - 1;
}

int MinCostFlow::addArc(int u, int v, Cap cap, Cost cost) {
  int id = static_cast<int>(to_.size());
  to_.push_back(v); next_.push_back(head_[u]); head_[u] = id;
  cap_.push_back(cap); cap0_.push_back(cap); cost_.push_back(cost);
  to_.push_back(u); next_.push_back(head_[v]); head_[v] = id + 1;
  cap_.push_back(0); cap0_.push_back(0); cost_.push_back(-cost);
  if (cap > 0 && cost + pot_[u] - pot_[v] < 0) dirty_ = true;
  return id;
}

bool MinCostFlow::dijkstra(int s, int t) {
  int n = numNodes();
  dist_.assign(n, CINF);
  using Item = std::pair<Cost, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
  dist_[s] = 0;
  pq.push({0, s});
  std::vector<char> done(n, 0);
  while (!pq.empty()) {
    Cost d = pq.top().first;
    int u = pq.top().second;
    pq.pop();
    if (done[u] || d > dist_[u]) continue;
    done[u] = 1;
    ++settled_;
    if (u == t) break;
    for (int a = head_[u]; a != -1; a = next_[a]) {
      if (cap_[a] <= 0) continue;
      int v = to_[a];
      Cost rc = cost_[a] + pot_[u] - pot_[v];  // >= 0 by invariant
      if (rc < 0) rc = 0;                       // defensive (never triggers with valid potentials)
      if (d + rc < dist_[v]) {
        dist_[v] = d + rc;
        pq.push({dist_[v], v});
      }
    }
  }
  if (dist_[t] >= CINF) return false;
  Cost dt = dist_[t];
  for (int v = 0; v < n; ++v) pot_[v] += std::min(dist_[v], dt);
  return true;
}

MinCostFlow::Cap MinCostFlow::dfs(int v, int t, Cap f) {
  if (v == t) return f;
  for (int& a = it_[v]; a != -1; a = next_[a]) {
    int w = to_[a];
    if (cap_[a] <= 0 || level_[w] != level_[v] + 1) continue;
    if (cost_[a] + pot_[v] - pot_[w] != 0) continue;  // admissible = zero reduced cost
    Cap d = dfs(w, t, std::min(f, cap_[a]));
    if (d > 0) {
      cap_[a] -= d;
      cap_[a ^ 1] += d;
      return d;
    }
  }
  return 0;
}

MinCostFlow::Cap MinCostFlow::blockingFlow(int s, int t, Cap limit) {
  Cap total = 0;
  int n = numNodes();
  while (total < limit) {
    level_.assign(n, -1);
    std::vector<int> q;
    q.reserve(n);
    level_[s] = 0;
    q.push_back(s);
    for (size_t qi = 0; qi < q.size(); ++qi) {
      int u = q[qi];
      for (int a = head_[u]; a != -1; a = next_[a]) {
        int v = to_[a];
        if (cap_[a] > 0 && level_[v] < 0 && cost_[a] + pot_[u] - pot_[v] == 0) {
          level_[v] = level_[u] + 1;
          q.push_back(v);
        }
      }
    }
    if (level_[t] < 0) break;
    it_ = head_;
    Cap f;
    while (total < limit && (f = dfs(s, t, limit - total)) > 0) total += f;
  }
  return total;
}

MinCostFlow::Result MinCostFlow::solve(int s, int t, Cap maxFlow, const Deadline& dl) {
  Result r;
  if (s == t || s < 0 || t < 0) return r;
  if (dirty_) {
    if (restorePotentials(&dl) < 0) {  // deadline hit while repairing potentials
      r.timedOut = true;
      return r;
    }
    dirty_ = false;
  }
  while (r.flow < maxFlow) {
    if (dl.expired()) {
      r.timedOut = true;
      break;
    }
    if (!dijkstra(s, t)) break;
    r.phases++;
    Cap f = blockingFlow(s, t, maxFlow - r.flow);
    if (f <= 0) break;
    r.flow += f;
    r.cost += f * (pot_[t] - pot_[s]);
  }
  return r;
}

MinCostFlow::Cap MinCostFlow::pushAlong(const std::vector<int>& arcs, Cap amount) {
  Cap b = amount;
  for (int a : arcs) b = std::min(b, cap_[a]);
  if (b <= 0) return 0;
  for (int a : arcs) {
    cap_[a] -= b;
    cap_[a ^ 1] += b;
  }
  dirty_ = true;
  return b;
}

MinCostFlow::Cap MinCostFlow::cancelThrough(int arc, Cap amount, int s, int t) {
  int n = numNodes();
  Cap done = 0;
  std::vector<int> seen(n, 0), predArc(n, -1);
  int stampv = 0;
  // DFS from `start` following arcs selected by `ok`; returns arc list from start to goal.
  auto search = [&](int start, int goal, bool backward, std::vector<int>& out) -> bool {
    out.clear();
    if (start == goal) return true;
    ++stampv;
    std::vector<int> stack{start};
    seen[start] = stampv;
    predArc[start] = -1;
    while (!stack.empty()) {
      int u = stack.back();
      stack.pop_back();
      for (int a = head_[u]; a != -1; a = next_[a]) {
        // backward: odd residual arcs u -> w with cap > 0 correspond to forward flow w -> u
        bool okArc = backward ? ((a & 1) && cap_[a] > 0) : (!(a & 1) && flow(a) > 0);
        if (!okArc) continue;
        int w = to_[a];
        if (seen[w] == stampv) continue;
        seen[w] = stampv;
        predArc[w] = a;
        if (w == goal) {
          for (int x = goal; x != start;) {
            int pa = predArc[x];
            out.push_back(backward ? (pa ^ 1) : pa);  // store forward arcs
            x = to_[pa ^ 1];
          }
          if (!backward) std::reverse(out.begin(), out.end());
          return true;
        }
        stack.push_back(w);
      }
    }
    return false;
  };
  std::vector<int> in, out;
  while (done < amount && flow(arc) > 0) {
    if (!search(from(arc), s, true, in) || !search(to(arc), t, false, out)) break;
    Cap b = std::min(amount - done, flow(arc));
    for (int a : in) b = std::min(b, flow(a));
    for (int a : out) b = std::min(b, flow(a));
    if (b <= 0) break;
    auto unpush = [&](int a) {
      cap_[a] += b;
      cap_[a ^ 1] -= b;
    };
    for (int a : in) unpush(a);
    unpush(arc);
    for (int a : out) unpush(a);
    done += b;
  }
  dirty_ = true;
  return done;
}

void MinCostFlow::setCapacity(int arc, Cap cap, int s, int t) {
  cap = std::max<Cap>(0, cap);
  Cap f = flow(arc);
  if (cap < f) cancelThrough(arc, f - cap, s, t);
  f = flow(arc);
  cap0_[arc] = std::max(cap, f);
  cap_[arc] = cap0_[arc] - f;
  dirty_ = true;
}

int MinCostFlow::restorePotentials(const Deadline* dl) {
  // SPFA on REDUCED costs from a virtual source (all distances 0): with the old potentials most
  // arcs are still non-negative, so only the few violated arcs propagate (near-linear in practice).
  // Any cycle in the predecessor graph is negative: cancel it and restart. On success
  // pot += dist gives feasible potentials. Returns #cycles cancelled, or -1 if the deadline hit.
  int n = numNodes();
  int cancelled = 0;
  std::vector<Cost> d(n);
  std::vector<int> pred(n), color(n);
  std::vector<char> inq(n);
  std::vector<int> q;
  q.reserve(n);
  auto findCycle = [&]() -> int {
    std::fill(color.begin(), color.end(), 0);
    for (int v = 0; v < n; ++v) {
      if (color[v]) continue;
      int x = v;
      while (x >= 0 && color[x] == 0) {
        color[x] = v + 1;
        x = pred[x] >= 0 ? to_[pred[x] ^ 1] : -1;
      }
      int hit = (x >= 0 && color[x] == v + 1) ? x : -1;
      for (int y = v; y >= 0 && color[y] == v + 1;) {
        color[y] = -1;
        y = pred[y] >= 0 ? to_[pred[y] ^ 1] : -1;
      }
      if (hit >= 0) return hit;
    }
    return -1;
  };
  for (int guard = 0; guard < 1000000; ++guard) {
    std::fill(d.begin(), d.end(), 0);
    std::fill(pred.begin(), pred.end(), -1);
    q.clear();
    for (int v = 0; v < n; ++v) {
      q.push_back(v);
      inq[v] = 1;
    }
    size_t head = 0;
    uint64_t pops = 0;
    int cyc = -1;
    while (head < q.size()) {
      int u = q[head++];
      inq[u] = 0;
      if (head > static_cast<size_t>(4 * n) + 1024) {  // compact the queue
        q.erase(q.begin(), q.begin() + static_cast<long>(head));
        head = 0;
      }
      ++pops;
      for (int a = head_[u]; a != -1; a = next_[a]) {
        if (cap_[a] <= 0) continue;
        int v = to_[a];
        Cost nd = d[u] + cost_[a] + pot_[u] - pot_[v];
        if (nd < d[v]) {
          d[v] = nd;
          pred[v] = a;
          if (!inq[v]) {
            inq[v] = 1;
            q.push_back(v);
          }
        }
      }
      if (pops % static_cast<uint64_t>(std::max(64, n)) == 0) {
        if (dl && dl->expired()) return -1;
        cyc = findCycle();
        if (cyc >= 0) break;
      }
    }
    if (cyc < 0 && head < q.size()) cyc = findCycle();
    if (cyc < 0) {
      for (int v = 0; v < n; ++v) pot_[v] += d[v];
      return cancelled;
    }
    std::vector<int> arcs;
    int x = cyc;
    do {
      arcs.push_back(pred[x]);
      x = to_[pred[x] ^ 1];
    } while (x != cyc && arcs.size() <= static_cast<size_t>(n));
    Cap b = std::numeric_limits<Cap>::max();
    for (int a : arcs) b = std::min(b, cap_[a]);
    for (int a : arcs) {
      cap_[a] -= b;
      cap_[a ^ 1] += b;
    }
    ++cancelled;
  }
  return cancelled;
}

MinCostFlow::Cost MinCostFlow::totalCost() const {
  Cost c = 0;
  for (size_t a = 0; a < to_.size(); a += 2) c += flow(static_cast<int>(a)) * cost_[a];
  return c;
}

bool MinCostFlow::checkConservation(int s, int t) const {
  int n = numNodes();
  std::vector<Cap> bal(n, 0);
  for (size_t a = 0; a < to_.size(); a += 2) {
    Cap f = flow(static_cast<int>(a));
    if (f < 0 || f > cap0_[a]) return false;
    bal[to_[a]] += f;
    bal[to_[a ^ 1]] -= f;
  }
  for (int v = 0; v < n; ++v)
    if (v != s && v != t && bal[v] != 0) return false;
  return bal[s] == -bal[t];
}

// ================================================================ allocator
ResourceAllocator::ResourceAllocator(World& w, Router& r) : w_(w), router_(r) {}

namespace {
template <class Info>
Info makeInfo(int kind, NodeId i, NodeId j, double unit = 0) {
  Info a;
  a.kind = kind;
  a.i = i;
  a.j = j;
  a.unitCost = unit;
  return a;
}
}  // namespace

double ResourceAllocator::unitPenalty(NodeId j, int r) const {
  const Planet& p = w_.planets[j];
  double popW = 1.0 + std::log10(1.0 + p.population) / 3.0;
  return cfg_.penaltyScale * RESOURCE_IMPORTANCE[r] * p.strategic * popW;
}

void ResourceAllocator::balance(NodeId p, int r, double incoming, double outgoing, double& supply, double& demand,
                                double& overflow) const {
  supply = demand = overflow = 0;
  const Planet& P = w_.planets[p];
  if (!P.alive) return;
  double H = cfg_.horizonDays;
  double cons = P.consumption[r], net = P.netRate(r);
  double stock = P.stock[r] + incoming - outgoing;
  double reserve = cons * cfg_.reserveDays;
  double extra = w_.extraDemand[static_cast<size_t>(p) * NUM_RESOURCES + r];
  double projected = stock + net * H;
  double need = reserve + extra - projected;
  double freeCap = std::max(0.0, P.capacity[r] - P.stock[r] - incoming);
  if (need > 0) {
    demand = std::min(freeCap, need + std::max(0.0, -net) * std::max(0.0, cfg_.coverDays - H));
    return;
  }
  if (w_.supplyFrozen(p, r)) return;
  supply = std::max(0.0, std::min(stock - reserve - extra, projected - reserve - extra));
  overflow = std::min(supply, std::max(0.0, projected - P.capacity[r]));
}

double ResourceAllocator::deadlineFor(NodeId j, int r, double now, double incomingUnits, double leadH) const {
  const Planet& P = w_.planets[j];
  double loss = -P.netRate(r);
  double reserve = P.consumption[r] * cfg_.reserveDays * 0.5;
  double stock = P.stock[r] + incomingUnits;
  double tts = loss > 1e-9 ? std::max(0.0, (stock - reserve) / loss) * HOURS_PER_DAY : 7 * HOURS_PER_DAY;
  return std::max(now + tts - leadH, now + 1.2 * leadH + 6.0);
}

int ResourceAllocator::newArc(Net& N, int u, int v, double cap, double cost, const ArcInfo& info) {
  int a = N.mcf.addArc(u, v, toCap(cap), toCost(cost));
  if (static_cast<int>(N.info.size()) <= a / 2) N.info.resize(a / 2 + 1);
  N.info[a / 2] = info;
  counters().candidatesEvaluated++;
  return a;
}

void ResourceAllocator::addSupplyNode(int r, NodeId i, double supply, double overflow) {
  Net& N = net_[r];
  int node = N.mcf.addNode();
  N.nodeOf[i] = node;
  ArcInfo inf = makeInfo<ArcInfo>(0, i, i);
  N.supplyArc[i] = newArc(N, N.F, node, overflow, 0.0, inf);           // overflow first (would be wasted)
  N.supplyArc2[i] = newArc(N, N.F, node, supply - overflow, 0.01, inf);  // regular surplus
}

void ResourceAllocator::precomputeSources(const Deadline& dl) {
  cands_.clear();
  int P = w_.numPlanets();
  std::vector<std::pair<double, NodeId>> order;  // most important deficits first (anytime)
  for (int j = 0; j < P; ++j) {
    double best = 0;
    for (int r = 0; r < NUM_RESOURCES; ++r)
      if (demand_[r][j] >= cfg_.minShipment) best = std::max(best, unitPenalty(j, r) * demand_[r][j]);
    if (best > 0) order.push_back({-best, j});
  }
  std::sort(order.begin(), order.end());
  std::vector<std::pair<NodeId, double>> src;
  std::vector<std::vector<EdgeId>> paths;
  std::array<int, NUM_RESOURCES> cnt{};
  for (size_t k = 0; k < order.size(); ++k) {
    if ((k & 63) == 0 && dl.expired()) break;  // remaining deficits get only shortage arcs this cycle
    NodeId j = order[k].second;
    int needed = 0;
    for (int r = 0; r < NUM_RESOURCES; ++r) {
      cnt[r] = demand_[r][j] >= cfg_.minShipment ? 0 : cfg_.K;  // resources j does not need are "full"
      needed += cnt[r] == 0 ? 1 : 0;
    }
    router_.nearestSources(
        j, PROFILE_CARGO, cfg_.K * needed, cfg_.settleLimit,
        [&](NodeId v) {
          bool useful = false;
          for (int r = 0; r < NUM_RESOURCES; ++r)
            if (cnt[r] < cfg_.K && supply_[r][v] >= cfg_.minShipment) {
              ++cnt[r];
              useful = true;
            }
          return useful;
        },
        src, &paths);
    auto& vec = cands_[j];
    for (size_t i = 0; i < src.size(); ++i) vec.push_back({src[i].first, src[i].second, std::move(paths[i])});
  }
}

void ResourceAllocator::addDeficitNode(int r, NodeId j, double demand) {
  Net& N = net_[r];
  int node = N.mcf.addNode();
  N.nodeOf[j] = node;
  double pen = unitPenalty(j, r);
  N.penalty[j] = pen;
  N.demandArc[j] = newArc(N, node, N.T, demand, 0.0, makeInfo<ArcInfo>(3, j, j));
  N.shortArc[j] = newArc(N, N.S, node, demand, pen, makeInfo<ArcInfo>(2, j, j, pen));
  std::vector<std::pair<NodeId, double>> src;
  std::vector<std::vector<EdgeId>> paths;
  const auto& sup = supply_[r];
  auto isSrc = [&](NodeId v) {
    return v < static_cast<NodeId>(sup.size()) && N.nodeOf[v] >= 0 && N.supplyArc[v] >= 0;
  };
  auto pre = cands_.find(j);
  if (pre != cands_.end()) {  // shared multi-resource search (allocate)
    for (const auto& c : pre->second)
      if (static_cast<int>(src.size()) < cfg_.K && isSrc(c.v)) {
        src.push_back({c.v, c.cost});
        paths.push_back(c.path);
      }
  } else if (!inAllocate_) {  // planets that became deficits after the cycle plan (repair)
    router_.nearestSources(j, PROFILE_CARGO, cfg_.K, cfg_.settleLimit, isSrc, src, &paths);
  }  // else: source discovery ran out of time for j -> only its shortage arc this cycle
  for (size_t k = 0; k < src.size(); ++k) {
    NodeId i = src[k].first;
    double unit = src[k].second / std::max(1.0, cfg_.refShipCap);
    if (unit >= pen) continue;  // never cheaper than the shortage
    Path p;
    p.edges = paths[k];
    fillPathMetrics(w_.graph, p, PROFILE_CARGO);
    double bottleneck = INF;
    for (EdgeId e : p.edges) {
      auto it = edgeUsed_.find(e);
      bottleneck = std::min(bottleneck, w_.graph.cap(e) - (it == edgeUsed_.end() ? 0.0 : it->second));
    }
    double cap = std::min({sup[i], demand, fleetLeft_, std::max(0.0, bottleneck)});
    if (cap < cfg_.minShipment) continue;
    ArcInfo inf = makeInfo<ArcInfo>(1, i, j, unit);
    inf.path = p.edges;
    inf.pathTime = p.timeMean;
    inf.pathSigma = std::sqrt(p.timeVar);
    int a = newArc(N, N.nodeOf[i], node, cap, unit, inf);
    N.transport.push_back(a);
    for (EdgeId e : inf.path) arcByEdge_[e].push_back({r, a});
  }
}

int ResourceAllocator::boundCapacities(int r) {
  // Arcs are created with their route bottleneck; this only fixes arcs of the same resource that
  // jointly over-use a shared edge, or arcs whose route vanished.
  Net& N = net_[r];
  int tightened = 0;
  std::unordered_map<EdgeId, double> load;
  for (int a : N.transport) {
    MinCostFlow::Cap f = N.mcf.flow(a);
    if (f <= 0) continue;
    for (EdgeId e : N.info[a / 2].path) load[e] += static_cast<double>(f);
  }
  for (int a : N.transport) {
    MinCostFlow::Cap f = N.mcf.flow(a);
    if (f <= 0) continue;
    const ArcInfo& I = N.info[a / 2];
    double scale = 1.0;
    bool dead = I.path.empty() && I.i != I.j;
    for (EdgeId e : I.path) {
      if (!w_.graph.available(e)) dead = true;
      auto it = edgeUsed_.find(e);
      double room = w_.graph.cap(e) - (it == edgeUsed_.end() ? 0.0 : it->second);
      if (load[e] > room + 1e-9) scale = std::min(scale, std::max(0.0, room) / load[e]);
    }
    if (dead) {
      N.mcf.setCapacity(a, 0, N.S, N.T);
      ++tightened;
    } else if (scale < 1.0 - 1e-9) {
      N.mcf.setCapacity(a, toCap(static_cast<double>(f) * scale), N.S, N.T);
      ++tightened;
    }
  }
  return tightened;
}

void ResourceAllocator::solveResource(int r, const Deadline& dl, AllocationStats* st) {
  Net& N = net_[r];
  MinCostFlow::Cap big = static_cast<MinCostFlow::Cap>(1e15);
  int64_t settled0 = N.mcf.settled();
  for (int round = 0; round < 3; ++round) {
    auto res = N.mcf.solve(N.S, N.T, big, dl);
    if (st) {
      st->phases += res.phases;
      st->timedOut = st->timedOut || res.timedOut;
    }
    if (res.timedOut) {
      // anytime fallback: route every unmet unit through its shortage arc
      for (size_t j = 0; j < N.demandArc.size(); ++j) {
        int da = N.demandArc[j];
        if (da < 0) continue;
        MinCostFlow::Cap rem = N.mcf.capacity(da) - N.mcf.flow(da);
        if (rem > 0) N.mcf.pushAlong({N.shortArc[j], da}, rem);
      }
      break;
    }
    if (boundCapacities(r) == 0) break;
  }
  uint64_t settled = static_cast<uint64_t>(N.mcf.settled() - settled0);
  counters().nodesExplored += settled;
  counters().statesExplored += settled;
}

void ResourceAllocator::dropShipment(Shipment& sh) {
  if (!sh.active) return;
  sh.active = false;
  for (EdgeId e : sh.path) edgeUsed_[e] -= sh.amount;
  fleetLeft_ += sh.amount;
}

void ResourceAllocator::collectShipments(int r, double now, std::vector<int>* added, std::vector<int>* cancelled) {
  // Diff the flow on each transport arc against what was already materialised, so that a
  // re-optimisation after an event only cancels/creates the difference (stable plans).
  Net& N = net_[r];
  for (int a : N.transport) {
    ArcInfo& I = N.info[a / 2];
    double f = static_cast<double>(N.mcf.flow(a));
    double have = I.shipped + I.dropped;
    if (f < have - 0.5) {  // flow decreased: cancel shipments (latest first) until consistent
      while (!I.shipments.empty() && I.shipped + I.dropped > f + 0.5) {
        Shipment& sh = ship_[I.shipments.back()];
        I.shipped -= sh.amount;
        dropShipment(sh);
        if (cancelled) cancelled->push_back(sh.id);
        I.shipments.pop_back();
      }
      if (I.shipped + I.dropped > f + 0.5) I.dropped = std::max(0.0, f - I.shipped);
      have = I.shipped + I.dropped;
    }
    double extra = f - have;
    if (extra < cfg_.minShipment) continue;
    Shipment sh;
    sh.id = static_cast<int>(ship_.size());
    sh.from = I.i;
    sh.to = I.j;
    sh.resource = r;
    sh.amount = std::floor(extra);
    sh.unitCost = I.unitCost;
    sh.path = I.path;
    sh.pathTime = I.pathTime;
    sh.pathSigma = I.pathSigma;
    sh.arc = a;
    double lead = I.pathTime + 1.2816 * I.pathSigma + 12.0;
    double inc = lastIncoming_.empty() ? 0.0 : lastIncoming_[static_cast<size_t>(I.j) * NUM_RESOURCES + r];
    sh.deadline = deadlineFor(I.j, r, now, inc, lead);
    for (EdgeId e : sh.path) edgeUsed_[e] += sh.amount;
    fleetLeft_ -= sh.amount;
    I.shipped += sh.amount;
    I.shipments.push_back(sh.id);
    ship_.push_back(sh);
    if (added) added->push_back(sh.id);
  }
}

void ResourceAllocator::enforceTripBudget() {
  // Each shipment needs at least one trip: keep the most urgent ones (earliest deadline, then
  // highest shortage penalty per unit) and drop the rest; their deficits are re-planned next cycle.
  std::vector<int> act;
  for (const auto& sh : ship_)
    if (sh.active) act.push_back(sh.id);
  if (static_cast<int>(act.size()) <= cfg_.maxShipments) return;
  std::stable_sort(act.begin(), act.end(), [&](int a, int b) {
    const Shipment &A = ship_[a], &B = ship_[b];
    if (A.deadline != B.deadline) return A.deadline < B.deadline;
    return unitPenalty(A.to, A.resource) * A.amount > unitPenalty(B.to, B.resource) * B.amount;
  });
  for (size_t k = static_cast<size_t>(std::max(0, cfg_.maxShipments)); k < act.size(); ++k) {
    Shipment& sh = ship_[act[k]];
    Net& N = net_[sh.resource];
    if (sh.arc >= 0 && N.built && sh.arc / 2 < static_cast<int>(N.info.size())) {
      ArcInfo& I = N.info[sh.arc / 2];
      I.shipped -= sh.amount;
      I.dropped += sh.amount;
      I.shipments.erase(std::remove(I.shipments.begin(), I.shipments.end(), sh.id), I.shipments.end());
    }
    dropShipment(sh);
  }
}

void ResourceAllocator::refreshShortageWaste(int r) {
  Net& N = net_[r];
  int P = w_.numPlanets();
  std::vector<double> out(P, 0.0);
  for (const auto& sh : ship_)
    if (sh.active && sh.resource == r) out[sh.from] += sh.amount;
  for (int p = 0; p < P; ++p) {
    size_t k = static_cast<size_t>(p) * NUM_RESOURCES + r;
    shortage_[k] = (N.built && p < static_cast<int>(N.shortArc.size()) && N.shortArc[p] >= 0)
                       ? static_cast<double>(N.mcf.flow(N.shortArc[p]))
                       : 0.0;
    double ovf = p < static_cast<int>(overflow_[r].size()) ? overflow_[r][p] : 0.0;
    waste_[k] = std::max(0.0, ovf - out[p]);
  }
}

void ResourceAllocator::greedyAllocate(int r, double now) {
  // Baseline: deficits by decreasing penalty take from their cheapest sources.
  int P = w_.numPlanets();
  auto& sup = supply_[r];
  auto& dem = demand_[r];
  std::vector<double> supLeft = sup;
  std::vector<int> order;
  for (int j = 0; j < P; ++j)
    if (dem[j] >= cfg_.minShipment) order.push_back(j);
  std::vector<double> pen(P, 0.0);
  for (int j : order) pen[j] = unitPenalty(j, r);
  std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return pen[a] > pen[b]; });
  std::vector<std::pair<NodeId, double>> src;
  std::vector<std::vector<EdgeId>> paths;
  for (int j : order) {
    double need = dem[j];
    if (allocDl_.expired()) {  // out of time: the remaining deficits stay short this cycle
      shortage_[static_cast<size_t>(j) * NUM_RESOURCES + r] = need;
      continue;
    }
    router_.nearestSources(
        j, PROFILE_CARGO, cfg_.K, cfg_.settleLimit, [&](NodeId v) { return supLeft[v] >= cfg_.minShipment; }, src,
        &paths);
    for (size_t k = 0; k < src.size(); ++k) {
      NodeId i = src[k].first;
      if (need < cfg_.minShipment || fleetLeft_ < cfg_.minShipment) break;
      double unit = src[k].second / std::max(1.0, cfg_.refShipCap);
      if (unit >= pen[j]) continue;
      counters().candidatesEvaluated++;
      Path p;
      p.edges = paths[k];
      fillPathMetrics(w_.graph, p, PROFILE_CARGO);
      double bottleneck = INF;
      for (EdgeId e : p.edges) {
        auto it = edgeUsed_.find(e);
        bottleneck = std::min(bottleneck, w_.graph.cap(e) - (it == edgeUsed_.end() ? 0.0 : it->second));
      }
      double amt = std::floor(std::min({need, supLeft[i], bottleneck, fleetLeft_}));
      if (amt < cfg_.minShipment) continue;
      Shipment sh;
      sh.id = static_cast<int>(ship_.size());
      sh.from = i;
      sh.to = j;
      sh.resource = r;
      sh.amount = amt;
      sh.unitCost = unit;
      sh.path = p.edges;
      sh.pathTime = p.timeMean;
      sh.pathSigma = std::sqrt(p.timeVar);
      double inc = lastIncoming_.empty() ? 0.0 : lastIncoming_[static_cast<size_t>(j) * NUM_RESOURCES + r];
      sh.deadline = deadlineFor(j, r, now, inc, sh.pathTime + 1.2816 * sh.pathSigma + 12.0);
      for (EdgeId e : sh.path) edgeUsed_[e] += amt;
      fleetLeft_ -= amt;
      supLeft[i] -= amt;
      need -= amt;
      ship_.push_back(std::move(sh));
    }
    shortage_[static_cast<size_t>(j) * NUM_RESOURCES + r] = std::max(0.0, need);
  }
  std::vector<double> out(P, 0.0);
  for (const auto& sh : ship_)
    if (sh.resource == r) out[sh.from] += sh.amount;
  for (int p = 0; p < P; ++p)
    waste_[static_cast<size_t>(p) * NUM_RESOURCES + r] = std::max(0.0, overflow_[r][p] - out[p]);
}

AllocationStats ResourceAllocator::allocate(const AllocConfig& cfg, double now, const std::vector<double>& incoming,
                                            const std::vector<double>& outgoing, const Deadline& dl) {
  Timer timer;
  cfg_ = cfg;
  now_ = now;
  AllocationStats st;
  int P = w_.numPlanets();
  ship_.clear();
  edgeUsed_.clear();
  arcByEdge_.clear();
  shortage_.assign(static_cast<size_t>(P) * NUM_RESOURCES, 0.0);
  waste_.assign(static_cast<size_t>(P) * NUM_RESOURCES, 0.0);
  lastIncoming_ = incoming;
  lastOutgoing_ = outgoing;
  fleetLeft_ = cfg.fleetCapacity;
  for (int r = 0; r < NUM_RESOURCES; ++r) {
    auto& sup = supply_[r];
    auto& dem = demand_[r];
    auto& ovf = overflow_[r];
    sup.assign(P, 0.0);
    dem.assign(P, 0.0);
    ovf.assign(P, 0.0);
    for (int p = 0; p < P; ++p) {
      size_t k = static_cast<size_t>(p) * NUM_RESOURCES + r;
      balance(p, r, incoming.empty() ? 0.0 : incoming[k], outgoing.empty() ? 0.0 : outgoing[k], sup[p], dem[p], ovf[p]);
      if (dem[p] >= cfg_.minShipment) st.deficits++;
      if (sup[p] >= cfg_.minShipment) st.supplies++;
    }
  }
  inAllocate_ = true;
  allocDl_ = dl;
  if (cfg.method == AllocMethod::MinCostFlow) {
    // at most half of the allocation time for source discovery, the rest for the flow solves
    precomputeSources(Deadline::inMs(0.5 * std::max(0.0, dl.remainingMs())));
  }
  for (int r : CRITICALITY_ORDER) {
    auto& sup = supply_[r];
    auto& dem = demand_[r];
    auto& ovf = overflow_[r];
    Net& N = net_[r];
    N = Net();
    if (cfg.method == AllocMethod::Greedy) {
      greedyAllocate(r, now);
      continue;
    }
    N.mcf.reset(0);
    N.S = N.mcf.addNode();
    N.T = N.mcf.addNode();
    N.F = N.mcf.addNode();
    N.nodeOf.assign(P, -1);
    N.supplyArc.assign(P, -1);
    N.supplyArc2.assign(P, -1);
    N.demandArc.assign(P, -1);
    N.shortArc.assign(P, -1);
    N.penalty.assign(P, 0.0);
    newArc(N, N.S, N.F, std::max(0.0, fleetLeft_), 0.0, makeInfo<ArcInfo>(4, -1, -1));
    bool anyDeficit = false;
    for (int p = 0; p < P && !anyDeficit; ++p) anyDeficit = dem[p] >= cfg_.minShipment;
    N.built = true;
    if (anyDeficit) {
      for (int p = 0; p < P; ++p)
        if (sup[p] >= cfg_.minShipment) addSupplyNode(r, p, sup[p], ovf[p]);
      for (int p = 0; p < P; ++p)
        if (dem[p] >= cfg_.minShipment) addDeficitNode(r, p, dem[p]);
      solveResource(r, dl, &st);
      collectShipments(r, now, nullptr, nullptr);
    }
    st.arcs += N.mcf.numArcs();
  }
  inAllocate_ = false;
  enforceTripBudget();
  for (int r = 0; r < NUM_RESOURCES; ++r)
    if (net_[r].built) refreshShortageWaste(r);
  for (const auto& sh : ship_)
    if (sh.active) {
      st.transportCost += sh.amount * sh.unitCost;
      st.plannedUnits += sh.amount;
    }
  for (int p = 0; p < P; ++p)
    for (int r = 0; r < NUM_RESOURCES; ++r) {
      size_t k = static_cast<size_t>(p) * NUM_RESOURCES + r;
      st.shortageUnits += shortage_[k];
      st.shortagePenalty += shortage_[k] * unitPenalty(p, r);
      st.wasteUnits += waste_[k];
    }
  st.ms = timer.ms();
  return st;
}

ResourceAllocator::RepairResult ResourceAllocator::repair(const std::vector<EdgeId>& changedEdges,
                                                          const std::vector<NodeId>& changedPlanets, double now,
                                                          const Deadline& dl) {
  Timer timer;
  RepairResult rr;
  now_ = now;
  int P = w_.numPlanets();
  shortage_.resize(static_cast<size_t>(P) * NUM_RESOURCES, 0.0);
  waste_.resize(static_cast<size_t>(P) * NUM_RESOURCES, 0.0);
  lastIncoming_.resize(static_cast<size_t>(P) * NUM_RESOURCES, 0.0);
  lastOutgoing_.resize(static_cast<size_t>(P) * NUM_RESOURCES, 0.0);
  std::array<bool, NUM_RESOURCES> touched{};
  // 1) arcs whose priced route uses a changed edge: close them, re-route if possible
  for (EdgeId e : changedEdges) {
    auto it = arcByEdge_.find(e);
    if (it == arcByEdge_.end()) continue;
    auto refs = it->second;
    arcByEdge_.erase(it);
    for (auto [r, a] : refs) {
      Net& N = net_[r];
      if (!N.built || a / 2 >= static_cast<int>(N.info.size())) continue;
      ArcInfo I = N.info[a / 2];
      if (I.kind != 1 || N.mcf.capacity(a) == 0) continue;
      bool stillOk = pathUsable(w_.graph, I.path, 0.0);
      double newCost = 0;
      for (EdgeId x : I.path) newCost += edgeCost(w_.graph, x, PROFILE_CARGO);
      double oldCost = I.unitCost * std::max(1.0, cfg_.refShipCap);
      if (stillOk && newCost <= oldCost * 1.05) continue;  // still valid and not notably worse
      N.mcf.setCapacity(a, 0, N.S, N.T);
      touched[r] = true;
      const Path& p = router_.route(I.i, I.j, PROFILE_CARGO);
      if (p.found && N.nodeOf[I.i] >= 0 && N.nodeOf[I.j] >= 0 && N.demandArc[I.j] >= 0) {
        ArcInfo ni = makeInfo<ArcInfo>(1, I.i, I.j, p.cost / std::max(1.0, cfg_.refShipCap));
        ni.path = p.edges;
        ni.pathTime = p.timeMean;
        ni.pathSigma = std::sqrt(p.timeVar);
        if (ni.unitCost < N.penalty[I.j]) {
          int na = newArc(N, N.nodeOf[I.i], N.nodeOf[I.j], static_cast<double>(N.mcf.capacity(N.demandArc[I.j])),
                          ni.unitCost, ni);
          N.transport.push_back(na);
          for (EdgeId x : ni.path) arcByEdge_[x].push_back({r, na});
        }
      }
    }
  }
  // 2) planets whose rates/stock/population changed: update supply / demand arcs
  for (NodeId p : changedPlanets) {
    if (p < 0 || p >= P) continue;
    for (int r = 0; r < NUM_RESOURCES; ++r) {
      Net& N = net_[r];
      if (!N.built) continue;
      if (static_cast<int>(N.nodeOf.size()) < P) {
        N.nodeOf.resize(P, -1); N.supplyArc.resize(P, -1); N.supplyArc2.resize(P, -1);
        N.demandArc.resize(P, -1); N.shortArc.resize(P, -1); N.penalty.resize(P, 0.0);
      }
      for (auto* v : {&supply_[r], &demand_[r], &overflow_[r]})
        if (static_cast<int>(v->size()) < P) v->resize(P, 0.0);
      size_t k = static_cast<size_t>(p) * NUM_RESOURCES + r;
      double s, d, o;
      balance(p, r, lastIncoming_[k], lastOutgoing_[k], s, d, o);
      if (std::fabs(s - supply_[r][p]) < 1 && std::fabs(d - demand_[r][p]) < 1) continue;
      supply_[r][p] = s;
      demand_[r][p] = d;
      overflow_[r][p] = o;
      touched[r] = true;
      if (N.supplyArc[p] >= 0) {
        N.mcf.setCapacity(N.supplyArc[p], toCap(o), N.S, N.T);
        N.mcf.setCapacity(N.supplyArc2[p], toCap(std::max(0.0, s - o)), N.S, N.T);
        if (d >= cfg_.minShipment && N.demandArc[p] < 0) {  // supplier turned into a deficit planet
          N.supplyArc[p] = N.supplyArc2[p] = -1;
          addDeficitNode(r, p, d);
        }
      } else if (N.demandArc[p] >= 0) {
        N.mcf.setCapacity(N.demandArc[p], toCap(d), N.S, N.T);
        N.mcf.setCapacity(N.shortArc[p], toCap(d), N.S, N.T);
      } else if (d >= cfg_.minShipment) {
        addDeficitNode(r, p, d);
      } else if (s >= cfg_.minShipment) {
        addSupplyNode(r, p, s, o);
      }
    }
  }
  // 3) warm-start re-augmentation and shipment diff
  for (int r = 0; r < NUM_RESOURCES; ++r) {
    if (!touched[r]) continue;
    rr.resourcesTouched++;
    solveResource(r, dl, nullptr);
    collectShipments(r, now, &rr.added, &rr.cancelled);
    refreshShortageWaste(r);
  }
  rr.ms = timer.ms();
  return rr;
}

double ResourceAllocator::totalObjective() const {
  double c = 0;
  for (const auto& N : net_)
    if (N.built) c += static_cast<double>(N.mcf.totalCost()) / COST_SCALE;
  return c;
}

size_t ResourceAllocator::memoryBytes() const {
  size_t b = ship_.size() * (sizeof(Shipment) + 64);
  for (const auto& N : net_) b += N.info.size() * (sizeof(ArcInfo) + 16) + N.mcf.numArcs() * 2 * 40;
  return b;
}

}  // namespace chronos
