// scheduler.cpp - ALNS (+SA acceptance) mission scheduler, construction heuristics,
// local search, constraint validation and the GA / SA / B&B baselines.
// See scheduler.hpp for the complexity of each component.
#include <cmath>
#include <cstdlib>
#include "chronos/scheduler.hpp"

#include <queue>
#include <sstream>
#include <unordered_set>

namespace chronos {

namespace {
constexpr double INFEASIBLE = 1e9;
constexpr double EVALS_PER_MS = 400.0;  // deterministic work model: mission-evaluations per ms
const char* DESTROY_NAMES[5] = {"random", "worst", "shaw", "urgent", "dependency"};
const char* REPAIR_NAMES[3] = {"greedy", "regret2", "regret3"};

int roulette(const std::vector<double>& w, Rng& rng) {
  double sum = 0;
  for (double x : w) sum += x;
  double u = rng.uniform() * sum;
  for (size_t i = 0; i < w.size(); ++i) {
    u -= w[i];
    if (u <= 0) return static_cast<int>(i);
  }
  return static_cast<int>(w.size()) - 1;
}
}  // namespace

const char* schedMethodName(SchedMethod m) {
  switch (m) {
    case SchedMethod::Greedy: return "greedy";
    case SchedMethod::RegretALNS: return "regret_alns";
    case SchedMethod::LocalRepair: return "local_repair";
    case SchedMethod::GreedyPruneALNS: return "greedy_prune_alns";
    case SchedMethod::GA: return "ga";
    case SchedMethod::SA: return "sa";
    case SchedMethod::BranchAndBound: return "bnb";
    default: return "auto";
  }
}

SchedMethod schedMethodFromString(const std::string& s) {
  if (s == "greedy") return SchedMethod::Greedy;
  if (s == "alns" || s == "regret_alns") return SchedMethod::RegretALNS;
  if (s == "local" || s == "local_repair") return SchedMethod::LocalRepair;
  if (s == "greedy_prune_alns" || s == "fast") return SchedMethod::GreedyPruneALNS;
  if (s == "ga") return SchedMethod::GA;
  if (s == "sa") return SchedMethod::SA;
  if (s == "bnb") return SchedMethod::BranchAndBound;
  return SchedMethod::Auto;
}

Scheduler::Scheduler(World& w, Router& r, const SchedulerConfig& cfg, uint64_t seed)
    : w_(w), router_(r), cfg_(cfg), rng_(seed) {
  dW_.assign(5, 1.0); dS_.assign(5, 0.0); dN_.assign(5, 0);
  rW_.assign(3, 1.0); rS_.assign(3, 0.0); rN_.assign(3, 0);
}

// ------------------------------------------------------------------ bookkeeping
void Scheduler::ensureSizes() {
  size_t M = w_.missions.size(), S = w_.ships.size();
  if (sol_.shipOf.size() < M) sol_.shipOf.resize(M, -1);
  if (isActive_.size() < M) isActive_.resize(M, 0);
  if (cand_.size() < M) cand_.resize(M);
  if (poolFlag_.size() < M) poolFlag_.resize(M, 0);
  if (sol_.seq.size() < S) {
    sol_.seq.resize(S);
    sol_.shipCost.resize(S, 0.0);
  }
  if (shipMask_.size() < S) shipMask_.resize(S, 1);
  if (touchStamp_.size() < S) touchStamp_.resize(S, 0);
  if (dirtyFlag_.size() < S) dirtyFlag_.resize(S, 0);
  if (refuelMask_.size() != w_.planets.size()) refuelMask_ = w_.refuelMask();
}

bool Scheduler::shipEnabled(int s) const {
  return s >= 0 && s < static_cast<int>(w_.ships.size()) && w_.ships[s].alive &&
         (!restricted_ || (s < static_cast<int>(shipMask_.size()) && shipMask_[s]));
}

void Scheduler::restrictShips(const std::vector<int>& ships) {
  ensureSizes();
  restricted_ = !ships.empty();
  std::fill(shipMask_.begin(), shipMask_.end(), ships.empty() ? 1 : 0);
  for (int s : ships)
    if (s >= 0 && s < static_cast<int>(shipMask_.size())) shipMask_[s] = 1;
  recomputeSlots();
}

void Scheduler::activate(const std::vector<int>& missions) {
  ensureSizes();
  for (int m : missions) {
    if (m < 0 || m >= static_cast<int>(w_.missions.size()) || isActive_[m]) continue;
    if (!w_.missions[m].active()) continue;
    isActive_[m] = 1;
    active_.push_back(m);
    sol_.shipOf[m] = -1;
    sol_.total += rejectPenalty(m);
  }
}

void Scheduler::deactivate(const std::vector<int>& missions) {
  ensureSizes();
  bool any = false;
  for (int m : missions) {
    if (!isActive(m)) continue;
    applyRemove(m);
    sol_.total -= rejectPenalty(m);
    isActive_[m] = 0;
    any = true;
  }
  if (any) active_.erase(std::remove_if(active_.begin(), active_.end(), [&](int m) { return !isActive_[m]; }),
                         active_.end());
}

void Scheduler::refreshShip(int s) {
  ensureSizes();
  if (s < 0 || s >= static_cast<int>(w_.ships.size())) return;
  double nc = sol_.seq[s].empty() ? 0.0 : evaluateShip(s, sol_.seq[s], nullptr);
  sol_.total += nc - sol_.shipCost[s];
  sol_.shipCost[s] = nc;
  recomputeSlots();
}

void Scheduler::recomputeSlots() {
  slots_ = 0;
  for (size_t s = 0; s < w_.ships.size(); ++s)
    if (shipEnabled(static_cast<int>(s))) slots_ += static_cast<double>(sol_.seq[s].size() + 1);
}

std::vector<int> Scheduler::unassigned() const {
  std::vector<int> u;
  for (int m : active_)
    if (sol_.shipOf[m] < 0) u.push_back(m);
  return u;
}

int Scheduler::assignedCount() const {
  int c = 0;
  for (int m : active_)
    if (sol_.shipOf[m] >= 0) ++c;
  return c;
}

double Scheduler::rejectPenalty(int m) const { return w_.missions[m].value() * cfg_.rejectFactor; }

bool Scheduler::capable(int s, int m) const {
  const Ship& S = w_.ships[s];
  const Mission& M = w_.missions[m];
  return S.alive && (S.caps & M.requiredCaps) == M.requiredCaps && S.cargoCap + 1e-9 >= M.amount;
}

int Scheduler::profileFor(int m) const {
  const Mission& M = w_.missions[m];
  if (M.source == MissionSource::Emergency || M.priority >= 5) return PROFILE_EMERGENCY;
  if (M.priority == 4 || M.resource == static_cast<int>(Resource::Medicine)) return PROFILE_SAFE;
  if (M.priority <= 2) return PROFILE_CARGO;
  return PROFILE_BALANCED;
}

void Scheduler::syncWithWorld() {
  ensureSizes();
  refuelMask_ = w_.refuelMask();
  // drop missions that are no longer schedulable and missions on dead ships
  for (size_t s = 0; s < w_.ships.size(); ++s) {
    auto& q = sol_.seq[s];
    bool alive = w_.ships[s].alive;
    std::vector<int> keep;
    for (int m : q) {
      if (alive && w_.missions[m].active()) keep.push_back(m);
      else sol_.shipOf[m] = -1;
    }
    q.swap(keep);
  }
  std::vector<int> next;
  for (int m : active_) isActive_[m] = 0;
  for (size_t m = 0; m < w_.missions.size(); ++m) {
    const Mission& M = w_.missions[m];
    if (M.active() && M.release <= now_ + 1e-9) {
      next.push_back(static_cast<int>(m));
      isActive_[m] = 1;
    } else {
      if (sol_.shipOf[m] >= 0) {
        auto& q = sol_.seq[sol_.shipOf[m]];
        q.erase(std::remove(q.begin(), q.end(), static_cast<int>(m)), q.end());
      }
      sol_.shipOf[m] = -1;
    }
  }
  active_.swap(next);
  double xmin = INF, xmax = -INF, ymin = INF, ymax = -INF;
  for (const auto& p : w_.planets) {
    xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
  }
  scale_ = std::max(1.0, std::max(xmax - xmin, ymax - ymin));
  recomputeTotal();
}

double Scheduler::recomputeTotal() {
  ensureSizes();
  double t = 0;
  for (size_t s = 0; s < w_.ships.size(); ++s) {
    sol_.shipCost[s] = sol_.seq[s].empty() ? 0.0 : evaluateShip(static_cast<int>(s), sol_.seq[s], nullptr);
    t += sol_.shipCost[s];
  }
  for (int m : active_)
    if (sol_.shipOf[m] < 0) t += rejectPenalty(m);
  sol_.total = t;
  recomputeSlots();
  return t;
}

// ------------------------------------------------------------------ evaluation
Scheduler::LegInfo Scheduler::leg(NodeId a, NodeId b, int profile, double minCap, std::vector<EdgeId>* edges) {
  LegInfo li;
  if (a == b) {
    li.found = true;
    if (edges) edges->clear();
    return li;
  }
  const Path& p = router_.route(a, b, profile, minCap);
  li.found = p.found;
  if (!p.found) return li;
  li.cost = p.cost; li.time = p.timeMean; li.var = p.timeVar; li.fuel = p.fuel;
  li.rel = p.reliability; li.risk = p.risk; li.dist = p.dist; li.bottleneck = p.bottleneck;
  if (edges) *edges = p.edges;
  return li;
}

Path Scheduler::refuelAlongRoute(NodeId a, NodeId b, int profile, double minCap, double fuelStart, double fuelCap) {
  // Refuel along the best route: take the memoised route() path a -> b and walk it, filling the
  // tank at the current node whenever the next edge needs more fuel than is left. O(|path|) after
  // one cached query. Feasible by construction; it fails only if a needed stop is not a refuel
  // node or an edge needs more than a full tank, and then the exact label search takes over.
  // (Most galaxies have refuel stations almost everywhere, so this settles nearly every leg.)
  Path none;
  const Path& base = router_.route(a, b, profile, minCap);
  if (!base.found) return none;
  const DynamicGraph& g = w_.graph;
  double fuel = fuelStart;
  std::vector<NodeId> stops;
  for (EdgeId e : base.edges) {
    double need = g.fuel(e);
    if (need > fuelCap + 1e-9) return none;
    if (fuel + 1e-9 < need) {
      NodeId u = g.from(e);
      if (u < 0 || u >= static_cast<NodeId>(refuelMask_.size()) || !refuelMask_[u]) return none;
      fuel = fuelCap;
      stops.push_back(u);
    }
    fuel -= need;
  }
  Path p = base;  // copy: route() references are short-lived
  p.refuelNodes = std::move(stops);
  p.cost += cfg_.refuelCost * static_cast<double>(p.refuelNodes.size());  // Router::routeFuel convention
  p.fuelAtArrival = fuel;
  counters().refuelAlongRoute++;
  return p;
}

bool Scheduler::fuelLeg(int s, NodeId a, NodeId b, int profile, double minCap, double& fuel, LegInfo& li,
                        std::vector<EdgeId>* edges, std::vector<NodeId>* refuels) {
  // Called when the direct leg needs more fuel than available and refuelling at `a` is impossible
  // or insufficient. First: refuel along the best route (refuelAlongRoute); fallback: the exact
  // fuel-constrained label search. Both results are memoised per (leg, profile, capacity class,
  // start fuel in 1/16 tank, tank size).
  const Ship& S = w_.ships[s];
  int fb = static_cast<int>(fuel / std::max(1e-9, S.fuelCap) * 16.0);
  uint64_t key = hashCombine(RouteCache::key(a, b, profile, Router::capClass(minCap)),
                             static_cast<uint64_t>(fb) * 1000003ULL + static_cast<uint64_t>(std::llround(S.fuelCap)));
  if (key == FlatIndex::EMPTY) key ^= 1;  // reserved sentinel
  int32_t slot = fuelIndex_.find(key);
  if (slot < 0) {
    double fq = fb / 16.0 * S.fuelCap;  // quantised down: conservative w.r.t. actual fuel
    Path p = refuelAlongRoute(a, b, profile, minCap, fq, S.fuelCap);
    if (!p.found)
      p = router_.routeFuel(a, b, profile, S.fuelCap, fq, refuelMask_, Router::capOfClass(Router::capClass(minCap)),
                            16, cfg_.refuelCost);
    if (fuelIndex_.size() > 50000) invalidateFuelCache();
    slot = static_cast<int32_t>(fuelPaths_.size());
    fuelPaths_.push_back(std::move(p));
    fuelIndex_.put(key, slot);
  }
  const Path& p = fuelPaths_[static_cast<size_t>(slot)];
  if (!p.found) return false;
  li.found = true;
  li.cost = p.cost - cfg_.refuelCost * static_cast<double>(p.refuelNodes.size());
  li.time = p.timeMean; li.var = p.timeVar; li.fuel = p.fuel; li.rel = p.reliability; li.risk = p.risk;
  li.dist = p.dist; li.bottleneck = p.bottleneck;
  if (edges) *edges = p.edges;
  if (refuels) refuels->insert(refuels->end(), p.refuelNodes.begin(), p.refuelNodes.end());
  fuel = p.fuelAtArrival;  // conservative: computed from the quantised (lower) start fuel
  return true;
}

double Scheduler::evaluateShip(int s, const std::vector<int>& seq, ShipEval* out) {
  ++evals_;
  counters().statesExplored++;
  const Ship& S = w_.ships[s];
  double t = std::max(S.availableAt, now_), var = 0, fuel = S.fuel, total = 0;
  NodeId loc = S.location;
  if (out) {
    out->items.clear();
    out->feasible = true;
  }
  for (size_t idx = 0; idx < seq.size(); ++idx) {
    int m = seq[idx];
    const Mission& M = w_.missions[m];
    MissionEval me;
    me.mission = m;
    me.start = t;
    me.profile = profileFor(m);
    LegInfo e1, e2;
    int nRef = 0;
    std::vector<NodeId> refs;
    bool ok = S.alive && capable(s, m);
    // leg runner: direct route, refuel at the leg start if needed, else fuel-constrained labels
    auto runLeg = [&](NodeId a, NodeId b, int profile, double minCap, const std::vector<EdgeId>* fixed, LegInfo& li,
                      std::vector<EdgeId>* edges) -> bool {
      if (fixed && !fixed->empty() && pathUsable(w_.graph, *fixed, minCap) && w_.graph.from(fixed->front()) == a &&
          w_.graph.to(fixed->back()) == b) {
        Path p;
        p.edges = *fixed;
        fillPathMetrics(w_.graph, p, profile);
        li.found = true;
        li.cost = p.cost; li.time = p.timeMean; li.var = p.timeVar; li.fuel = p.fuel; li.rel = p.reliability;
        li.risk = p.risk; li.dist = p.dist; li.bottleneck = p.bottleneck;
        if (edges) *edges = p.edges;
      } else {
        li = leg(a, b, profile, minCap, edges);
      }
      if (!li.found) return false;
      // opportunistic top-up at refuel nodes when the tank is below half (keeps the fleet mobile)
      if (a < static_cast<NodeId>(refuelMask_.size()) && refuelMask_[a] && fuel < 0.5 * S.fuelCap &&
          li.fuel <= S.fuelCap + 1e-9) {
        ++nRef;
        refs.push_back(a);
        fuel = S.fuelCap - li.fuel;
        return true;
      }
      if (li.fuel <= fuel + 1e-9) {
        fuel -= li.fuel;
        return true;
      }
      if (a < static_cast<NodeId>(refuelMask_.size()) && refuelMask_[a] && li.fuel <= S.fuelCap + 1e-9) {
        ++nRef;
        refs.push_back(a);
        fuel = S.fuelCap - li.fuel;
        return true;
      }
      size_t before = refs.size();
      if (!fuelLeg(s, a, b, profile, minCap, fuel, li, edges, &refs)) return false;
      nRef += static_cast<int>(refs.size() - before);
      return true;
    };
    if (ok) ok = runLeg(loc, M.origin, PROFILE_CARGO, 0.0, nullptr, e1, out ? &me.emptyPath : nullptr);
    me.refuelSplit = static_cast<int>(refs.size());
    if (ok) ok = runLeg(M.origin, M.dest, me.profile, M.amount, &M.fixedPath, e2, out ? &me.loadedPath : nullptr);
    if (!ok) {
      me.feasible = false;
      me.cost = INFEASIBLE;
      total += INFEASIBLE;
      if (out) {
        out->feasible = false;
        out->items.push_back(std::move(me));
      }
      continue;
    }
    double sp = S.speed;
    double t1 = t + e1.time / sp + nRef * cfg_.refuelH;
    me.arriveOrigin = t1;
    double dep = std::max(t1, M.release) + cfg_.loadH;
    me.depart = dep;
    double arr = dep + e2.time / sp;
    var += (e1.var + e2.var) / (sp * sp);
    me.arrive = arr;
    me.sigma = std::sqrt(var);
    double wgt = M.weight();
    me.travelCost = S.opCost * (e1.time + e2.time) / sp + cfg_.emptyLegPlanWeight * cfg_.emptyLegFactor * e1.cost +
                    e2.cost + nRef * cfg_.refuelCost;
    me.latePen = cfg_.wLate * wgt * std::max(0.0, arr - M.deadline);
    if (M.priority >= 4) {  // chance constraint P(on time) >= 0.9 via slack z*sigma
      double need = cfg_.zOnTime * me.sigma, slack = M.deadline - arr;
      if (slack < need) me.chancePen = cfg_.wChance * wgt * (need - slack);
    }
    me.pOnTime = me.sigma > 1e-9 ? normalCdf((M.deadline - arr) / me.sigma) : (arr <= M.deadline + 1e-9 ? 1.0 : 0.0);
    me.failP = 1.0 - S.reliability * e1.rel * e2.rel;
    me.failPen = me.failP * M.value();
    me.cost = me.travelCost + me.latePen + me.chancePen + me.failPen;
    me.dist = e1.dist + e2.dist;
    me.fuel = e1.fuel + e2.fuel;
    me.risk = 1.0 - (1.0 - e1.risk) * (1.0 - e2.risk);
    me.reliability = S.reliability * e1.rel * e2.rel;
    me.refuels = refs;
    total += me.cost;
    t = arr + cfg_.unloadH;
    loc = M.dest;
    if (out) out->items.push_back(std::move(me));
  }
  if (out) out->cost = total;
  return total;
}

// ------------------------------------------------------------------ insertion primitives
std::vector<int> Scheduler::positionsFor(int m, int s) const {
  const auto& q = sol_.seq[s];
  int L = static_cast<int>(q.size());
  std::vector<int> pos;
  if (cfg_.appendOnly && cfg_.usePruning) {
    pos.push_back(L);
    return pos;
  }
  if (!cfg_.usePruning || L <= 2 * cfg_.beam + 1) {
    for (int p = 0; p <= L; ++p) pos.push_back(p);
    return pos;
  }
  double dl = w_.missions[m].deadline;
  int rank = 0;
  for (int x : q)
    if (w_.missions[x].deadline <= dl) ++rank;
  for (int p = std::max(0, rank - cfg_.beam); p <= std::min(L, rank + cfg_.beam); ++p) pos.push_back(p);
  if (pos.back() != L) pos.push_back(L);
  return pos;
}

double Scheduler::insertionDelta(int m, int s, int pos) {
  counters().candidatesEvaluated++;
  counters().insertionEvals++;
  const auto& q = sol_.seq[s];
  tmp_.assign(q.begin(), q.end());
  tmp_.insert(tmp_.begin() + pos, m);
  return evaluateShip(s, tmp_, nullptr) - sol_.shipCost[s];
}

Scheduler::Ins Scheduler::bestInsertion(int m, std::vector<double>* perShipBest, bool appendOnly) {
  Ins best;
  ensureCandidates(m);  // lazy mode: candidate list built when the mission is first tried this cycle
  const auto& cands = candidates(m);
  if (perShipBest) perShipBest->assign(cands.size(), INF);
  counters().naiveStates += slots_;  // a naive scheduler evaluates every slot of every ship
  for (size_t i = 0; i < cands.size(); ++i) {
    int s = cands[i];
    if (!shipEnabled(s) || !capable(s, m)) continue;
    std::vector<int> pos;
    if (appendOnly) pos.push_back(static_cast<int>(sol_.seq[s].size()));
    else pos = positionsFor(m, s);
    for (int p : pos) {
      double d = insertionDelta(m, s, p);
      if (perShipBest && d < (*perShipBest)[i]) (*perShipBest)[i] = d;
      if (d < best.delta) {
        best.delta = d;
        best.ship = s;
        best.pos = p;
      }
    }
  }
  return best;
}

void Scheduler::touch(int s) {
  if (recording_ && touchStamp_[s] != stamp_) {
    touchStamp_[s] = stamp_;
    undoSeq_.push_back({s, sol_.seq[s]});
    undoCost_.push_back({s, sol_.shipCost[s]});
  }
  if (snapOn_) {
    if (snap_.find(s) == snap_.end()) snap_.emplace(s, std::make_pair(sol_.seq[s], sol_.shipCost[s]));
    if (!dirtyFlag_[s]) {
      dirtyFlag_[s] = 1;
      dirty_.push_back(s);
    }
  }
}

void Scheduler::applyInsert(int m, int s, int pos) {
  touch(s);
  auto& q = sol_.seq[s];
  pos = std::min(std::max(0, pos), static_cast<int>(q.size()));
  q.insert(q.begin() + pos, m);
  if (recording_) undoShipOf_.push_back({m, sol_.shipOf[m]});
  sol_.shipOf[m] = s;
  double nc = evaluateShip(s, q, nullptr);
  sol_.total += nc - sol_.shipCost[s] - (isActive(m) ? rejectPenalty(m) : 0.0);
  sol_.shipCost[s] = nc;
  slots_ += 1;
}

void Scheduler::applyRemove(int m) {
  int s = sol_.shipOf[m];
  if (s < 0) return;
  touch(s);
  auto& q = sol_.seq[s];
  q.erase(std::remove(q.begin(), q.end(), m), q.end());
  if (recording_) undoShipOf_.push_back({m, s});
  sol_.shipOf[m] = -1;
  unassignedLog_.push_back(m);
  double nc = q.empty() ? 0.0 : evaluateShip(s, q, nullptr);
  sol_.total += nc - sol_.shipCost[s] + (isActive(m) ? rejectPenalty(m) : 0.0);
  sol_.shipCost[s] = nc;
  slots_ -= 1;
}

void Scheduler::setSequence(int s, const std::vector<int>& seq) {
  touch(s);
  sol_.seq[s] = seq;
  for (int m : seq) {
    if (recording_) undoShipOf_.push_back({m, sol_.shipOf[m]});
    sol_.shipOf[m] = s;
  }
  double nc = seq.empty() ? 0.0 : evaluateShip(s, seq, nullptr);
  sol_.total += nc - sol_.shipCost[s];
  sol_.shipCost[s] = nc;
}

void Scheduler::beginUndo() {
  recording_ = true;
  if (++stamp_ == 0) {
    std::fill(touchStamp_.begin(), touchStamp_.end(), 0u);
    stamp_ = 1;
  }
  undoSeq_.clear();
  undoCost_.clear();
  undoShipOf_.clear();
  undoTotal_ = sol_.total;
  undoSlots_ = slots_;
}

void Scheduler::rollback() {
  for (auto it = undoShipOf_.rbegin(); it != undoShipOf_.rend(); ++it) sol_.shipOf[it->first] = it->second;
  for (auto& [s, q] : undoSeq_) sol_.seq[s].swap(q);
  for (auto& [s, c] : undoCost_) sol_.shipCost[s] = c;
  sol_.total = undoTotal_;
  slots_ = undoSlots_;
  recording_ = false;
}

std::vector<int> Scheduler::remove(const std::vector<int>& missions) {
  std::vector<int> was;
  for (int m : missions) {
    if (m < 0 || m >= static_cast<int>(sol_.shipOf.size())) continue;
    if (sol_.shipOf[m] >= 0) {
      was.push_back(m);
      applyRemove(m);
    }
  }
  return was;
}

void Scheduler::unassignAll() {
  for (int m : active_) applyRemove(m);
}

void Scheduler::restore(const Solution& s) {
  sol_ = s;
  ensureSizes();
  recomputeSlots();
}

// ------------------------------------------------------------------ snapshots of the best solution
void Scheduler::startSnapshots() {
  snapOn_ = true;
  snap_.clear();
  for (int s : dirty_) dirtyFlag_[s] = 0;
  dirty_.clear();
  bestTotal_ = sol_.total;
}

void Scheduler::captureBest() {
  for (int s : dirty_) {
    snap_[s] = {sol_.seq[s], sol_.shipCost[s]};
    dirtyFlag_[s] = 0;
  }
  dirty_.clear();
  bestTotal_ = sol_.total;
}

void Scheduler::restoreBest() {
  if (sol_.total <= bestTotal_ + 1e-9) return;
  for (auto& [s, sc] : snap_)
    for (int m : sol_.seq[s]) sol_.shipOf[m] = -1;
  for (auto& [s, sc] : snap_) {
    sol_.seq[s] = sc.first;
    sol_.shipCost[s] = sc.second;
    for (int m : sol_.seq[s]) sol_.shipOf[m] = s;
  }
  sol_.total = bestTotal_;
  recomputeSlots();
}

void Scheduler::stopSnapshots() {
  snapOn_ = false;
  snap_.clear();
  for (int s : dirty_) dirtyFlag_[s] = 0;
  dirty_.clear();
}

// ------------------------------------------------------------------ candidates
const std::vector<int>& Scheduler::candidates(int m) const {
  static const std::vector<int> empty;
  return (m >= 0 && m < static_cast<int>(cand_.size())) ? cand_[m] : empty;
}

void Scheduler::beginCandidateEpoch(bool lazy) {
  ensureSizes();
  ++candEpoch_;
  gridValid_ = false;
  lazyCand_ = lazy;
}

void Scheduler::ensureCandidates(int m) {
  if (!lazyCand_ || m < 0 || m >= static_cast<int>(cand_.size())) return;
  if (candStamp_.size() < cand_.size()) candStamp_.resize(cand_.size(), 0);
  if (candStamp_[m] == candEpoch_) return;
  candOne_.assign(1, m);
  buildCandidates(candOne_);
}

void Scheduler::buildCandidates(const std::vector<int>& missions) {
  ensureSizes();
  if (!lazyCand_) gridValid_ = false;  // eager build: fresh grid, as before
  int S = static_cast<int>(w_.ships.size());
  if (!cfg_.usePruning) {
    for (int m : missions) {
      cand_[m].clear();
      for (int s = 0; s < S; ++s)
        if (shipEnabled(s) && capable(s, m)) cand_[m].push_back(s);
    }
    return;
  }
  if (!gridValid_) {  // ship grid: built once per candidate epoch, shared by lazy single builds
    std::vector<double> xs, ys;
    std::vector<int> ids;
    for (int s = 0; s < S; ++s) {
      if (!shipEnabled(s)) continue;
      const Planet& p = w_.planets[w_.ships[s].location];
      xs.push_back(p.x);
      ys.push_back(p.y);
      ids.push_back(s);
    }
    grid_.build(xs, ys, ids, 2.0);
    gridValid_ = true;
  }
  const SpatialGrid2D& grid = grid_;
  if (candStamp_.size() < cand_.size()) candStamp_.resize(cand_.size(), 0);
  for (int m : missions) candStamp_[m] = candEpoch_;
  std::vector<int> near;
  const int K = std::max(1, cfg_.kNearest);
  for (int m : missions) {
    const Mission& M = w_.missions[m];
    const Planet& O = w_.planets[M.origin];
    grid.kNearest(O.x, O.y, K + 8, near, [&](int s) { return capable(s, m); });
    int prof = profileFor(m);
    double lbLoaded = router_.lowerBound(M.origin, M.dest, PROFILE_TIME);
    const Path& loaded = router_.route(M.origin, M.dest, prof, M.amount);
    double loadedFuel = loaded.found ? loaded.fuel : 0.0;
    struct C {
      int s;
      double lbArr;
    };
    std::vector<C> kept, all;
    for (int s : near) {
      counters().candidatesEvaluated++;
      const Ship& Sh = w_.ships[s];
      double lb = std::max(Sh.availableAt, now_) +
                  (router_.lowerBound(Sh.location, M.origin, PROFILE_TIME) + lbLoaded) / Sh.speed + cfg_.loadH;
      all.push_back({s, lb});
      bool deadlineOk = lb <= M.deadline + cfg_.maxLateH;   // ALT deadline pruning
      bool fuelOk = Sh.fuelCap * 3.0 >= loadedFuel;         // > 3 refuel stops: prune
      if (deadlineOk && fuelOk) kept.push_back({s, lb});
    }
    if (kept.empty()) {  // keep the two most promising ones so nothing becomes impossible by pruning
      std::sort(all.begin(), all.end(), [](const C& a, const C& b) { return a.lbArr < b.lbArr; });
      for (size_t i = 0; i < all.size() && i < 2; ++i) kept.push_back(all[i]);
    }
    // ship dominance among idle ships at the same location
    std::vector<char> dom(kept.size(), 0);
    for (size_t i = 0; i < kept.size(); ++i) {
      const Ship& A = w_.ships[kept[i].s];
      if (!sol_.seq[kept[i].s].empty()) continue;
      for (size_t j = 0; j < kept.size(); ++j) {
        if (i == j || dom[j]) continue;
        const Ship& B = w_.ships[kept[j].s];
        if (!sol_.seq[kept[j].s].empty() || A.location != B.location) continue;
        bool aDomB = A.availableAt <= B.availableAt && A.cargoCap >= B.cargoCap && A.fuel >= B.fuel &&
                     A.fuelCap >= B.fuelCap && A.reliability >= B.reliability && A.opCost <= B.opCost &&
                     A.speed >= B.speed && (A.caps & B.caps) == B.caps;
        bool same = A.availableAt == B.availableAt && A.cargoCap == B.cargoCap && A.fuel == B.fuel &&
                    A.fuelCap == B.fuelCap && A.reliability == B.reliability && A.opCost == B.opCost &&
                    A.speed == B.speed && A.caps == B.caps;
        if (aDomB && (!same || kept[i].s < kept[j].s)) {
          dom[j] = 1;
          counters().dominatedShips++;
        }
      }
    }
    cand_[m].clear();
    for (size_t i = 0; i < kept.size() && static_cast<int>(cand_[m].size()) < K; ++i)
      if (!dom[i]) cand_[m].push_back(kept[i].s);
    counters().candidatesPruned += static_cast<uint64_t>(std::max(0, S - static_cast<int>(cand_[m].size())));
  }
}

// ------------------------------------------------------------------ construction
void Scheduler::insertGreedy(const std::vector<int>& missions, const Deadline& dl) {
  std::vector<int> order;
  for (int m : missions)
    if (isActive(m) && sol_.shipOf[m] < 0) order.push_back(m);
  std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
    const Mission &A = w_.missions[a], &B = w_.missions[b];
    if (A.priority != B.priority) return A.priority > B.priority;
    return A.deadline < B.deadline;
  });
  for (int m : order) {
    if (dl.expired() || workExceeded()) break;
    Ins ins = bestInsertion(m, nullptr);
    if (ins.ship >= 0 && ins.delta < rejectPenalty(m)) applyInsert(m, ins.ship, ins.pos);
  }
}

void Scheduler::insertRegret(const std::vector<int>& missions, int k, const Deadline& dl) {
  struct Entry {
    std::vector<double> best;  // per candidate ship: best delta
    double regret = 0, top = INF;
    uint32_t ver = 0;
    bool done = false;
  };
  std::vector<int> pool;
  for (int m : missions)
    if (isActive(m) && sol_.shipOf[m] < 0) pool.push_back(m);
  if (pool.empty()) return;
  if (static_cast<int>(pool.size()) > std::max(1, cfg_.regretBatch)) {
    // anytime: deadline-ordered batches so an interrupted construction still inserted the urgent ones
    std::stable_sort(pool.begin(), pool.end(),
                     [&](int a, int b) { return w_.missions[a].deadline < w_.missions[b].deadline; });
    for (size_t b = 0; b < pool.size() && !dl.expired() && !workExceeded(); b += static_cast<size_t>(cfg_.regretBatch)) {
      std::vector<int> batch(pool.begin() + static_cast<long>(b),
                             pool.begin() + static_cast<long>(std::min(pool.size(), b + cfg_.regretBatch)));
      int keep = cfg_.regretBatch;
      cfg_.regretBatch = static_cast<int>(batch.size()) + 1;
      insertRegret(batch, k, dl);
      cfg_.regretBatch = keep;
    }
    return;
  }
  std::unordered_map<int, int> idx;
  std::vector<Entry> E(pool.size());
  std::unordered_map<int, std::vector<int>> byShip;  // ship -> pool indices listing it
  auto score = [&](size_t i) {
    Entry& e = E[i];
    std::vector<double> v;
    for (double d : e.best)
      if (std::isfinite(d)) v.push_back(d);
    std::sort(v.begin(), v.end());
    double rp = rejectPenalty(pool[i]);
    e.top = v.empty() ? INF : v[0];
    double reg = 0;
    for (int j = 1; j < k; ++j) reg += (j < static_cast<int>(v.size()) ? std::min(v[j], rp) : rp) - std::min(e.top, rp);
    e.regret = reg;
  };
  using HItem = std::tuple<double, int, uint32_t, int>;  // regret, priority, version, pool index
  std::priority_queue<HItem> heap;
  for (size_t i = 0; i < pool.size(); ++i) {
    if (dl.expired() || workExceeded()) return;
    int m = pool[i];
    idx[m] = static_cast<int>(i);
    bestInsertion(m, &E[i].best);
    const auto& c = candidates(m);
    for (int s : c) byShip[s].push_back(static_cast<int>(i));
    score(i);
    heap.push({E[i].regret, w_.missions[m].priority, E[i].ver, static_cast<int>(i)});
  }
  while (!heap.empty()) {
    if (dl.expired() || workExceeded()) break;
    auto [reg, pr, ver, i] = heap.top();
    heap.pop();
    (void)reg;
    (void)pr;
    Entry& e = E[i];
    if (e.done || ver != e.ver) continue;
    e.done = true;
    int m = pool[i];
    if (!(e.top < rejectPenalty(m))) continue;  // inserting costs more than rejecting
    // locate the best ship/position again (only the stored best ship needs exact position)
    const auto& c = candidates(m);
    int bestShip = -1;
    double bd = INF;
    for (size_t j = 0; j < c.size(); ++j)
      if (e.best[j] < bd) { bd = e.best[j]; bestShip = c[j]; }
    if (bestShip < 0) continue;
    int bestPos = -1;
    double bestD = INF;
    for (int p : positionsFor(m, bestShip)) {
      double d = insertionDelta(m, bestShip, p);
      if (d < bestD) { bestD = d; bestPos = p; }
    }
    if (bestPos < 0 || !(bestD < rejectPenalty(m))) continue;
    applyInsert(m, bestShip, bestPos);
    // only missions that list this ship need an update
    auto it = byShip.find(bestShip);
    if (it == byShip.end()) continue;
    for (int j : it->second) {
      Entry& f = E[j];
      if (f.done) continue;
      int x = pool[j];
      const auto& cx = candidates(x);
      for (size_t a = 0; a < cx.size(); ++a) {
        if (cx[a] != bestShip) continue;
        double bdx = INF;
        if (shipEnabled(bestShip) && capable(bestShip, x))
          for (int p : positionsFor(x, bestShip)) bdx = std::min(bdx, insertionDelta(x, bestShip, p));
        f.best[a] = bdx;
      }
      score(j);
      ++f.ver;
      heap.push({f.regret, w_.missions[x].priority, f.ver, j});
    }
  }
}

// ------------------------------------------------------------------ ALNS operators
double Scheduler::missionContribution(int m) {
  int s = sol_.shipOf[m];
  if (s < 0) return 0;
  tmp_ = sol_.seq[s];
  tmp_.erase(std::remove(tmp_.begin(), tmp_.end(), m), tmp_.end());
  return sol_.shipCost[s] - (tmp_.empty() ? 0.0 : evaluateShip(s, tmp_, nullptr));
}

double Scheduler::relatedness(int a, int b) const {
  const Mission &A = w_.missions[a], &B = w_.missions[b];
  const Planet &oa = w_.planets[A.origin], &ob = w_.planets[B.origin];
  const Planet &da = w_.planets[A.dest], &db = w_.planets[B.dest];
  double d1 = std::hypot(oa.x - ob.x, oa.y - ob.y), d2 = std::hypot(da.x - db.x, da.y - db.y);
  double r = (d1 + d2) / scale_ + std::fabs(A.deadline - B.deadline) / 240.0 + (A.resource != B.resource ? 0.3 : 0.0);
  if (sol_.shipOf[a] >= 0 && sol_.shipOf[a] == sol_.shipOf[b]) r -= 0.2;
  return r;
}

std::vector<int> Scheduler::related(const std::vector<int>& seeds, int count, const std::vector<int>* scope) {
  std::vector<int> out;
  if (seeds.empty() || count <= 0) return out;
  std::unordered_set<int> seedSet(seeds.begin(), seeds.end());
  const std::vector<int>& base = scope ? *scope : active_;
  std::vector<int> sample;
  if (base.size() <= 600) sample = base;
  else
    for (int i = 0; i < 600; ++i) sample.push_back(base[rng_.uniformInt(static_cast<int>(base.size()))]);
  std::vector<std::pair<double, int>> sc;
  std::unordered_set<int> seen;
  for (int x : sample) {
    if (seedSet.count(x) || !seen.insert(x).second || !isActive(x)) continue;
    double best = INF;
    for (size_t i = 0; i < seeds.size() && i < 64; ++i) best = std::min(best, relatedness(seeds[i], x));
    sc.push_back({best, x});
  }
  std::sort(sc.begin(), sc.end());
  for (size_t i = 0; i < sc.size() && static_cast<int>(out.size()) < count; ++i) out.push_back(sc[i].second);
  return out;
}

std::vector<int> Scheduler::destroy(int op, int q, const std::vector<int>& pool) {
  std::vector<int> assigned;
  for (int m : pool)
    if (sol_.shipOf[m] >= 0) assigned.push_back(m);
  std::vector<int> rem;
  if (assigned.empty()) return rem;
  q = std::min<int>(q, static_cast<int>(assigned.size()));
  auto sampleOf = [&](int n) {
    std::vector<int> s;
    if (static_cast<int>(assigned.size()) <= n) return assigned;
    for (int i = 0; i < n; ++i) s.push_back(assigned[rng_.uniformInt(static_cast<int>(assigned.size()))]);
    std::sort(s.begin(), s.end());
    s.erase(std::unique(s.begin(), s.end()), s.end());
    return s;
  };
  switch (op) {
    case 0: {  // random
      std::vector<int> a = assigned;
      for (int i = 0; i < q; ++i) {
        int j = i + rng_.uniformInt(static_cast<int>(a.size()) - i);
        std::swap(a[i], a[j]);
        rem.push_back(a[i]);
      }
      break;
    }
    case 1: {  // worst cost (randomised, power 3)
      std::vector<int> s = sampleOf(std::max(4 * q, 16));
      std::vector<std::pair<double, int>> c;
      for (int m : s) c.push_back({-missionContribution(m), m});
      std::sort(c.begin(), c.end());
      std::vector<char> used(c.size(), 0);
      for (int i = 0; i < q && i < static_cast<int>(c.size()); ++i) {
        int j = static_cast<int>(std::pow(rng_.uniform(), 3.0) * c.size());
        while (j < static_cast<int>(c.size()) && used[j]) ++j;
        if (j >= static_cast<int>(c.size())) {
          j = 0;
          while (used[j]) ++j;
        }
        used[j] = 1;
        rem.push_back(c[j].second);
      }
      break;
    }
    case 2: {  // Shaw / related
      int seed = assigned[rng_.uniformInt(static_cast<int>(assigned.size()))];
      rem.push_back(seed);
      std::vector<int> rel = related({seed}, q - 1, &assigned);
      rem.insert(rem.end(), rel.begin(), rel.end());
      break;
    }
    case 3: {  // deadline-urgent: largest weighted lateness / smallest slack
      std::vector<int> s = sampleOf(std::max(4 * q, 16));
      std::unordered_map<int, ShipEval> evs;
      std::vector<std::pair<double, int>> c;
      for (int m : s) {
        int sh = sol_.shipOf[m];
        auto it = evs.find(sh);
        if (it == evs.end()) {
          ShipEval ev;
          evaluateShip(sh, sol_.seq[sh], &ev);
          it = evs.emplace(sh, std::move(ev)).first;
        }
        for (const auto& item : it->second.items)
          if (item.mission == m) c.push_back({-(item.latePen + item.chancePen + 1e-3 * (item.arrive - w_.missions[m].deadline)), m});
      }
      std::sort(c.begin(), c.end());
      for (int i = 0; i < q && i < static_cast<int>(c.size()); ++i) rem.push_back(c[i].second);
      break;
    }
    default: {  // dependency-guided: a ship's missions + missions that list that ship as a candidate
      int seed = assigned[rng_.uniformInt(static_cast<int>(assigned.size()))];
      int sh = sol_.shipOf[seed];
      for (int m : sol_.seq[sh])
        if (static_cast<int>(rem.size()) < q && poolFlag_[m]) rem.push_back(m);
      for (int m : assigned) {
        if (static_cast<int>(rem.size()) >= q) break;
        if (sol_.shipOf[m] == sh) continue;
        const auto& c = candidates(m);
        if (std::find(c.begin(), c.end(), sh) != c.end()) rem.push_back(m);
      }
      break;
    }
  }
  std::sort(rem.begin(), rem.end());
  rem.erase(std::unique(rem.begin(), rem.end()), rem.end());
  for (int m : rem) applyRemove(m);
  return rem;
}

void Scheduler::repairOp(int op, std::vector<int>& removed) {
  // also give some currently rejected pool missions a chance
  std::vector<int> un;
  for (int m : active_)
    if (poolFlag_[m] && sol_.shipOf[m] < 0 && std::find(removed.begin(), removed.end(), m) == removed.end())
      un.push_back(m);
  int extra = std::min<int>(static_cast<int>(un.size()), std::max(1, static_cast<int>(removed.size()) / 2));
  for (int i = 0; i < extra; ++i) {
    int j = i + rng_.uniformInt(static_cast<int>(un.size()) - i);
    std::swap(un[i], un[j]);
    removed.push_back(un[i]);
  }
  Deadline never = Deadline::never();
  if (op == 0) {
    for (size_t i = removed.size(); i > 1; --i) std::swap(removed[i - 1], removed[rng_.uniformInt(static_cast<int>(i))]);
    insertGreedy(removed, never);
  } else {
    insertRegret(removed, op == 1 ? 2 : 3, never);
  }
}

int64_t Scheduler::iterationsForBudget(double ms, size_t scope) const {
  double q = std::min<double>(cfg_.destroyMax, std::max<double>(cfg_.destroyMin, cfg_.destroyFrac * scope));
  int alive = 0;
  for (const auto& s : w_.ships) alive += s.alive ? 1 : 0;
  double avgLen = static_cast<double>(active_.size()) / std::max(1, alive);  // work per evaluation ~ len + 1
  double perIter = std::max(1.0, q * 1.5 * cfg_.kNearest * (2 * cfg_.beam + 2) * (1.0 + avgLen));
  double it = ms * EVALS_PER_MS / perIter;
  return static_cast<int64_t>(std::min(20000.0, std::max(5.0, it)));
}

int64_t Scheduler::runALNS(int64_t iterations, const Deadline& dl, const std::vector<int>* scope,
                           std::vector<ConvergencePoint>* trace) {
  ensureSizes();
  std::vector<int> pool;
  for (int m : (scope ? *scope : active_))
    if (isActive(m)) pool.push_back(m);
  if (pool.empty() || iterations <= 0) return 0;
  for (int m : pool) poolFlag_[m] = 1;
  Timer timer;
  startSnapshots();
  double cur = sol_.total;
  double T0 = std::max(1e-6, 0.05 * std::max(1.0, cur) / std::log(2.0));
  int64_t it = 0;
  for (; it < iterations; ++it) {
    if (((it & 3) == 0 && dl.expired()) || workExceeded(0.85)) break;  // 15% left for local search
    int d = roulette(dW_, rng_), r = roulette(rW_, rng_);
    int q = static_cast<int>(std::lround(cfg_.destroyFrac * pool.size() * rng_.uniform(0.5, 1.5)));
    q = std::max(cfg_.destroyMin, std::min(cfg_.destroyMax, q));
    beginUndo();
    std::vector<int> removed = destroy(d, q, pool);
    repairOp(r, removed);
    double nc = sol_.total;
    // cooling follows the larger of iteration progress and work progress (work-capped runs)
    double prog = std::max(static_cast<double>(it) / static_cast<double>(iterations), std::min(1.0, workProgress() / 0.85));
    double T = T0 * std::pow(0.002, prog);
    bool accept = nc < cur - 1e-9 || rng_.uniform() < std::exp(-(nc - cur) / T);
    double score = 0;
    if (nc < bestTotal_ - 1e-9) score = 33;
    else if (nc < cur - 1e-9) score = 9;
    else if (accept) score = 13;
    if (accept) {
      recording_ = false;
      cur = nc;
      if (nc < bestTotal_ - 1e-9) captureBest();
    } else {
      rollback();
    }
    dS_[d] += score; dN_[d]++;
    rS_[r] += score; rN_[r]++;
    if ((it + 1) % cfg_.segment == 0) {
      for (size_t i = 0; i < dW_.size(); ++i) {
        if (dN_[i]) dW_[i] = std::max(0.05, (1 - cfg_.reaction) * dW_[i] + cfg_.reaction * dS_[i] / dN_[i]);
        dS_[i] = 0; dN_[i] = 0;
      }
      for (size_t i = 0; i < rW_.size(); ++i) {
        if (rN_[i]) rW_[i] = std::max(0.05, (1 - cfg_.reaction) * rW_[i] + cfg_.reaction * rS_[i] / rN_[i]);
        rS_[i] = 0; rN_[i] = 0;
      }
    }
    if (trace && (score == 33 || (it + 1) % cfg_.segment == 0) && trace->size() < 2000)
      trace->push_back({timer.ms(), it + 1, bestTotal_, cur});
  }
  restoreBest();
  stopSnapshots();
  for (int m : pool) poolFlag_[m] = 0;
  return it;
}

int Scheduler::localSearch(const Deadline& dl, const std::vector<int>* scope) {
  ensureSizes();
  std::vector<int> pool;
  for (int m : (scope ? *scope : active_))
    if (isActive(m)) pool.push_back(m);
  int improved = 0;
  for (int pass = 0; pass < 3; ++pass) {
    int before = improved;
    // relocate
    for (int m : pool) {
      if (dl.expired() || workExceeded()) return improved;
      if (sol_.shipOf[m] < 0) continue;
      beginUndo();
      double old = sol_.total;
      applyRemove(m);
      Ins ins = bestInsertion(m, nullptr);
      if (ins.ship >= 0 && ins.delta < rejectPenalty(m)) applyInsert(m, ins.ship, ins.pos);
      if (sol_.total < old - 1e-6) { recording_ = false; ++improved; }
      else rollback();
    }
    // swap between ships
    for (int m : pool) {
      if (dl.expired() || workExceeded()) return improved;
      int s1 = sol_.shipOf[m];
      if (s1 < 0) continue;
      int tries = 0;
      for (int s2 : candidates(m)) {
        if (s2 == s1 || !shipEnabled(s2) || sol_.seq[s2].empty() || tries >= 3) continue;
        ++tries;
        int x = sol_.seq[s2][rng_.uniformInt(static_cast<int>(sol_.seq[s2].size()))];
        if (!capable(s2, m) || !capable(s1, x)) continue;
        beginUndo();
        double old = sol_.total;
        std::vector<int> q1 = sol_.seq[s1], q2 = sol_.seq[s2];
        std::replace(q1.begin(), q1.end(), m, x);
        std::replace(q2.begin(), q2.end(), x, m);
        setSequence(s1, q1);
        setSequence(s2, q2);
        counters().candidatesEvaluated += 2;
        if (sol_.total < old - 1e-6) { recording_ = false; ++improved; break; }
        rollback();
      }
    }
    // 2-opt*: exchange sequence tails between two ships
    for (int m : pool) {
      if (dl.expired() || workExceeded()) return improved;
      int s1 = sol_.shipOf[m];
      if (s1 < 0) continue;
      const auto& seq1 = sol_.seq[s1];
      int i = static_cast<int>(std::find(seq1.begin(), seq1.end(), m) - seq1.begin());
      int tries = 0;
      for (int s2 : candidates(m)) {
        if (s2 == s1 || !shipEnabled(s2) || tries >= 2) continue;
        ++tries;
        const auto& seq2 = sol_.seq[s2];
        int L2 = static_cast<int>(seq2.size());
        bool done = false;
        for (int j = std::max(0, L2 - 3); j <= L2 && !done; ++j) {
          std::vector<int> n1(sol_.seq[s1].begin(), sol_.seq[s1].begin() + i);
          n1.insert(n1.end(), sol_.seq[s2].begin() + j, sol_.seq[s2].end());
          std::vector<int> n2(sol_.seq[s2].begin(), sol_.seq[s2].begin() + j);
          n2.insert(n2.end(), sol_.seq[s1].begin() + i, sol_.seq[s1].end());
          bool ok = true;
          for (int x : n1) ok = ok && capable(s1, x);
          for (int x : n2) ok = ok && capable(s2, x);
          if (!ok) continue;
          beginUndo();
          double old = sol_.total;
          setSequence(s1, n1);
          setSequence(s2, n2);
          counters().candidatesEvaluated += 2;
          if (sol_.total < old - 1e-6) { recording_ = false; ++improved; done = true; }
          else rollback();
        }
        if (done) break;
      }
    }
    if (improved == before) break;
  }
  return improved;
}

// ------------------------------------------------------------------ baselines
void Scheduler::solveGA(int generations, const Deadline& dl, std::vector<ConvergencePoint>* trace) {
  ensureSizes();
  std::vector<int> pool = active_;
  int M = static_cast<int>(pool.size());
  if (M == 0) return;
  Timer timer;
  auto decode = [&](const std::vector<int>& perm) {
    for (int m : pool) applyRemove(m);
    for (int m : perm) {
      Ins ins = bestInsertion(m, nullptr, true);
      if (ins.ship >= 0 && ins.delta < rejectPenalty(m)) applyInsert(m, ins.ship, ins.pos);
    }
    return sol_.total;
  };
  int P = std::max(4, cfg_.gaPopulation);
  std::vector<std::vector<int>> pop(P, pool);
  std::stable_sort(pop[0].begin(), pop[0].end(),
                   [&](int a, int b) { return w_.missions[a].deadline < w_.missions[b].deadline; });
  for (int i = 1; i < P; ++i)
    for (int j = M - 1; j > 0; --j) std::swap(pop[i][j], pop[i][rng_.uniformInt(j + 1)]);
  std::vector<double> fit(P);
  Solution best;
  double bestCost = INF;
  for (int i = 0; i < P; ++i) {
    fit[i] = decode(pop[i]);
    if (fit[i] < bestCost) { bestCost = fit[i]; best = sol_; }
    if (dl.expired()) break;
  }
  auto tournament = [&]() {
    int b = rng_.uniformInt(P);
    for (int k = 0; k < 2; ++k) {
      int c = rng_.uniformInt(P);
      if (fit[c] < fit[b]) b = c;
    }
    return b;
  };
  for (int g = 0; g < generations && !dl.expired() && !workExceeded(); ++g) {
    std::vector<int> order(P);
    for (int i = 0; i < P; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) { return fit[a] < fit[b]; });
    std::vector<std::vector<int>> next = {pop[order[0]], pop[order[1]]};
    std::vector<double> nfit = {fit[order[0]], fit[order[1]]};
    while (static_cast<int>(next.size()) < P && !dl.expired()) {
      const auto &A = pop[tournament()], &B = pop[tournament()];
      // order crossover (OX)
      int a = rng_.uniformInt(M), b = rng_.uniformInt(M);
      if (a > b) std::swap(a, b);
      std::vector<int> child(M, -1);
      std::unordered_set<int> used;
      for (int i = a; i <= b; ++i) { child[i] = A[i]; used.insert(A[i]); }
      int pos = (b + 1) % M;
      for (int k = 0; k < M; ++k) {
        int x = B[(b + 1 + k) % M];
        if (used.count(x)) continue;
        child[pos] = x;
        pos = (pos + 1) % M;
      }
      if (rng_.uniform() < 0.3) std::swap(child[rng_.uniformInt(M)], child[rng_.uniformInt(M)]);
      if (rng_.uniform() < 0.1) {
        int x = rng_.uniformInt(M), y = rng_.uniformInt(M);
        if (x > y) std::swap(x, y);
        std::reverse(child.begin() + x, child.begin() + y + 1);
      }
      double f = decode(child);
      counters().statesExplored++;
      if (f < bestCost) { bestCost = f; best = sol_; }
      next.push_back(std::move(child));
      nfit.push_back(f);
    }
    if (static_cast<int>(next.size()) < P) break;
    pop.swap(next);
    fit.swap(nfit);
    if (trace && trace->size() < 2000) trace->push_back({timer.ms(), g + 1, bestCost, fit[0]});
  }
  if (std::isfinite(bestCost)) restore(best);
}

void Scheduler::solveSA(int64_t iterations, const Deadline& dl, std::vector<ConvergencePoint>* trace) {
  ensureSizes();
  if (active_.empty()) return;
  if (assignedCount() == 0) insertGreedy(active_, dl);
  Timer timer;
  startSnapshots();
  double cur = sol_.total;
  double T0 = std::max(1e-6, 0.02 * std::max(1.0, cur));
  for (int64_t it = 0; it < iterations; ++it) {
    if (((it & 15) == 0 && dl.expired()) || workExceeded()) break;
    double T = T0 * std::pow(0.001, std::max(static_cast<double>(it) / static_cast<double>(iterations),
                                             std::min(1.0, workProgress())));
    beginUndo();
    int m = active_[rng_.uniformInt(static_cast<int>(active_.size()))];
    if (rng_.uniform() < 0.7 || sol_.shipOf[m] < 0) {  // relocate
      const auto& c = candidates(m);
      if (c.empty()) { rollback(); continue; }
      int s = c[rng_.uniformInt(static_cast<int>(c.size()))];
      if (!shipEnabled(s) || !capable(s, m)) { rollback(); continue; }
      applyRemove(m);
      if (rng_.uniform() < 0.1) { /* leave rejected */ }
      else applyInsert(m, s, rng_.uniformInt(static_cast<int>(sol_.seq[s].size()) + 1));
    } else {  // swap with a random mission of a candidate ship
      int s1 = sol_.shipOf[m];
      const auto& c = candidates(m);
      if (c.empty()) { rollback(); continue; }
      int s2 = c[rng_.uniformInt(static_cast<int>(c.size()))];
      if (s2 == s1 || sol_.seq[s2].empty()) { rollback(); continue; }
      int x = sol_.seq[s2][rng_.uniformInt(static_cast<int>(sol_.seq[s2].size()))];
      if (!capable(s2, m) || !capable(s1, x)) { rollback(); continue; }
      std::vector<int> q1 = sol_.seq[s1], q2 = sol_.seq[s2];
      std::replace(q1.begin(), q1.end(), m, x);
      std::replace(q2.begin(), q2.end(), x, m);
      setSequence(s1, q1);
      setSequence(s2, q2);
    }
    counters().candidatesEvaluated++;
    double nc = sol_.total;
    if (nc < cur - 1e-9 || rng_.uniform() < std::exp(-(nc - cur) / T)) {
      recording_ = false;
      cur = nc;
      if (nc < bestTotal_ - 1e-9) {
        captureBest();
        if (trace && trace->size() < 2000) trace->push_back({timer.ms(), it + 1, bestTotal_, cur});
      }
    } else {
      rollback();
    }
  }
  restoreBest();
  stopSnapshots();
}

void Scheduler::bnbRecurse(size_t k) {
  ++bnbNodes_;
  counters().bnbNodes++;
  counters().statesExplored++;
  if (bnbNodes_ > bnbLimit_ || ((bnbNodes_ & 255) == 0 && bnbDl_.expired())) {
    bnbAborted_ = true;
    return;
  }
  double cur = sol_.total - bnbSR_[k];  // cost of decided missions (ships + rejections)
  if (k == bnbOrder_.size()) {
    if (cur < bnbBest_ - 1e-9) {
      bnbBest_ = cur;
      bnbBestSol_ = sol_;
    }
    return;
  }
  if (cur + bnbSuffix_[k] >= bnbBest_ - 1e-9) return;  // bound
  int m = bnbOrder_[k];
  struct Child {
    double delta;
    int ship, pos;
  };
  std::vector<Child> ch;
  for (int s : bnbShips_) {
    if (!capable(s, m)) continue;
    int L = static_cast<int>(sol_.seq[s].size());
    for (int p = 0; p <= L; ++p) ch.push_back({insertionDelta(m, s, p), s, p});
  }
  ch.push_back({rejectPenalty(m), -1, -1});
  std::sort(ch.begin(), ch.end(), [](const Child& a, const Child& b) { return a.delta < b.delta; });
  for (const Child& c : ch) {
    if (bnbAborted_) return;
    if (cur + c.delta + bnbSuffix_[k + 1] >= bnbBest_ - 1e-9) break;  // children sorted: rest is worse
    if (c.ship < 0) {
      bnbRecurse(k + 1);
    } else {
      applyInsert(m, c.ship, c.pos);
      bnbRecurse(k + 1);
      applyRemove(m);
    }
  }
}

bool Scheduler::solveBranchAndBound(uint64_t nodeLimit, const Deadline& dl) {
  ensureSizes();
  std::vector<int> pool = active_;
  if (pool.empty()) return true;
  // ship set: every enabled ship for small fleets (exact over the whole problem); for large fleets
  // the union of the missions' candidate ships (exact over that restricted set)
  bnbShips_.clear();
  for (size_t s = 0; s < w_.ships.size(); ++s)
    if (shipEnabled(static_cast<int>(s))) bnbShips_.push_back(static_cast<int>(s));
  bool pr = cfg_.usePruning;
  if (bnbShips_.size() <= 40) cfg_.usePruning = false;  // candidates = all capable ships
  buildCandidates(pool);
  if (bnbShips_.size() > 40) {
    std::vector<char> inSet(w_.ships.size(), 0);
    bnbShips_.clear();
    for (int m : pool)
      for (int s : candidates(m))
        if (!inSet[s] && shipEnabled(s)) {
          inSet[s] = 1;
          bnbShips_.push_back(s);
        }
    std::sort(bnbShips_.begin(), bnbShips_.end());
  }
  // incumbent (upper bound): regret insertion + local search over the same candidates
  unassignAll();
  insertRegret(pool, 2, dl);
  localSearch(dl, nullptr);
  bnbBest_ = sol_.total;
  bnbBestSol_ = sol_;
  unassignAll();
  // order and per-mission lower bounds on the marginal contribution
  bnbOrder_ = pool;
  std::stable_sort(bnbOrder_.begin(), bnbOrder_.end(),
                   [&](int a, int b) { return w_.missions[a].deadline < w_.missions[b].deadline; });
  size_t n = bnbOrder_.size();
  bnbMin_.assign(n, 0);
  bnbSuffix_.assign(n + 1, 0);
  bnbSR_.assign(n + 1, 0);
  for (size_t i = 0; i < n; ++i) {
    int m = bnbOrder_[i];
    const Mission& M = w_.missions[m];
    // Lower bound on the marginal contribution of m in ANY completion. The loaded leg may replace
    // part of an empty leg of the successor (triangle inequality), so only the part of its cost that
    // exceeds the discounted Cargo cost of the same trip is certain; travel time can be fully absorbed.
    double lb = rejectPenalty(m);
    const Path& pl = router_.route(M.origin, M.dest, profileFor(m), M.amount);
    double loadedCost = pl.found ? pl.cost : INF;
    const Path& pc = router_.route(M.origin, M.dest, PROFILE_CARGO, 0.0);
    double cargoCost = pc.found ? pc.cost : 0.0;
    for (int s : bnbShips_) {
      if (!capable(s, m) || !std::isfinite(loadedCost)) continue;
      const Ship& S = w_.ships[s];
      double c = std::max(0.0, loadedCost - cfg_.emptyLegPlanWeight * cfg_.emptyLegFactor * cargoCost) +
                 (1.0 - S.reliability) * M.value();
      lb = std::min(lb, c);
    }
    bnbMin_[i] = lb;
  }
  for (size_t i = n; i-- > 0;) {
    bnbSuffix_[i] = bnbSuffix_[i + 1] + bnbMin_[i];
    bnbSR_[i] = bnbSR_[i + 1] + rejectPenalty(bnbOrder_[i]);
  }
  bnbNodes_ = 0;
  bnbLimit_ = nodeLimit;
  bnbDl_ = dl;
  bnbAborted_ = false;
  bnbRecurse(0);
  restore(bnbBestSol_);
  cfg_.usePruning = pr;
  return !bnbAborted_;
}

// ------------------------------------------------------------------ repair / validation / output
int Scheduler::repairInvalid(const Deadline& dl) {
  std::vector<int> bad;
  for (size_t s = 0; s < w_.ships.size(); ++s) {
    if (sol_.seq[s].empty()) continue;
    if (!shipEnabled(static_cast<int>(s))) {
      bad.insert(bad.end(), sol_.seq[s].begin(), sol_.seq[s].end());
      continue;
    }
    ShipEval ev;
    evaluateShip(static_cast<int>(s), sol_.seq[s], &ev);
    for (const auto& it : ev.items)
      if (!it.feasible) bad.push_back(it.mission);
  }
  if (bad.empty()) return 0;
  remove(bad);
  insertGreedy(bad, dl);
  return static_cast<int>(bad.size());
}

ValidationReport Scheduler::validate() {
  ValidationReport rep;
  std::vector<int> count(w_.missions.size(), 0);
  auto fail = [&](const std::string& msg) {
    rep.hardViolations++;
    if (rep.messages.size() < 50) rep.messages.push_back(msg);
  };
  for (size_t si = 0; si < w_.ships.size(); ++si) {
    int s = static_cast<int>(si);
    const auto& q = sol_.seq[s];
    if (q.empty()) continue;
    const Ship& S = w_.ships[s];
    if (!shipEnabled(s)) fail("ship " + std::to_string(S.extId) + " unavailable but scheduled");
    ShipEval ev;
    evaluateShip(s, q, &ev);
    double fuel = S.fuel;
    for (const auto& it : ev.items) {
      rep.checked++;
      const Mission& M = w_.missions[it.mission];
      count[it.mission]++;
      std::string id = "mission " + std::to_string(M.extId);
      if (sol_.shipOf[it.mission] != s) fail(id + " ship index mismatch");
      if (!it.feasible) { fail(id + " infeasible (route/fuel/capability)"); continue; }
      if (M.amount > S.cargoCap + 1e-9) fail(id + " exceeds cargo capacity");
      if ((S.caps & M.requiredCaps) != M.requiredCaps) fail(id + " missing capability");
      if (it.start + 1e-9 < std::max(S.availableAt, now_)) fail(id + " departs before ship availability");
      if (it.depart + 1e-9 < M.release) fail(id + " departs before release");
      for (EdgeId e : it.emptyPath)
        if (!w_.graph.available(e)) fail(id + " uses unavailable route");
      for (EdgeId e : it.loadedPath) {
        if (!w_.graph.available(e)) fail(id + " uses unavailable route");
        if (w_.graph.cap(e) + 1e-9 < M.amount) fail(id + " exceeds route capacity");
      }
      // fuel simulation along both legs with the planned refuel stops
      auto walk = [&](const std::vector<EdgeId>& path, NodeId start, size_t r0, size_t r1) {
        size_t ri = r0;
        auto maybeRefuel = [&](NodeId x) {
          while (ri < r1 && it.refuels[ri] == x) {
            fuel = S.fuelCap;
            ++ri;
          }
        };
        maybeRefuel(start);
        for (EdgeId e : path) {
          fuel -= w_.graph.fuel(e);
          if (fuel < -1e-6) fail(id + " runs out of fuel");
          maybeRefuel(w_.graph.to(e));
        }
      };
      size_t split = std::min<size_t>(static_cast<size_t>(it.refuelSplit), it.refuels.size());
      NodeId startLoc = it.emptyPath.empty() ? M.origin : w_.graph.from(it.emptyPath.front());
      walk(it.emptyPath, startLoc, 0, split);
      walk(it.loadedPath, M.origin, split, it.refuels.size());
      if (it.arrive > M.deadline + 1e-9) rep.lateMissions++;
    }
  }
  for (size_t m = 0; m < count.size(); ++m)
    if (count[m] > 1) fail("mission " + std::to_string(w_.missions[m].extId) + " assigned twice");
  return rep;
}

void Scheduler::extractPlans(const std::vector<int>* ships) {
  std::vector<int> list;
  if (ships) list = *ships;
  else
    for (size_t s = 0; s < w_.ships.size(); ++s) list.push_back(static_cast<int>(s));
  for (int s : list) {
    if (s < 0 || s >= static_cast<int>(w_.ships.size()) || sol_.seq[s].empty()) continue;
    ShipEval ev;
    evaluateShip(s, sol_.seq[s], &ev);
    for (size_t i = 0; i < ev.items.size(); ++i) {
      auto& it = ev.items[i];
      Mission& M = w_.missions[it.mission];
      MissionPlan& P = M.plan;
      P.ship = s;
      P.seqIndex = static_cast<int>(i);
      P.plannedStart = it.start;
      P.arriveOrigin = it.arriveOrigin;
      P.depart = it.depart;
      P.arrive = it.arrive;
      P.sigma = it.sigma;
      P.pOnTime = it.pOnTime;
      P.cost = it.cost;
      P.profile = it.profile;
      P.emptyPath = std::move(it.emptyPath);
      P.loadedPath = std::move(it.loadedPath);
      P.refuelNodes = std::move(it.refuels);
      P.version++;
      P.valid = it.feasible;
      if (M.status == MissionStatus::Pending) M.status = MissionStatus::Assigned;
    }
  }
  // unassigned missions lose their plan: all active ones on a full extraction, otherwise only
  // the ones unassigned since the last extraction (O(changes) instead of O(active))
  auto invalidate = [&](int m) {
    if (!isActive(m) || sol_.shipOf[m] >= 0) return;
    Mission& M = w_.missions[m];
    if (M.plan.valid || M.plan.ship >= 0) {
      M.plan.valid = false;
      M.plan.ship = -1;
      M.plan.version++;
    }
    if (M.status == MissionStatus::Assigned) M.status = MissionStatus::Pending;
  };
  if (!ships)
    for (int m : active_) invalidate(m);
  for (int m : unassignedLog_) invalidate(m);
  unassignedLog_.clear();
}

std::string Scheduler::lastOperatorStats() const {
  std::ostringstream o;
  o << "destroy{";
  for (size_t i = 0; i < dW_.size(); ++i) o << (i ? "," : "") << DESTROY_NAMES[i] << ":" << fmtDouble(dW_[i], 2);
  o << "} repair{";
  for (size_t i = 0; i < rW_.size(); ++i) o << (i ? "," : "") << REPAIR_NAMES[i] << ":" << fmtDouble(rW_[i], 2);
  o << "}";
  return o.str();
}

}  // namespace chronos
