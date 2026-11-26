// replan.cpp - event application and Dependency-Guided Incremental ALNS.
// Invalid-set lookup: O(index entries of the touched edges/ship/planet).
// Incremental repair: O(|N| * K * P * L) per ALNS iteration where N = invalid set + Shaw neighbours.
#include "chronos/replan.hpp"

#include <unordered_set>

namespace chronos {

const char* replanModeName(ReplanMode m) { return m == ReplanMode::Full ? "full" : "incremental"; }

// ------------------------------------------------------------------ dependency index
void DependencyIndex::clear() {
  edge_.clear();
  ship_.clear();
  planet_.clear();
  entries_ = 0;
}

bool DependencyIndex::live(const World& w, const Ref& r) {
  if (r.first < 0 || r.first >= static_cast<int>(w.missions.size())) return false;
  const Mission& M = w.missions[r.first];
  return M.active() && M.plan.valid && M.plan.version == r.second;
}

void DependencyIndex::indexMission(const World& w, int m) {
  if (m < 0 || m >= static_cast<int>(w.missions.size())) return;
  const Mission& M = w.missions[m];
  if (!M.active() || !M.plan.valid) return;
  Ref ref{m, M.plan.version};
  for (EdgeId e : M.plan.emptyPath) edge_[e].push_back(ref);
  for (EdgeId e : M.plan.loadedPath) edge_[e].push_back(ref);
  ship_[M.plan.ship].push_back(ref);
  planet_[M.origin].push_back(ref);
  entries_ += M.plan.emptyPath.size() + M.plan.loadedPath.size() + 2;
}

void DependencyIndex::indexAll(const World& w) {
  clear();
  for (size_t m = 0; m < w.missions.size(); ++m) indexMission(w, static_cast<int>(m));
}

std::vector<int> DependencyIndex::missionsOnEdge(const World& w, EdgeId e) const {
  std::vector<int> out;
  auto it = edge_.find(e);
  if (it == edge_.end()) return out;
  for (const Ref& r : it->second)
    if (live(w, r)) out.push_back(r.first);
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

std::vector<int> DependencyIndex::missionsOfShip(const World& w, int ship) const {
  std::vector<int> out;
  auto it = ship_.find(ship);
  if (it == ship_.end()) return out;
  for (const Ref& r : it->second)
    if (live(w, r) && w.missions[r.first].plan.ship == ship) out.push_back(r.first);
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

std::vector<int> DependencyIndex::missionsFromPlanet(const World& w, NodeId p, int resource) const {
  std::vector<int> out;
  auto it = planet_.find(p);
  if (it == planet_.end()) return out;
  for (const Ref& r : it->second)
    if (live(w, r) && (resource < 0 || w.missions[r.first].resource == resource)) out.push_back(r.first);
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

static int eventResource(const Event& ev) {
  double v = ev.type == EventType::ResourceUnavailable ? ev.a : ev.b;
  int r = static_cast<int>(std::lround(v));
  if (ev.type == EventType::ProductionDrop && v < 0) return -1;
  return (r >= 0 && r < NUM_RESOURCES) ? r : -1;
}

std::vector<int> DependencyIndex::invalidSet(const World& w, const Event& ev) const {
  std::vector<int> out;
  auto add = [&](const std::vector<int>& v) { out.insert(out.end(), v.begin(), v.end()); };
  switch (ev.type) {
    case EventType::WormholeCollapse:
    case EventType::RouteUnavailable:
    case EventType::PirateBlockade:
    case EventType::RouteDanger:
      if (const auto* es = w.edgesOfRoute(ev.target))
        for (EdgeId e : *es) add(missionsOnEdge(w, e));
      break;
    case EventType::ShipDestroyed:
    case EventType::ShipFailure: {
      int s = w.shipOf(ev.target);
      if (s >= 0) add(missionsOfShip(w, s));
      break;
    }
    case EventType::ProductionDrop: {
      int p = w.planetOf(ev.target);
      if (p >= 0) add(missionsFromPlanet(w, p, eventResource(ev)));
      break;
    }
    case EventType::ResourceUnavailable: {
      int r = eventResource(ev);
      if (r < 0) break;
      if (ev.target < 0) {
        for (const auto& kv : planet_) add(missionsFromPlanet(w, kv.first, r));
      } else {
        int p = w.planetOf(ev.target);
        if (p >= 0) add(missionsFromPlanet(w, p, r));
      }
      break;
    }
    default:
      break;
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

size_t DependencyIndex::memoryBytes() const {
  return entries_ * sizeof(Ref) + (edge_.size() + ship_.size() + planet_.size()) * 64;
}

std::vector<int> bruteForceInvalidSet(const World& w, const Event& ev) {
  std::vector<int> out;
  std::unordered_set<EdgeId> edges;
  if (const auto* es = w.edgesOfRoute(ev.target)) edges.insert(es->begin(), es->end());
  int ship = w.shipOf(ev.target), planet = w.planetOf(ev.target), r = eventResource(ev);
  for (size_t m = 0; m < w.missions.size(); ++m) {
    const Mission& M = w.missions[m];
    if (!M.active() || !M.plan.valid) continue;
    bool hit = false;
    switch (ev.type) {
      case EventType::WormholeCollapse:
      case EventType::RouteUnavailable:
      case EventType::PirateBlockade:
      case EventType::RouteDanger:
        for (EdgeId e : M.plan.emptyPath) hit = hit || edges.count(e);
        for (EdgeId e : M.plan.loadedPath) hit = hit || edges.count(e);
        break;
      case EventType::ShipDestroyed:
      case EventType::ShipFailure:
        hit = ship >= 0 && M.plan.ship == ship;
        break;
      case EventType::ProductionDrop:
        hit = planet >= 0 && M.origin == planet && (r < 0 || M.resource == r);
        break;
      case EventType::ResourceUnavailable:
        hit = r >= 0 && M.resource == r && (ev.target < 0 || (planet >= 0 && M.origin == planet));
        break;
      default:
        break;
    }
    if (hit) out.push_back(static_cast<int>(m));
  }
  return out;
}

// ------------------------------------------------------------------ helpers
std::vector<int> materializeShipment(World& w, const Shipment& sh, double now, double chunkCap) {
  std::vector<int> ids;
  if (sh.amount <= 0) return ids;
  int n = static_cast<int>(std::ceil(sh.amount / std::max(1.0, chunkCap) - 1e-9));
  n = std::max(1, n);
  double chunk = sh.amount / n;
  double slack = sh.deadline - now;
  int pri = slack < 48 ? 5 : (slack < 96 ? 4 : 3);
  if (sh.resource == static_cast<int>(Resource::Medicine)) pri = std::min(5, pri + 1);
  double imp = std::min(1.0, std::max(0.05, RESOURCE_IMPORTANCE[sh.resource] * w.planets[sh.to].strategic / 2.0));
  int64_t parent = -1;
  for (int k = 0; k < n; ++k) {
    Mission m;
    m.origin = sh.from;
    m.dest = sh.to;
    m.resource = sh.resource;
    m.amount = std::floor(chunk);
    if (k == n - 1) m.amount = sh.amount - std::floor(chunk) * (n - 1);
    m.release = now;
    m.deadline = sh.deadline;
    m.priority = pri;
    m.importance = imp;
    m.requiredCaps = CAP_CARGO | (sh.resource == static_cast<int>(Resource::Medicine) ? CAP_MEDICAL : 0u);
    m.source = MissionSource::Proactive;
    m.flowKey = sh.id;
    m.parent = parent;
    int id = w.addMission(m);
    if (k == 0 && n > 1) parent = w.missions[id].extId;
    ids.push_back(id);
  }
  return ids;
}

bool assignParetoPath(World& w, Router& r, int m, double z) {
  Mission& M = w.missions[m];
  std::vector<Path> front = r.paretoPaths(M.origin, M.dest, M.amount, 6, 0.05);
  if (front.empty()) return false;
  double minFail = INF;
  for (const auto& p : front) minFail = std::min(minFail, 1.0 - p.reliability * (1.0 - p.risk));
  const Path* best = nullptr;
  double bestT = INF;
  for (const auto& p : front) {
    double fail = 1.0 - p.reliability * (1.0 - p.risk);
    if (fail > 1.5 * minFail + 1e-9) continue;  // safe enough
    double t = p.timeMean + z * std::sqrt(p.timeVar);
    if (t < bestT) { bestT = t; best = &p; }
  }
  if (!best) return false;
  M.fixedPath = best->edges;
  return true;
}

// ------------------------------------------------------------------ replanner
Replanner::Replanner(World& w, Router& r, Scheduler& s, ResourceAllocator& a, Predictor& p, DependencyIndex& d)
    : w_(w), router_(r), sched_(s), alloc_(a), pred_(p), dep_(d) {}

void Replanner::registerShipmentMissions(int shipmentId, const std::vector<int>& missions) {
  auto& v = shipMissions_[shipmentId];
  v.insert(v.end(), missions.begin(), missions.end());
}

std::vector<int> Replanner::missionsOfShipments(const std::vector<int>& ids) const {
  std::vector<int> out;
  for (int id : ids) {
    auto it = shipMissions_.find(id);
    if (it == shipMissions_.end()) continue;
    for (int m : it->second)
      if (w_.missions[m].active()) out.push_back(m);
  }
  return out;
}

void Replanner::reindex(const std::vector<int>* missions) {
  if (!missions) {
    dep_.indexAll(w_);
    return;
  }
  for (int m : *missions) dep_.indexMission(w_, m);
}

std::vector<int> Replanner::applyEvent(const Event& ev, double now, std::vector<Event>& followUps,
                                       std::vector<NodeId>& changedPlanets, std::vector<EdgeId>& changedEdges) {
  std::vector<int> created;
  auto mutateRoute = [&](int64_t rid, const std::function<void(EdgeId)>& fn) {
    const auto* es = w_.edgesOfRoute(rid);
    if (!es) return false;
    for (EdgeId e : *es) {
      auto old = router_.edgeCosts(e);
      fn(e);
      router_.onEdgeChanged(e, old);
      changedEdges.push_back(e);
    }
    return true;
  };
  auto newMission = [&](NodeId o, NodeId d, int r, double amount, double deadline, int pri, double imp,
                        MissionSource src, uint32_t caps) {
    if (o < 0 || d < 0 || o == d || amount <= 0) return;
    int n = std::max(1, static_cast<int>(std::ceil(amount / std::max(1.0, chunkCap_) - 1e-9)));
    for (int k = 0; k < n; ++k) {
      Mission m;
      m.origin = o;
      m.dest = d;
      m.resource = r;
      m.amount = amount / n;
      m.release = now;
      m.deadline = deadline;
      m.priority = pri;
      m.importance = imp;
      m.requiredCaps = caps | CAP_CARGO;
      m.source = src;
      int id = w_.addMission(m);
      if (pri >= 5) assignParetoPath(w_, router_, id, 1.2816);
      created.push_back(id);
    }
  };
  switch (ev.type) {
    case EventType::WormholeCollapse:
      mutateRoute(ev.target, [&](EdgeId e) { w_.graph.setAvailable(e, false); });
      break;
    case EventType::RouteUnavailable:
      if (mutateRoute(ev.target, [&](EdgeId e) { w_.graph.setAvailable(e, false); }) && ev.a > 0) {
        Event f = ev;
        f.type = EventType::RouteRestore;
        f.time = now + ev.a;
        f.severity = 0;
        followUps.push_back(f);
      }
      break;
    case EventType::PirateBlockade:
      if (mutateRoute(ev.target, [&](EdgeId e) { w_.graph.setAvailable(e, false); })) {
        Event f = ev;
        f.type = EventType::RouteRestore;
        f.time = now + (ev.a > 0 ? ev.a : 48.0);
        f.severity = ev.severity;
        followUps.push_back(f);
      }
      break;
    case EventType::RouteRestore:
      mutateRoute(ev.target, [&](EdgeId e) {
        w_.graph.setAvailable(e, true);
        if (ev.severity > 0) w_.graph.setRisk(e, w_.graph.risk(e) + static_cast<float>(0.5 * ev.severity));
      });
      break;
    case EventType::RouteDanger:
      mutateRoute(ev.target, [&](EdgeId e) {
        double sev = std::min(1.0, std::max(0.0, ev.severity));
        w_.graph.setRisk(e, static_cast<float>(std::min(0.95, w_.graph.risk(e) + sev)));
        w_.graph.setReliability(e, static_cast<float>(w_.graph.rel(e) * (1.0 - sev / 2.0)));
      });
      break;
    case EventType::ShipDestroyed: {
      int s = w_.shipOf(ev.target);
      if (s >= 0) w_.ships[s].alive = false;
      break;
    }
    case EventType::ShipFailure: {
      int s = w_.shipOf(ev.target);
      if (s >= 0) {
        Ship& S = w_.ships[s];
        S.availableAt = std::max(S.availableAt, now) + std::max(1.0, ev.a > 0 ? ev.a : 24.0);
        S.reliability = std::max(0.5, S.reliability * 0.97);
      }
      break;
    }
    case EventType::EmergencyMedical: {
      int p = w_.planetOf(ev.target);
      if (p < 0) break;
      const int MED = static_cast<int>(Resource::Medicine);
      double amount = ev.a > 0 ? ev.a : 100.0;
      std::vector<std::pair<NodeId, double>> src;
      router_.nearestSources(
          p, PROFILE_EMERGENCY, 1, 50000,
          [&](NodeId v) {
            const Planet& P = w_.planets[v];
            return P.alive && !w_.supplyFrozen(v, MED) && P.stock[MED] - P.consumption[MED] * 2 >= amount;
          },
          src);
      if (src.empty()) break;
      newMission(src[0].first, p, MED, amount, now + (ev.b > 0 ? ev.b : 48.0), 5, 1.0, MissionSource::Emergency,
                 CAP_MEDICAL);
      break;
    }
    case EventType::ProductionDrop: {
      int p = w_.planetOf(ev.target);
      if (p < 0) break;
      int r = eventResource(ev);
      double f = 1.0 - std::min(1.0, std::max(0.0, ev.severity > 0 ? ev.severity : 0.6));
      for (int k = 0; k < NUM_RESOURCES; ++k)
        if (r < 0 || k == r) w_.planets[p].production[k] *= f;
      changedPlanets.push_back(p);
      infraTriggers_.push_back(p);
      pred_.update(p, now);
      break;
    }
    case EventType::NewColony: {
      Planet np;
      np.extId = w_.nextPlanetExt;
      np.type = PlanetType::Colony;
      np.x = ev.a; np.y = ev.b; np.z = ev.c;
      np.population = np.initialPopulation = ev.d > 0 ? ev.d : 5000.0;
      np.strategic = 1.5;
      np.discovered = true;
      const double perCap[NUM_RESOURCES] = {0.002, 0.003, 0.002, 0.0005, 0.0002, 0.0, 0.0005};
      for (int k = 0; k < NUM_RESOURCES; ++k) {
        np.consumption[k] = np.population * perCap[k];
        np.production[k] = 0.3 * np.consumption[k];
        np.capacity[k] = std::max(500.0, 40.0 * np.consumption[k]);
        np.stock[k] = 5.0 * np.consumption[k];
      }
      // nearest existing planets (brute force O(P), one event)
      std::vector<std::pair<double, int>> near;
      for (int v = 0; v < w_.numPlanets(); ++v) {
        const Planet& P = w_.planets[v];
        if (!P.alive) continue;
        near.push_back({std::hypot(P.x - np.x, P.y - np.y, P.z - np.z), v});
      }
      std::sort(near.begin(), near.end());
      NodeId v = w_.addPlanet(np);
      router_.onNodeAdded();
      for (size_t i = 0; i < near.size() && i < 3; ++i) {
        int u = near[i].second;
        double dist = std::max(1.0, near[i].first);
        int64_t rid = w_.nextRouteExt++;
        for (int dir = 0; dir < 2; ++dir) {
          EdgeAttr a;
          a.from = dir ? u : v;
          a.to = dir ? v : u;
          a.dist = static_cast<float>(dist);
          a.tmean = static_cast<float>(dist / 2.0);
          a.tmin = 0.8f * a.tmean;
          a.tmax = 1.4f * a.tmean;
          a.fuel = static_cast<float>(0.8 * dist);
          a.risk = 0.05f;
          a.cap = 1000.0f;
          a.rel = 0.98f;
          a.routeId = rid;
          EdgeId e = w_.graph.addEdge(a);
          w_.routeEdges[rid].push_back(e);
          router_.onEdgeAdded(e);
          changedEdges.push_back(e);
        }
      }
      // infrastructure options for the new colony (farm + energy plant, two levels each)
      for (int t : {static_cast<int>(FacilityType::Farm), static_cast<int>(FacilityType::EnergyPlant)})
        for (int lvl = 1; lvl <= 2; ++lvl) {
          FacilityOption f;
          f.extId = -(static_cast<int64_t>(w_.facilities.size()) + 1);
          f.planet = v;
          f.type = t;
          f.level = lvl;
          f.cost = 80.0 * lvl;
          f.buildDays = 2.0 + lvl;
          int res = t == static_cast<int>(FacilityType::Farm) ? 0 : 2;
          f.resourceImpact[res] = np.consumption[res] * 0.6 * lvl;
          f.strategicValue = 1.0 * lvl;
          w_.facilities.push_back(f);
        }
      changedPlanets.push_back(v);
      infraTriggers_.push_back(v);
      pred_.update(v, now);
      break;
    }
    case EventType::PopulationSurge: {
      int p = w_.planetOf(ev.target);
      if (p < 0) break;
      double f = 1.0 + std::max(0.0, ev.severity);
      Planet& P = w_.planets[p];
      P.population *= f;
      for (int k = 0; k < NUM_RESOURCES; ++k) P.consumption[k] *= f;
      changedPlanets.push_back(p);
      infraTriggers_.push_back(p);
      pred_.update(p, now);
      break;
    }
    case EventType::ResourceUnavailable: {
      int r = eventResource(ev);
      if (r < 0) break;
      double until = now + (ev.b > 0 ? ev.b : 72.0);
      if (ev.target < 0) {
        for (int p = 0; p < w_.numPlanets(); ++p) {
          w_.frozenUntil[static_cast<size_t>(p) * NUM_RESOURCES + r] = until;
          changedPlanets.push_back(p);
        }
      } else {
        int p = w_.planetOf(ev.target);
        if (p < 0) break;
        w_.frozenUntil[static_cast<size_t>(p) * NUM_RESOURCES + r] = until;
        changedPlanets.push_back(p);
      }
      break;
    }
    case EventType::NewHighPriorityMission: {
      int o = w_.planetOf(ev.target), d = w_.planetOf(static_cast<int64_t>(std::llround(ev.a)));
      int r = static_cast<int>(std::lround(ev.b));
      if (r < 0 || r >= NUM_RESOURCES) r = 0;
      double sev = std::min(1.0, std::max(0.0, ev.severity));
      int pri = 3 + static_cast<int>(std::lround(2 * sev));
      newMission(o, d, r, ev.c > 0 ? ev.c : 200.0, now + (ev.d > 0 ? ev.d : 72.0), pri, std::max(0.5, sev),
                 MissionSource::EventMission, r == static_cast<int>(Resource::Medicine) ? CAP_MEDICAL : 0u);
      break;
    }
  }
  return created;
}

ReplanRecord Replanner::handle(const Event& ev, double now, std::vector<Event>& followUps, const Deadline& hard) {
  Timer timer;
  ReplanRecord rec;
  rec.eventIndex = ev.index;
  rec.time = now;
  rec.type = ev.type;
  rec.target = ev.target;
  rec.mode = mode_;
  sched_.setNow(now);
  w_.now = now;
  rec.costBefore = sched_.total();
  // snapshot of current assignments (for the "unchanged" statistic)
  std::vector<std::pair<int, int>> before;
  for (int m : sched_.activeMissions()) before.push_back({m, sched_.solution().shipOf[m]});

  std::vector<NodeId> changedPlanets;
  std::vector<EdgeId> changedEdges;
  std::vector<int> created = applyEvent(ev, now, followUps, changedPlanets, changedEdges);
  if (!changedEdges.empty()) sched_.invalidateFuelCache();

  Deadline box = Deadline::inMs(3.0 * boxMs_).earlier(hard);  // guard; the work cap below decides
  sched_.setWorkCap((mode_ == ReplanMode::Full ? 10.0 : 1.0) * boxMs_ * WORK_UNITS_PER_MS);
  // flow repair (warm start) and reconciliation of proactive missions
  std::vector<int> cancelled;
  if (!changedPlanets.empty() || !changedEdges.empty()) {
    auto rr = alloc_.repair(changedEdges, changedPlanets, now, box);
    cancelled = missionsOfShipments(rr.cancelled);
    for (int sid : rr.added) {
      if (sid < 0 || sid >= static_cast<int>(alloc_.shipments().size())) continue;
      auto ms = materializeShipment(w_, alloc_.shipments()[sid], now, chunkCap_);
      registerShipmentMissions(sid, ms);
      created.insert(created.end(), ms.begin(), ms.end());
    }
  }
  std::vector<int> cancelledShips;
  for (int m : cancelled) {
    int s = sched_.solution().shipOf[m];
    if (s >= 0) cancelledShips.push_back(s);
  }
  sched_.deactivate(cancelled);
  for (int m : cancelled) {
    w_.missions[m].status = MissionStatus::Cancelled;
    w_.missions[m].plan.valid = false;
    w_.missions[m].plan.version++;
  }
  sched_.activate(created);
  sched_.buildCandidates(created);
  int evShip = (ev.type == EventType::ShipDestroyed || ev.type == EventType::ShipFailure) ? w_.shipOf(ev.target) : -1;

  lastInvalid_ = dep_.invalidSet(w_, ev);
  rec.invalid = static_cast<int>(lastInvalid_.size());
  rec.created = static_cast<int>(created.size());
  rec.cancelled = static_cast<int>(cancelled.size());
  rec.active = static_cast<int>(sched_.activeMissions().size());

  std::vector<int> R = lastInvalid_;
  R.insert(R.end(), created.begin(), created.end());
  std::sort(R.begin(), R.end());
  R.erase(std::unique(R.begin(), R.end()), R.end());

  std::unordered_set<int> touched(cancelledShips.begin(), cancelledShips.end());
  for (int m : R) {
    int s = sched_.isActive(m) ? sched_.solution().shipOf[m] : -1;
    if (s >= 0) touched.insert(s);
  }
  if (mode_ == ReplanMode::Full) {
    // baseline: discard everything and re-solve from scratch
    std::vector<int> all = sched_.activeMissions();
    for (int m : all) {
      int s = sched_.solution().shipOf[m];
      if (s >= 0) touched.insert(s);
    }
    sched_.unassignAll();
    if (evShip >= 0) sched_.refreshShip(evShip);
    sched_.buildCandidates(all);
    sched_.insertRegret(all, 2, hard);
    sched_.runALNS(fullIters_, hard, nullptr, nullptr);
    rec.removed = static_cast<int>(all.size());
    lastRemoved_ = all;
  } else {
    std::vector<int> removed = sched_.remove(lastInvalid_);
    if (evShip >= 0) sched_.refreshShip(evShip);
    rec.removed = static_cast<int>(removed.size());
    sched_.buildCandidates(lastInvalid_);
    bool escalate = R.size() > escalation_ * std::max<size_t>(1, sched_.activeMissions().size());
    std::vector<int> N = R;
    if (!R.empty()) {
      int extra = std::min(60, shawExtra_ * static_cast<int>(std::max<size_t>(1, R.size())));
      std::vector<int> rel = sched_.related(R, extra, nullptr);
      N.insert(N.end(), rel.begin(), rel.end());
    }
    sched_.insertRegret(R, 2, box);
    int64_t iters = sched_.iterationsForBudget(boxMs_, N.size());
    if (escalate) {
      rec.escalated = true;
      Deadline wide = Deadline::inMs(9 * boxMs_).earlier(hard);
      sched_.setWorkCap(3.0 * boxMs_ * WORK_UNITS_PER_MS);
      sched_.runALNS(iters, wide, nullptr, nullptr);
      for (int m : sched_.activeMissions()) {
        int s = sched_.solution().shipOf[m];
        if (s >= 0) touched.insert(s);
      }
      lastRemoved_ = sched_.activeMissions();
    } else if (!N.empty()) {
      sched_.runALNS(iters, box, &N, nullptr);
      lastRemoved_ = N;
    } else {
      lastRemoved_.clear();
    }
  }
  // ships whose plans may have changed
  for (int m : lastRemoved_) {
    int s = sched_.isActive(m) ? sched_.solution().shipOf[m] : -1;
    if (s >= 0) touched.insert(s);
  }
  for (const auto& [m, s] : before)
    if (s >= 0 && std::binary_search(lastInvalid_.begin(), lastInvalid_.end(), m)) touched.insert(s);
  if (evShip >= 0) touched.insert(evShip);
  std::vector<int> ships(touched.begin(), touched.end());
  std::sort(ships.begin(), ships.end());
  sched_.extractPlans(&ships);
  std::vector<int> toIndex;
  for (int s : ships)
    for (int m : sched_.solution().seq[s]) toIndex.push_back(m);
  reindex(&toIndex);

  int reinserted = 0, unchanged = 0;
  for (int m : R)
    if (sched_.isActive(m) && sched_.solution().shipOf[m] >= 0) ++reinserted;
  std::vector<int> rem = lastRemoved_;
  std::sort(rem.begin(), rem.end());
  for (const auto& [m, s] : before) {
    if (!sched_.isActive(m) || std::binary_search(rem.begin(), rem.end(), m)) continue;
    if (sched_.solution().shipOf[m] == s) ++unchanged;
  }
  rec.reinserted = reinserted;
  rec.unchanged = unchanged;
  sched_.setWorkCap(0);
  rec.costAfter = sched_.total();
  rec.ms = timer.ms();
  if (ev.type != EventType::RouteRestore && rec.invalid == 0 && rec.created == 0 && rec.cancelled == 0)
    rec.note = "no plan affected";
  return rec;
}

}  // namespace chronos
