// test_replan.cpp - dependency-index invalid set == brute force; after every event type the plan
// is valid and decisions outside the repaired neighbourhood are unchanged.
#include "chronos/replan.hpp"
#include "test_util.hpp"

using namespace chronos;

namespace {
struct Pipe {
  World w;
  Router router;
  Scheduler sched;
  ResourceAllocator alloc;
  Predictor pred;
  DependencyIndex dep;
  Replanner rp;
  explicit Pipe(World&& world)
      : w(std::move(world)),
        router(w.graph),
        sched(w, router, SchedulerConfig(), 5),
        alloc(w, router),
        pred(w),
        rp(w, router, sched, alloc, pred, dep) {}
  void plan() {
    router.setBoundsEnabled(true);
    pred.rebuild(0);
    std::vector<double> zero(static_cast<size_t>(w.numPlanets()) * NUM_RESOURCES, 0.0);
    AllocConfig ac;
    ac.fleetCapacity = 1e9;
    ac.maxShipments = 25;
    alloc.allocate(ac, 0.0, zero, zero, Deadline::never());
    for (const auto& sh : alloc.shipments()) {
      if (!sh.active) continue;
      auto ms = materializeShipment(w, sh, 0.0, 1000.0);
      rp.registerShipmentMissions(sh.id, ms);
    }
    sched.setNow(0);
    sched.syncWithWorld();
    sched.buildCandidates(sched.activeMissions());
    sched.insertRegret(sched.unassigned(), 2, Deadline::never());
    sched.runALNS(80, Deadline::never(), nullptr, nullptr);
    sched.extractPlans(nullptr);
    dep.indexAll(w);
    rp.setEscalation(1.0);  // keep repairs local so the "unchanged" property is observable
    rp.setChunkCap(1000.0);
  }
  int plannedMission(Rng& rng, bool needLoadedPath) const {
    std::vector<int> c;
    for (size_t m = 0; m < w.missions.size(); ++m) {
      const Mission& M = w.missions[m];
      if (M.active() && M.plan.valid && (!needLoadedPath || !M.plan.loadedPath.empty())) c.push_back(static_cast<int>(m));
    }
    return c.empty() ? -1 : c[rng.uniformInt(static_cast<int>(c.size()))];
  }
};

World baseWorld(uint64_t seed) {
  World w = testing::makeWorld(70, 260, seed);
  testing::addShips(w, 10, seed + 1, 1000, 400);
  testing::addMissions(w, 25, seed + 2, 5);
  for (auto& p : w.planets) p.stock[static_cast<int>(Resource::Medicine)] += 400;  // emergency sources
  return w;
}

Event makeEvent(const Pipe& P, EventType type, Rng& rng) {
  Event ev;
  ev.type = type;
  ev.time = 1.0;
  ev.severity = 0.6;
  int m = P.plannedMission(rng, true);
  const World& w = P.w;
  switch (type) {
    case EventType::WormholeCollapse:
    case EventType::RouteUnavailable:
    case EventType::PirateBlockade:
    case EventType::RouteDanger:
      ev.target = m >= 0 ? w.graph.routeId(w.missions[m].plan.loadedPath.front()) : 0;
      ev.a = 24;
      ev.severity = 0.4;
      break;
    case EventType::ShipDestroyed:
    case EventType::ShipFailure:
      ev.target = m >= 0 ? w.ships[w.missions[m].plan.ship].extId : 0;
      ev.a = 12;
      break;
    case EventType::ProductionDrop:
      ev.target = m >= 0 ? w.planets[w.missions[m].origin].extId : 0;
      ev.b = m >= 0 ? w.missions[m].resource : 0;
      break;
    case EventType::ResourceUnavailable:
      ev.target = m >= 0 ? w.planets[w.missions[m].origin].extId : 0;
      ev.a = m >= 0 ? w.missions[m].resource : 0;
      ev.b = 48;
      break;
    case EventType::EmergencyMedical:
      ev.target = rng.uniformInt(w.numPlanets());
      ev.a = 40;
      ev.b = 48;
      ev.severity = 0.9;
      break;
    case EventType::NewColony:
      ev.a = w.planets[5].x + 3;
      ev.b = w.planets[5].y + 3;
      ev.d = 5000;
      break;
    case EventType::PopulationSurge:
      ev.target = rng.uniformInt(w.numPlanets());
      ev.severity = 0.5;
      break;
    case EventType::NewHighPriorityMission:
      ev.target = 3;
      ev.a = 40;
      ev.b = 1;
      ev.c = 150;
      ev.d = 72;
      ev.severity = 1.0;
      break;
    default:
      break;
  }
  return ev;
}
}  // namespace

TEST(replan_dependency_index_equals_brute_force) {
  const EventType types[] = {EventType::WormholeCollapse, EventType::RouteUnavailable, EventType::PirateBlockade,
                             EventType::RouteDanger,      EventType::ShipDestroyed,    EventType::ShipFailure,
                             EventType::ProductionDrop,   EventType::ResourceUnavailable};
  for (uint64_t seed = 1; seed <= 3; ++seed) {
    Pipe P(baseWorld(seed));
    P.plan();
    Rng rng(seed * 101);
    for (int k = 0; k < 30; ++k) {
      EventType t = types[rng.uniformInt(8)];
      Event ev = makeEvent(P, t, rng);
      std::vector<Event> follow;
      std::vector<NodeId> cp;
      std::vector<EdgeId> ce;
      P.rp.applyEvent(ev, 1.0, follow, cp, ce);
      std::vector<int> fast = P.dep.invalidSet(P.w, ev), slow = bruteForceInvalidSet(P.w, ev);
      CHECK(fast == slow);
    }
    // also with a global (target -1) resource freeze
    Event g;
    g.type = EventType::ResourceUnavailable;
    g.target = -1;
    g.a = 2;
    g.b = 24;
    CHECK(P.dep.invalidSet(P.w, g) == bruteForceInvalidSet(P.w, g));
  }
}

TEST(replan_every_event_type_keeps_plan_valid_and_local) {
  for (int t = 0; t < NUM_EVENT_TYPES; ++t) {
    Pipe P(baseWorld(40 + static_cast<uint64_t>(t)));
    P.plan();
    Rng rng(7 + static_cast<uint64_t>(t));
    Event ev = makeEvent(P, static_cast<EventType>(t), rng);
    std::vector<std::pair<int, int>> before;
    for (int m : P.sched.activeMissions()) before.push_back({m, P.sched.solution().shipOf[m]});
    std::vector<Event> follow;
    ReplanRecord rec = P.rp.handle(ev, 1.0, follow, Deadline::never());
    ValidationReport rep = P.sched.validate();
    for (const auto& msg : rep.messages) std::printf("    [%s] %s\n", eventTypeName(ev.type), msg.c_str());
    CHECK(rep.ok());
    // stored plans of assigned missions only use available routes
    for (int m : P.sched.activeMissions()) {
      const Mission& M = P.w.missions[m];
      if (P.sched.solution().shipOf[m] < 0) continue;
      CHECK(M.plan.valid);
      CHECK(pathUsable(P.w.graph, M.plan.emptyPath, 0.0));
      CHECK(pathUsable(P.w.graph, M.plan.loadedPath, 0.0));
      CHECK(P.w.ships[M.plan.ship].alive);
    }
    // decisions outside the repaired neighbourhood are unchanged
    std::vector<int> touched = P.rp.lastRemoved();
    touched.insert(touched.end(), P.rp.lastInvalid().begin(), P.rp.lastInvalid().end());
    std::sort(touched.begin(), touched.end());
    int kept = 0, moved = 0;
    for (const auto& [m, s] : before) {
      if (!P.sched.isActive(m) || std::binary_search(touched.begin(), touched.end(), m)) continue;
      if (P.sched.solution().shipOf[m] == s) ++kept;
      else ++moved;
    }
    CHECK(!rec.escalated);
    CHECK(moved == 0);
    CHECK(kept == rec.unchanged);
    if (ev.type == EventType::ShipDestroyed) {
      int s = P.w.shipOf(ev.target);
      CHECK(s >= 0 && P.sched.solution().seq[s].empty());
    }
    if (ev.type == EventType::EmergencyMedical || ev.type == EventType::NewHighPriorityMission ||
        ev.type == EventType::NewColony)
      CHECK(rec.created > 0 || ev.type == EventType::NewColony);
  }
}

TEST(replan_full_mode_also_valid) {
  Pipe P(baseWorld(77));
  P.plan();
  P.rp.setMode(ReplanMode::Full);
  P.rp.setFullIterations(50);
  Rng rng(1);
  for (int t : {0, 3, 4, 9, 11}) {
    Event ev = makeEvent(P, static_cast<EventType>(t), rng);
    std::vector<Event> follow;
    P.rp.handle(ev, 1.0, follow, Deadline::never());
    CHECK(P.sched.validate().ok());
  }
}

TEST(replan_escalation_triggers_on_large_invalid_sets) {
  Pipe P(baseWorld(88));
  P.plan();
  P.rp.setEscalation(0.0);
  Rng rng(2);
  Event ev = makeEvent(P, EventType::ShipDestroyed, rng);
  std::vector<Event> follow;
  ReplanRecord rec = P.rp.handle(ev, 1.0, follow, Deadline::never());
  CHECK(rec.invalid == 0 || rec.escalated);
  CHECK(P.sched.validate().ok());
}
