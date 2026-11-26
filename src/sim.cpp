// sim.cpp - per-cycle pipeline, execution model with stochastic travel times and
// failures, accounting, and output writers. Per cycle: O(P R) planet updates +
// allocation + scheduling (budgeted) + O(events * replan).
#include "chronos/sim.hpp"

#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <unordered_set>

#include "chronos/io.hpp"

namespace chronos {

namespace {
constexpr double GRACE_DAYS = 3.0;     // critical shortage tolerated before population loss
constexpr double POP_LOSS_PER_DAY = 0.02;
constexpr double EXPIRE_GRACE_H = 48.0;
constexpr double MATERIALS_PER_COST = 0.05;

std::string extPath(const World& w, const std::vector<EdgeId>& edges, NodeId start) {
  std::string s;
  auto nodes = pathNodes(w.graph, edges, start);
  for (size_t i = 0; i < nodes.size(); ++i) {
    if (i) s += ';';
    s += std::to_string(w.planets[nodes[i]].extId);
  }
  return s;
}
}  // namespace

Simulator::Simulator(World& w, const SimConfig& cfg)
    : w_(w),
      cfg_(cfg),
      budget_(std::max(1.0, cfg.timeMs - cfg.loadMs), cfg.memMB, cfg.timeMs),
      router_(w.graph),
      sched_(w, router_, SchedulerConfig(), cfg.seed * 7919ULL + 17ULL),
      alloc_(w, router_),
      pred_(w),
      infra_(w),
      replan_(w, router_, sched_, alloc_, pred_, dep_),
      rng_(cfg.seed) {}

// ------------------------------------------------------------------ helpers
double Simulator::chunkCapacity() const {
  std::vector<double> caps;
  for (const auto& s : w_.ships)
    if (s.alive) caps.push_back(s.cargoCap);
  if (caps.empty()) return 500.0;
  std::nth_element(caps.begin(), caps.begin() + caps.size() / 2, caps.end());
  return std::max(1.0, caps[caps.size() / 2]);
}

std::vector<double> Simulator::incomingVector() const {
  std::vector<double> v(static_cast<size_t>(w_.numPlanets()) * NUM_RESOURCES, 0.0);
  for (const auto& m : w_.missions) {
    size_t k = static_cast<size_t>(m.dest) * NUM_RESOURCES + m.resource;
    if (m.active()) v[k] += m.amount;
  }
  for (const auto& f : inFlight_)
    if (!f.failed) {
      const Mission& m = w_.missions[f.mission];
      v[static_cast<size_t>(m.dest) * NUM_RESOURCES + m.resource] += m.delivered;
    }
  return v;
}

std::vector<double> Simulator::outgoingVector() const {
  std::vector<double> v(static_cast<size_t>(w_.numPlanets()) * NUM_RESOURCES, 0.0);
  for (const auto& m : w_.missions)
    if (m.active()) v[static_cast<size_t>(m.origin) * NUM_RESOURCES + m.resource] += m.amount;
  return v;
}

Features Simulator::features() const {
  Features f;
  f.V = w_.graph.numNodes();
  f.E = w_.graph.numEdges();
  f.activeMissions = 0;
  f.minSlackH = INF;
  for (const auto& m : w_.missions)
    if (m.active() && m.release <= w_.now + 1e-9) {
      ++f.activeMissions;
      f.minSlackH = std::min(f.minSlackH, m.deadline - w_.now);
    }
  for (const auto& s : w_.ships)
    if (s.alive) ++f.ships;
  f.remainingMs = budget_.remainingMs();
  f.totalMs = budget_.totalMs();
  f.memUsedMB = currentMemoryMB();
  f.memBudgetMB = budget_.memBudgetMB();
  f.eventSeverity = maxSeverity_;
  int E = w_.graph.numEdges(), step = std::max(1, E / 10000), n = 0;
  double acc = 0;
  for (int e = 0; e < E; e += step) {
    double m = w_.graph.tmean(e);
    if (m > 0) { acc += w_.graph.sigma(e) / m; ++n; }
  }
  f.routeVariance = n ? acc / n : 0.0;
  int days = cfg_.days > 0 ? cfg_.days : w_.days;
  f.cyclesLeft = std::max(1, days - static_cast<int>(w_.now / HOURS_PER_DAY));
  f.cyclesTotal = std::max(1, days);
  return f;
}

void Simulator::reindexPlans(const std::vector<int>* ships) {
  if (!ships) {
    dep_.indexAll(w_);
    return;
  }
  for (int s : *ships)
    for (int m : sched_.solution().seq[s]) dep_.indexMission(w_, m);
}

// ------------------------------------------------------------------ execution
void Simulator::commitMission(int s, int m) {
  Ship& S = w_.ships[s];
  Mission& M = w_.missions[m];
  const MissionPlan& P = M.plan;
  double t = std::max(P.plannedStart, S.availableAt);
  double fuel = S.fuel, dist = 0, fuelUsed = 0, rel = S.reliability, travelH = 0, cost = 0;
  size_t ri = 0;
  auto refuelAt = [&](NodeId v) {
    while (ri < P.refuelNodes.size() && P.refuelNodes[ri] == v) {
      double need = S.fuelCap - fuel;
      Planet& pl = w_.planets[v];
      double& stock = pl.stock[static_cast<int>(Resource::Fuel)];
      double take = pl.type == PlanetType::Station ? need : std::min(need, std::max(0.0, stock));
      if (pl.type != PlanetType::Station) stock -= take;
      fuel += take;
      if (pl.type == PlanetType::Station) fuel = S.fuelCap;
      t += sched_.config().refuelH;
      cost += sched_.config().refuelCost;
      ++ri;
    }
  };
  auto travel = [&](const std::vector<EdgeId>& path, NodeId start, int profile, double costFactor) {
    refuelAt(start);
    for (EdgeId e : path) {
      double h = rng_.triangular(w_.graph.tmin(e), w_.graph.tmean(e), w_.graph.tmax(e)) / S.speed;
      t += h;
      travelH += h;
      dist += w_.graph.dist(e);
      fuel -= w_.graph.fuel(e);
      fuelUsed += w_.graph.fuel(e);
      rel *= w_.graph.rel(e);
      double c = edgeCost(w_.graph, e, profile);
      cost += costFactor * (std::isfinite(c) ? c : 0.0);
      refuelAt(w_.graph.to(e));
    }
  };
  NodeId start = P.emptyPath.empty() ? M.origin : w_.graph.from(P.emptyPath.front());
  travel(P.emptyPath, start, PROFILE_CARGO, sched_.config().emptyLegFactor);
  double depart = std::max(t, M.release) + sched_.config().loadH;
  t = depart;
  // load cargo (bounded by what the origin actually holds)
  Planet& O = w_.planets[M.origin];
  double loaded = std::min(M.amount, std::max(0.0, O.stock[M.resource]));
  O.stock[M.resource] -= loaded;
  travel(P.loadedPath, M.origin, P.profile, 1.0);
  double arrive = t;
  cost += S.opCost * travelH;
  bool failed = rng_.bernoulli(1.0 - rel) || loaded <= 1e-9;
  M.status = MissionStatus::InFlight;
  M.executedBy = s;
  M.actualDepart = depart;
  M.actualArrive = arrive;
  M.delivered = failed ? 0.0 : loaded;
  M.late = arrive > M.deadline + 1e-9;
  S.location = M.dest;
  S.availableAt = arrive + sched_.config().unloadH;
  S.fuel = std::max(0.0, std::min(S.fuelCap, fuel));
  S.distanceTraveled += dist;
  S.fuelUsed += fuelUsed;
  distance_ += dist;
  fuelUsed_ += fuelUsed;
  transportCost_ += cost;
  const Path& naive = router_.route(M.origin, M.dest, PROFILE_BALANCED);
  if (naive.found) {
    naiveTransport_ += 2.0 * (naive.cost + S.opCost * naive.timeMean / S.speed);
    naiveFuel_ += 2.0 * naive.fuel;
  }
  riskSum_ += 1.0 - rel;
  riskCount_++;
  inFlight_.push_back({m, s, arrive, failed});
}

void Simulator::execute(double until) {
  for (size_t si = 0; si < w_.ships.size(); ++si) {
    int s = static_cast<int>(si);
    bool changed = false;
    while (w_.ships[s].alive && !sched_.solution().seq[s].empty()) {
      int m = sched_.solution().seq[s].front();
      const Mission& M = w_.missions[m];
      if (!M.plan.valid || M.plan.ship != s || M.plan.seqIndex != 0) {
        std::vector<int> one{s};
        sched_.extractPlans(&one);
        if (!w_.missions[m].plan.valid) break;  // infeasible head: left for repair
      }
      if (w_.missions[m].plan.plannedStart >= until) break;
      if (w_.planets[M.origin].stock[M.resource] <= 1e-6) {
        // nothing to load at the origin: proactive missions are cancelled (the flow re-plans the
        // deficit next cycle); input missions are delayed (unassigned, re-scheduled later)
        if (M.source == MissionSource::Proactive) {
          w_.missions[m].status = MissionStatus::Cancelled;
          sched_.deactivate({m});
        } else {
          sched_.remove({m});
          w_.missions[m].plan.valid = false;
          w_.missions[m].plan.version++;
        }
        changed = true;
        continue;
      }
      commitMission(s, m);
      sched_.deactivate({m});
      changed = true;
    }
    if (changed) {
      std::vector<int> one{s};
      sched_.extractPlans(&one);
      reindexPlans(&one);
    }
  }
}

void Simulator::processArrivals(double until) {
  std::vector<InFlight> keep;
  for (const auto& f : inFlight_) {
    if (f.arrive > until) {
      keep.push_back(f);
      continue;
    }
    Mission& M = w_.missions[f.mission];
    if (f.failed) {
      M.status = MissionStatus::Failed;
      continue;
    }
    Planet& D = w_.planets[M.dest];
    D.stock[M.resource] += M.delivered;
    if (D.stock[M.resource] > D.capacity[M.resource]) {
      double ex = D.stock[M.resource] - D.capacity[M.resource];
      D.stock[M.resource] = D.capacity[M.resource];
      wasteUnits_ += ex;
      wasteW_ += RESOURCE_IMPORTANCE[M.resource] * ex;
    }
    delivered_ += M.delivered;
    M.status = MissionStatus::Completed;
  }
  inFlight_.swap(keep);
}

void Simulator::expireMissions(double now) {
  std::vector<int> exp;
  for (size_t m = 0; m < w_.missions.size(); ++m) {
    Mission& M = w_.missions[m];
    if (M.active() && M.release <= now && M.deadline + EXPIRE_GRACE_H < now) {
      M.status = MissionStatus::Expired;
      exp.push_back(static_cast<int>(m));
    }
  }
  sched_.deactivate(exp);
}

void Simulator::completeFacilities(double now) {
  const int C = static_cast<int>(Resource::Construction), E = static_cast<int>(Resource::Energy);
  for (auto& f : w_.facilities) {
    if (!f.started || f.built || f.completeAt > now) continue;
    Planet& P = w_.planets[f.planet];
    double need = f.cost * MATERIALS_PER_COST;
    if (P.stock[C] + 1e-9 < need && now < f.completeAt + 72.0) {
      f.completeAt += HOURS_PER_DAY;  // waiting for construction materials
      continue;
    }
    P.stock[C] = std::max(0.0, P.stock[C] - need);
    double& extra = w_.extraDemand[static_cast<size_t>(f.planet) * NUM_RESOURCES + C];
    extra = std::max(0.0, extra - need);
    for (int r = 0; r < NUM_RESOURCES; ++r) {
      if (f.resourceImpact[r] >= 0) P.production[r] += f.resourceImpact[r];
      else P.consumption[r] += -f.resourceImpact[r];
    }
    if (f.energyImpact >= 0) P.production[E] += f.energyImpact;
    else P.consumption[E] += -f.energyImpact;
    if (f.populationImpact > 0) {
      P.population += f.populationImpact;
      popExposed_ += f.populationImpact;
    }
    if (f.type == static_cast<int>(FacilityType::ResearchStation)) P.tech += 0.5 * f.level;
    if (f.type == static_cast<int>(FacilityType::DefenseSystem)) P.defense += f.level;
    P.strategic += 0.01 * f.strategicValue;
    f.built = true;
    stats_.facilitiesBuilt++;
  }
}

void Simulator::advancePlanets(double fromH, double toH) {
  double dt = (toH - fromH) / HOURS_PER_DAY;
  for (int p = 0; p < w_.numPlanets(); ++p) {
    Planet& P = w_.planets[p];
    if (!P.alive) continue;
    bool critical = false;
    for (int r = 0; r < NUM_RESOURCES; ++r) {
      double imp = RESOURCE_IMPORTANCE[r];
      double prod = P.production[r] * dt, cons = P.consumption[r] * dt;
      producedW_ += imp * prod;
      demandW_ += imp * cons;
      P.stock[r] += prod;
      if (P.stock[r] + 1e-9 >= cons) {
        P.stock[r] -= cons;
        P.shortageDays[r] = 0;
      } else {
        double sh = cons - P.stock[r];
        P.stock[r] = 0;
        shortageUnits_ += sh;
        shortageW_ += imp * sh;
        if (isCriticalResource(r)) P.shortageDays[r]++;
      }
      if (P.stock[r] > P.capacity[r]) {
        double ex = P.stock[r] - P.capacity[r];
        P.stock[r] = P.capacity[r];
        wasteUnits_ += ex;
        wasteW_ += imp * ex;
      }
      if (isCriticalResource(r) && P.shortageDays[r] > GRACE_DAYS) critical = true;
    }
    if (critical) {
      double loss = P.population * POP_LOSS_PER_DAY * dt;
      P.population -= loss;
      if (static_cast<size_t>(p) >= lostPop_.size()) lostPop_.resize(p + 1, 0.0);
      lostPop_[p] += loss;
    }
  }
}

void Simulator::handleShipLoss(int ship, double now, bool destroyed, double repairH) {
  for (auto& f : inFlight_) {
    if (f.ship != ship || f.arrive <= now) continue;
    if (destroyed) {
      f.failed = true;  // cargo lost with the ship
      w_.missions[f.mission].delivered = 0;
    } else {
      f.arrive += repairH;
      w_.missions[f.mission].actualArrive = f.arrive;
      w_.missions[f.mission].late = f.arrive > w_.missions[f.mission].deadline;
    }
  }
}

void Simulator::runInfrastructure(const std::vector<NodeId>* planets, const std::string& trigger, double share) {
  Timer t;
  std::vector<int> ids;
  std::unordered_set<int> want;
  if (planets) want.insert(planets->begin(), planets->end());
  for (size_t i = 0; i < w_.facilities.size(); ++i) {
    const auto& f = w_.facilities[i];
    if (f.built || f.started) continue;
    if (planets && !want.count(f.planet)) continue;
    ids.push_back(static_cast<int>(i));
  }
  if (ids.empty() || w_.budget <= 0) return;
  int days = cfg_.days > 0 ? cfg_.days : w_.days;
  double horizon = std::max(1.0, days - w_.now / HOURS_PER_DAY);
  double budget = w_.budget * share;
  InfraResult r = infra_.plan(ids, budget, horizon, strat_.infra, budget_.hard());
  for (int o : r.chosen) {
    FacilityOption& f = w_.facilities[o];
    if (f.cost > w_.budget + 1e-9) continue;
    f.started = true;
    f.completeAt = w_.now + f.buildDays * HOURS_PER_DAY;
    w_.budget -= f.cost;
    stats_.budgetSpent += f.cost;
    w_.extraDemand[static_cast<size_t>(f.planet) * NUM_RESOURCES + static_cast<int>(Resource::Construction)] +=
        f.cost * MATERIALS_PER_COST;
    InfraRecord rec;
    rec.option = o;
    rec.start = w_.now;
    rec.complete = f.completeAt;
    rec.value = infra_.optionValue(f, horizon);
    rec.method = infraMethodName(r.method);
    rec.trigger = trigger;
    infraLog_.push_back(rec);
  }
  if (!cycles_.empty()) cycles_.back().infraMs += t.ms();
}

// ------------------------------------------------------------------ one planning cycle
void Simulator::planCycle(int day) {
  Timer cycleTimer;  // everything this cycle spends counts against its share (see the scheduler deadline)
  CycleRecord cr;
  cr.day = day;
  double now = w_.now;
  if (budget_.memoryPressure()) {  // memory guard: drop memo tables first
    router_.cache().clear();
    router_.cache().setCapacity(10000);
    sched_.invalidateFuelCache();
  }
  // 1) prediction
  pred_.rebuild(now);
  std::vector<ShortagePrediction> preds = pred_.upcoming(now, 7 * HOURS_PER_DAY);
  cr.predicted = static_cast<int>(preds.size());
  // 2) meta layer (events were already repaired by the replanner when they happened, so the
  //    cycle-level decision does not use the urgent-event rule)
  Features f = features();
  f.eventSeverity = 0;
  strat_ = meta_.choose(f, budget_, cfg_.schedForced);
  maxSeverity_ = 0;
  if (strat_.routing == RouteAlgo::Bidirectional && cfg_.routing == RouteAlgo::ALT && budget_.memoryPressure()) {
    router_.setAlgo(RouteAlgo::Bidirectional);  // memory pressure: drop landmark tables
    router_.setBoundsEnabled(false);
    router_.alt().dropAll();
    router_.cache().clear();
  }
  // landmark rebuilds (after K decreases / new edges) only when they cost < 10% of a cycle budget
  {
    // deterministic work model: (2L+1) full Dijkstras per profile at ~20k relaxations per ms
    double work = (2.0 * router_.alt().landmarkCount() + 1) * (w_.graph.numNodes() + w_.graph.numEdges()) / 20000.0;
    router_.setRebuildAllowed(2.0 * work < 0.10 * budget_.plannedCycleMs(f.cyclesTotal));
  }
  router_.maintain();
  sched_.config().kNearest = strat_.kNearest;
  sched_.config().beam = strat_.beam;
  sched_.config().appendOnly = strat_.appendOnly;
  cr.strategy = schedMethodName(strat_.sched);
  cr.reason = strat_.reason;
  // 3) resource allocation (reduced min-cost flow) -> proactive missions
  AllocConfig ac;
  ac.method = cfg_.allocForced >= 0 ? static_cast<AllocMethod>(cfg_.allocForced) : strat_.alloc;
  ac.K = strat_.allocK;
  double chunk = chunkCapacity();
  ac.refShipCap = chunk;
  // soft fleet throughput for this cycle: ships free within 48 h with at most one queued mission
  double fleet = 0;
  for (size_t s = 0; s < w_.ships.size(); ++s) {
    const Ship& sh = w_.ships[s];
    size_t queued = s < sched_.solution().seq.size() ? sched_.solution().seq[s].size() : 0;
    if (sh.alive && sh.availableAt < now + 48.0 && queued <= 1) fleet += sh.cargoCap;
  }
  ac.fleetCapacity = fleet;
  int freeShips = 0;
  for (size_t s = 0; s < w_.ships.size(); ++s) {
    const Ship& sh = w_.ships[s];
    size_t queued = s < sched_.solution().seq.size() ? sched_.solution().seq[s].size() : 0;
    if (sh.alive && sh.availableAt < now + 48.0 && queued <= 1) ++freeShips;
  }
  // trip budget: free ships, and what the scheduling budget can insert this cycle (deterministic
  // throughput model: ~1 insertion per ms on graphs up to 20k nodes, ~0.25 beyond) minus the
  // missions still waiting for a ship, so the pool cannot grow faster than it can be scheduled
  {
    int V = w_.graph.numNodes();
    double rate = V <= 2000 ? 2.0 : (V <= 20000 ? 1.0 : (V <= 200000 ? 0.5 : 0.2));
    int waiting = 0;
    for (const auto& m : w_.missions)
      if (m.active() && m.release <= now && m.plan.ship < 0) ++waiting;
    int capacity = static_cast<int>(std::max(50.0, strat_.schedBudgetMs * rate)) - waiting;
    ac.maxShipments = std::max(0, std::min(freeShips, capacity));
  }
  std::vector<double> incoming = incomingVector(), outgoing = outgoingVector();
  AllocationStats ast =
      alloc_.allocate(ac, now, incoming, outgoing,
                      Deadline::inMs(std::min(2.0 * strat_.allocBudgetMs, 0.2 * budget_.cycleBudgetMs(f.cyclesLeft)))
                          .earlier(budget_.hard()));
  cr.allocMs = ast.ms;
  cr.allocObjective = ast.transportCost + ast.shortagePenalty;
  replan_.clearShipmentMissions();
  replan_.setChunkCap(chunk);
  std::vector<int> created;
  std::vector<char> shippedTo(static_cast<size_t>(w_.numPlanets()) * NUM_RESOURCES, 0);
  for (const auto& sh : alloc_.shipments()) {
    if (!sh.active) continue;
    pred_.setLeadTime(sh.to, sh.pathTime + 1.2816 * sh.pathSigma + 12.0);
    auto ms = materializeShipment(w_, sh, now, chunk);
    replan_.registerShipmentMissions(sh.id, ms);
    created.insert(created.end(), ms.begin(), ms.end());
    shipmentLog_.push_back(sh);
    shipmentDay_.push_back(day);
    shippedTo[static_cast<size_t>(sh.to) * NUM_RESOURCES + sh.resource] = 1;
  }
  stats_.proactiveMissions += static_cast<int>(created.size());
  cr.shipments = 0;  // shipments actually materialised (after the trip budget)
  for (const auto& sh : alloc_.shipments()) cr.shipments += sh.active ? 1 : 0;
  cr.created = static_cast<int>(created.size());
  for (const auto& p : preds) {
    size_t k = static_cast<size_t>(p.planet) * NUM_RESOURCES + p.resource;
    PredictionRecord pr;
    pr.day = day;
    pr.p = p;
    pr.p.leadTimeH = pred_.leadTime(p.planet);
    pr.p.deliveryDeadlineH = now + p.timeToShortageH - pr.p.leadTimeH;
    pr.covered = incoming[k] > 0 || shippedTo[k];
    predictions_.push_back(pr);
  }
  // 4) scheduling: the meta layer re-decides with the mission pool after allocation
  {
    Strategy allocPart = strat_;
    Features f2 = features();
    f2.eventSeverity = f.eventSeverity;
    strat_ = meta_.choose(f2, budget_, cfg_.schedForced);
    strat_.alloc = allocPart.alloc;
    strat_.allocK = allocPart.allocK;
    sched_.config().kNearest = strat_.kNearest;
    sched_.config().beam = strat_.beam;
    sched_.config().appendOnly = strat_.appendOnly;
    cr.strategy = schedMethodName(strat_.sched);
    cr.reason = strat_.reason;
  }
  sched_.setNow(now);
  sched_.config().usePruning = cfg_.usePruning;
  if (paretoFailDay_.size() < w_.missions.size()) paretoFailDay_.resize(w_.missions.size(), -100);
  for (size_t m = 0; m < w_.missions.size(); ++m) {
    Mission& M = w_.missions[m];
    // a mission whose Pareto search found nothing is retried every 3rd day, not every cycle
    // (the search is the most expensive per-mission step on 100k-node galaxies)
    if (M.active() && M.release <= now && M.priority >= 5 && M.fixedPath.empty() && M.source == MissionSource::Input &&
        day - paretoFailDay_[m] >= 3)
      if (!assignParetoPath(w_, router_, static_cast<int>(m), sched_.config().zOnTime)) paretoFailDay_[m] = day;
  }
  sched_.syncWithWorld();
  const std::vector<int> active = sched_.activeMissions();
  {
    // candidate lists for unassigned missions (and any whose list is empty); assigned missions keep
    // theirs, which bounds this step by the number of new missions instead of the whole pool
    std::vector<int> need;
    for (int m : active)
      if (sched_.solution().shipOf[m] < 0 || sched_.candidates(m).empty()) need.push_back(m);
    // greedy regimes try missions one at a time in priority order and usually stop on the work
    // cap: build their lists lazily. Regret insertion compares all missions, so it needs them all.
    bool lazy = strat_.sched == SchedMethod::GreedyPruneALNS || strat_.sched == SchedMethod::LocalRepair ||
                strat_.sched == SchedMethod::Greedy;
    sched_.beginCandidateEpoch(lazy);
    if (lazy) {
      for (int m : need) sched_.clearCandidates(m);  // stale lists must not survive into this cycle
    } else {
      sched_.buildCandidates(need);
    }
  }
  const Counters work0 = counters();  // for the CHRONOS_DIAG work breakdown below
  Timer ts;
  // guard: 2x the nominal budget, but never more than this cycle's fair share of what is left.
  // The share must also pay for what this cycle already spent (prediction, allocation, Pareto
  // paths, candidate lists) and for today's event repairs, which run after planning: on 100k-node
  // galaxies those were ~40 % of a day and pushed tight runs past the time limit.
  double share = budget_.cycleBudgetMs(f.cyclesLeft);
  double expectReplan = 0;
  {
    double t1 = now + HOURS_PER_DAY, sum = 0;
    int n = 0, today = 0;
    for (const auto& r : replans_) {
      sum += r.ms;
      ++n;
    }
    for (size_t i = queuePos_; i < queue_.size() && queue_[i].time < t1; ++i)
      if (queue_[i].type != EventType::RouteRestore) ++today;
    double avg = n ? sum / n : strat_.replanBoxMs;
    expectReplan = std::min(0.5 * share, avg * today);
  }
  // only when the run is behind its pace (the previous day overspent its share): at normal budgets
  // the work cap already bounds scheduling, and trimming it there costs quality (large: -1 point)
  double schedRoom = INF;
  if (lastDayMs_ > 1.05 * share) schedRoom = std::max(0.1 * share, 0.9 * share - expectReplan - cycleTimer.ms());
  Deadline sdl = Deadline::inMs(std::min({2.0 * strat_.schedBudgetMs, 0.8 * share, schedRoom})).earlier(budget_.hard());
  std::vector<int> un = sched_.unassigned();
  std::vector<ConvergencePoint>* trace = day == 0 ? &convergence_ : nullptr;
  sched_.setWorkCap(strat_.schedBudgetMs * WORK_UNITS_PER_MS);  // deterministic stage budget
  int64_t iters = 0;
  switch (strat_.sched) {
    case SchedMethod::BranchAndBound:
      sched_.solveBranchAndBound(5000000, sdl);
      break;
    case SchedMethod::Greedy:
    case SchedMethod::LocalRepair:
      sched_.insertGreedy(un, sdl);
      break;
    case SchedMethod::GA:
      sched_.solveGA(1000000, sdl, trace);
      break;
    case SchedMethod::SA:
      sched_.insertGreedy(un, sdl);
      iters = sched_.iterationsForBudget(strat_.schedBudgetMs, active.size()) * 20;
      sched_.solveSA(iters, sdl, trace);
      break;
    case SchedMethod::GreedyPruneALNS:
      sched_.insertGreedy(un, sdl);
      if (cfg_.useALNS) {
        iters = sched_.iterationsForBudget(0.5 * strat_.schedBudgetMs, active.size());
        iters = sched_.runALNS(iters, sdl, nullptr, trace);  // and bounded by the work cap
      }
      break;
    default:  // RegretALNS
      if (cfg_.useRegret) sched_.insertRegret(un, 2, sdl);
      else sched_.insertGreedy(un, sdl);
      if (cfg_.useALNS) {
        iters = sched_.iterationsForBudget(0.8 * strat_.schedBudgetMs, active.size());
        iters = sched_.runALNS(iters, sdl, nullptr, trace);  // and bounded by the work cap
      }
      if (cfg_.useLocalSearch) sched_.localSearch(sdl, nullptr);
      break;
  }
  sched_.setWorkCap(0);
  if (cfg_.useRepair) sched_.repairInvalid(sdl);
  cr.alnsIters = iters;
  stats_.alnsIterations += iters;
  cr.schedMs = ts.ms();
  // 5) validation + plan extraction + dependency index
  ValidationReport rep = sched_.validate();
  cr.hardViolations = rep.hardViolations;
  cr.late = rep.lateMissions;
  stats_.validationHardViolations += rep.hardViolations;
  sched_.extractPlans(nullptr);
  dep_.indexAll(w_);
  if (std::getenv("CHRONOS_DIAG")) {
    // diagnostics: where this cycle's scheduling work went (stderr; off unless CHRONOS_DIAG is set)
    const Counters& c = counters();
    auto d = [](uint64_t a, uint64_t b) { return static_cast<unsigned long long>(a - b); };
    std::fprintf(stderr,
                 "DIAG day %d sched: states=%llu routing_settled=%llu fuel_labels=%llu insertion_evals=%llu "
                 "candidates=%llu cache_misses=%llu refuel_along_route=%llu fuel_searches=%llu found=%llu capped=%llu "
                 "fuel_search_labels=%llu pareto_searches=%llu pareto_labels=%llu ms=%.0f\n",
                 day, d(c.statesExplored, work0.statesExplored), d(c.routingSettled, work0.routingSettled),
                 d(c.labelsCreated, work0.labelsCreated), d(c.insertionEvals, work0.insertionEvals),
                 d(c.candidatesEvaluated, work0.candidatesEvaluated), d(c.cacheMisses, work0.cacheMisses),
                 d(c.refuelAlongRoute, work0.refuelAlongRoute), d(c.fuelSearches, work0.fuelSearches),
                 d(c.fuelSearchesFound, work0.fuelSearchesFound), d(c.fuelSearchesCapped, work0.fuelSearchesCapped),
                 d(c.fuelLabels, work0.fuelLabels), d(c.paretoSearches, work0.paretoSearches),
                 d(c.paretoLabels, work0.paretoLabels), ts.ms());
  }
  cr.active = static_cast<int>(sched_.activeMissions().size());
  cr.assigned = sched_.assignedCount();
  cr.cost = sched_.total();
  cycles_.push_back(cr);
  // 6) infrastructure: initial plan on day 0 (60% of the budget, rest kept for event-triggered re-runs)
  if (day == 0) {
    initialBudget_ = w_.budget;
    runInfrastructure(nullptr, "initial", 0.6);
  }
  // growth round: on day 2 the event reserve is released down to 10% of the initial budget. Money
  // left at the end of the horizon scores nothing, while every facility that completes before the
  // end adds to the growth/science/security terms; day 2 still leaves time for the longest builds.
  if (day == 2 && w_.budget > 0.1 * initialBudget_) {
    double share = (w_.budget - 0.1 * initialBudget_) / w_.budget;
    runInfrastructure(nullptr, "growth", share);
  }
  if (!cfg_.quiet)
    std::fprintf(stderr, "[day %d] active=%d assigned=%d shipments=%d sched=%s %.1fms alloc %.1fms cost=%.1f\n", day,
                 cr.active, cr.assigned, cr.shipments, cr.strategy.c_str(), cr.schedMs, cr.allocMs, cr.cost);
}

// ------------------------------------------------------------------ main loop
Statistics Simulator::run() {
  clock_.reset();
  counters().reset();
  int days = cfg_.days > 0 ? cfg_.days : w_.days;
  stats_ = Statistics();
  stats_.replanMode = replanModeName(cfg_.replanMode);
  stats_.loadMs = cfg_.loadMs;
  // setup
  router_.setAlgo(cfg_.routing);
  router_.alt().setLandmarkCount(MetaSelector::landmarkCount(w_.graph.numNodes(), cfg_.memMB));
  size_t cacheCap = static_cast<size_t>(std::min(500000.0, std::max(10000.0, cfg_.memMB * 1e6 * 0.10 / 600.0)));
  router_.cache().setCapacity(cacheCap);
  sched_.config().usePruning = cfg_.usePruning;
  replan_.setMode(cfg_.replanMode);
  for (const auto& p : w_.planets) {
    popExposed_ += p.population;
    for (int r = 0; r < NUM_RESOURCES; ++r) prodValueStart_ += RESOURCE_IMPORTANCE[r] * p.production[r];
    techStart_ += p.tech * p.strategic;
  }
  infra_.setScoreReference(prodValueStart_, techStart_, days);
  lostPop_.assign(w_.planets.size(), 0.0);
  // chunk input missions larger than the biggest capable ship (merged into the mission pool)
  {
    size_t n0 = w_.missions.size();
    for (size_t m = 0; m < n0; ++m) {
      Mission M = w_.missions[m];
      if (M.source != MissionSource::Input) continue;
      double cap = 0;
      for (const auto& sh : w_.ships)
        if (sh.alive && (sh.caps & M.requiredCaps) == M.requiredCaps) cap = std::max(cap, sh.cargoCap);
      if (cap <= 0 || M.amount <= cap) continue;
      int parts = static_cast<int>(std::ceil(M.amount / cap - 1e-9));
      double chunk = M.amount / parts;
      w_.missions[m].amount = chunk;
      for (int k = 1; k < parts; ++k) {
        Mission c = M;
        c.extId = 0;
        c.amount = chunk;
        c.parent = M.extId;
        w_.addMission(c);
      }
    }
  }
  queue_ = w_.events;
  queuePos_ = 0;
  Timer lt;
  if (cfg_.routing == RouteAlgo::ALT) {
    router_.setBoundsEnabled(true);  // ALT lower bounds are used for pruning in every mode
    router_.ensureAlt(PROFILE_CARGO);
    router_.ensureAlt(PROFILE_TIME);
    // meta-layer routing race on 32 seeded queries: deterministic work = settled nodes, with the
    // per-node ALT overhead (2 table lookups per landmark) folded in
    Rng prng(cfg_.seed ^ 0xA17ULL);
    int V = w_.graph.numNodes();
    double altW = 0, biW = 0;
    for (int q = 0; q < 32 && V > 1; ++q) {
      NodeId a = prng.uniformInt(V), b = prng.uniformInt(V);
      uint64_t s0 = counters().routingSettled;
      (void)router_.astar(a, b, PROFILE_CARGO);
      uint64_t s1 = counters().routingSettled;
      (void)router_.bidirectional(a, b, PROFILE_CARGO);
      uint64_t s2 = counters().routingSettled;
      altW += static_cast<double>(s1 - s0);
      biW += static_cast<double>(s2 - s1);
    }
    stats_.probeAltSettled = altW;
    stats_.probeBidirSettled = biW;
    double altCost = altW * (1.0 + router_.alt().landmarkCount() / 8.0);
    if (2.0 * biW < altCost) router_.setAlgo(RouteAlgo::Bidirectional);
  }
  stats_.routingChosen = routeAlgoName(router_.algo());
  stats_.landmarkMs = lt.ms();
  int day = 0;
  for (; day < days; ++day) {
    if (budget_.remainingMs() < budget_.reserveMs()) break;  // hard guard: keep the incumbent, stop
    Timer dayTimer;
    double t0 = day * HOURS_PER_DAY, t1 = t0 + HOURS_PER_DAY;
    w_.now = t0;
    sched_.setNow(t0);
    processArrivals(t0);
    completeFacilities(t0);
    expireMissions(t0);
    planCycle(day);
    while (queuePos_ < queue_.size() && queue_[queuePos_].time < t1) {
      Event ev = queue_[queuePos_++];
      double te = std::max(ev.time, t0);
      execute(te);
      processArrivals(te);
      w_.now = te;
      int s = (ev.type == EventType::ShipDestroyed || ev.type == EventType::ShipFailure) ? w_.shipOf(ev.target) : -1;
      if (s >= 0) handleShipLoss(s, te, ev.type == EventType::ShipDestroyed, ev.a > 0 ? ev.a : 24.0);
      double popBefore = 0;
      int pp = ev.type == EventType::PopulationSurge ? w_.planetOf(ev.target) : -1;
      if (pp >= 0) popBefore = w_.planets[pp].population;
      std::vector<Event> follow;
      {
        Features fe = features();
        fe.eventSeverity = ev.severity;
        Strategy se = meta_.choose(fe, budget_, SchedMethod::Auto);
        // urgent event (severity >= 0.8 with a mission due within 12 h): local repair with a tight box
        replan_.setTimeBoxMs(se.sched == SchedMethod::LocalRepair ? 0.5 * strat_.replanBoxMs : strat_.replanBoxMs);
      }
      replan_.setFullIterations(sched_.iterationsForBudget(strat_.replanBoxMs * 10, sched_.activeMissions().size()));
      double slotsNow = sched_.slots();
      double activeNow = static_cast<double>(sched_.activeMissions().size());
      ReplanRecord rec = replan_.handle(ev, te, follow, budget_.hard());
      if (ev.type != EventType::RouteRestore) counters().replanNaive += activeNow * slotsNow;
      if (pp >= 0) popExposed_ += std::max(0.0, w_.planets[pp].population - popBefore);
      if (ev.type == EventType::NewColony && !w_.planets.empty()) {
        popExposed_ += w_.planets.back().population;
        lostPop_.resize(w_.planets.size(), 0.0);
      }
      maxSeverity_ = std::max(maxSeverity_, ev.severity);
      replans_.push_back(rec);
      for (const Event& fu : follow) {
        auto it = std::upper_bound(queue_.begin() + static_cast<long>(queuePos_), queue_.end(), fu,
                                   [](const Event& a, const Event& b) { return a.time < b.time; });
        queue_.insert(it, fu);
      }
      if (!replan_.infraTriggers().empty()) {
        std::vector<NodeId> trig = replan_.infraTriggers();
        replan_.clearInfraTriggers();
        runInfrastructure(&trig, std::string("event:") + eventTypeName(ev.type), 0.25);
      }
    }
    execute(t1);
    processArrivals(t1);
    advancePlanets(t0, t1);
    w_.now = t1;
    lastDayMs_ = dayTimer.ms();
  }
  stats_.days = day;
  for (size_t i = queuePos_; i < queue_.size(); ++i)
    if (queue_[i].type != EventType::RouteRestore) stats_.eventsSkipped++;
  finalizeStats();
  return stats_;
}

void Simulator::finalizeStats() {
  Statistics& S = stats_;
  double endTime = w_.now;
  ScoreInputs in;
  for (const auto& M : w_.missions) {
    double wgt = M.weight();
    S.missionsTotal++;
    switch (M.status) {
      case MissionStatus::Completed:
        S.missionsCompleted++;
        in.missionWeightTotal += wgt;
        if (!M.late) {
          S.missionsOnTime++;
          in.missionWeightOnTime += wgt * std::min(1.0, M.amount > 0 ? M.delivered / M.amount : 1.0);
        } else {
          S.missionsLate++;
        }
        break;
      case MissionStatus::Failed:
        S.missionsFailed++;
        in.missionWeightTotal += wgt;
        in.missionWeightFailed += wgt;
        break;
      case MissionStatus::Expired:
        S.missionsExpired++;
        S.missionsFailed++;
        in.missionWeightTotal += wgt;
        in.missionWeightFailed += wgt;
        break;
      case MissionStatus::Cancelled:
        S.missionsCancelled++;
        break;
      default:  // pending / assigned / in flight at the end
        if (M.release <= endTime && M.deadline < endTime) {
          S.missionsExpired++;
          S.missionsFailed++;
          in.missionWeightTotal += wgt;
          in.missionWeightFailed += wgt;
        } else {
          S.missionsPendingAtEnd++;
        }
        break;
    }
  }
  S.resourcesDelivered = delivered_;
  S.resourceShortages = shortageUnits_;
  S.resourceWaste = wasteUnits_;
  S.totalDistance = distance_;
  S.totalFuel = fuelUsed_;
  S.totalTransportCost = transportCost_;
  S.totalRisk = riskSum_;
  const Counters& C = counters();
  S.nodesExplored = C.nodesExplored;
  S.candidatesEvaluated = C.candidatesEvaluated;
  S.statesExplored = C.statesExplored;
  S.routeQueries = C.routeQueries;
  S.cacheHits = C.cacheHits;
  S.cacheInvalidations = C.cacheInvalidations;
  S.labelsPruned = C.labelsPruned;
  S.candidatesPruned = C.candidatesPruned;
  S.dominatedShips = C.dominatedShips;
  S.bnbNodes = C.bnbNodes;
  S.altRebuilds = C.altRebuilds;
  // naive baseline = plain Dijkstra (measured on sampled queries) + all ship-mission slots
  // for every insertion request (counted) + a full regret re-insertion per event (counted)
  double ratio = C.samples > 0 && C.sampleActual > 0 ? C.sampleNaive / C.sampleActual : 1.0;
  S.naiveRoutingRatio = ratio;
  double routingNaive = static_cast<double>(C.routingSettled) * ratio;
  double naive = static_cast<double>(C.statesExplored) - static_cast<double>(C.routingSettled) + routingNaive -
                 static_cast<double>(C.insertionEvals) + C.naiveStates;
  if (cfg_.replanMode == ReplanMode::Incremental) naive += C.replanNaive;
  S.naiveStates = std::max(naive, static_cast<double>(C.statesExplored));
  S.searchSpaceReductionPct = S.naiveStates > 0 ? 100.0 * (1.0 - static_cast<double>(C.statesExplored) / S.naiveStates) : 0.0;
  int handled = 0;
  double sumMs = 0;
  for (const auto& r : replans_) {
    if (r.type == EventType::RouteRestore) continue;
    ++handled;
    sumMs += r.ms;
    S.maxReplanMs = std::max(S.maxReplanMs, r.ms);
  }
  S.eventsHandled = handled;
  S.avgReplanMs = handled ? sumMs / handled : 0.0;
  S.totalExecMs = cfg_.loadMs + clock_.ms();
  S.peakMemoryMB = peakMemoryMB();
  S.schedulerUsed = cycles_.empty() ? "none" : cycles_.back().strategy;
  // score inputs
  double popNow = 0, techEnd = 0, prodEnd = 0;
  int colonies = 0, established = 0;
  for (int p = 0; p < w_.numPlanets(); ++p) {
    const Planet& P = w_.planets[p];
    popNow += P.alive ? P.population : 0.0;
    techEnd += P.tech * P.strategic;
    for (int r = 0; r < NUM_RESOURCES; ++r) prodEnd += RESOURCE_IMPORTANCE[r] * P.production[r];
    if (P.type == PlanetType::Colony) {
      ++colonies;
      bool crit = false;
      for (int r = 0; r < NUM_RESOURCES; ++r)
        if (isCriticalResource(r) && P.shortageDays[r] > GRACE_DAYS) crit = true;
      bool reachable = false;
      w_.graph.forEachIn(p, [&](EdgeId e, NodeId) { reachable = reachable || w_.graph.available(e); });
      if (P.alive && !crit && reachable) ++established;
    }
  }
  in.popExposed = popExposed_;
  in.popSurvived = popNow;
  in.shortageWeighted = shortageW_;
  in.demandWeighted = demandW_;
  in.productionValueStart = prodValueStart_;
  in.productionValueEnd = prodEnd;
  in.techStart = techStart_;
  in.techEnd = techEnd;
  in.colonies = colonies;
  in.coloniesEstablished = established;
  in.transportCost = transportCost_;
  in.naiveTransportCost = naiveTransport_;
  in.riskSum = riskSum_;
  in.riskCount = riskCount_;
  in.wasteWeighted = wasteW_;
  in.producedWeighted = producedW_;
  in.fuelUsed = fuelUsed_;
  in.naiveFuel = naiveFuel_;
  in.execMs = S.totalExecMs;
  in.budgetMs = cfg_.timeMs;
  in.peakMB = S.peakMemoryMB;
  in.memBudgetMB = cfg_.memMB;
  score_ = computeScore(in);
  S.totalScore = score_.total;
  S.populationSurvivalRate = score_.populationSurvival;
}

// ------------------------------------------------------------------ outputs
static std::string statsToJson(const Statistics& S) {
  std::ostringstream o;
  auto num = [&](const char* k, double v, bool last = false) {
    o << "  \"" << k << "\": " << fmtDouble(v, 6) << (last ? "\n" : ",\n");
  };
  auto integer = [&](const char* k, uint64_t v) { o << "  \"" << k << "\": " << v << ",\n"; };
  o << "{\n";
  num("total_score", S.totalScore);
  num("population_survival_rate", S.populationSurvivalRate);
  num("resources_delivered", S.resourcesDelivered);
  num("resource_shortages", S.resourceShortages);
  num("resource_waste", S.resourceWaste);
  integer("missions_completed", static_cast<uint64_t>(S.missionsCompleted));
  integer("missions_failed", static_cast<uint64_t>(S.missionsFailed));
  num("total_distance", S.totalDistance);
  num("total_fuel", S.totalFuel);
  num("total_transport_cost", S.totalTransportCost);
  num("total_risk", S.totalRisk);
  integer("nodes_explored", S.nodesExplored);
  integer("candidates_evaluated", S.candidatesEvaluated);
  integer("states_explored", S.statesExplored);
  num("search_space_reduction_pct", S.searchSpaceReductionPct);
  integer("events_handled", static_cast<uint64_t>(S.eventsHandled));
  num("avg_replan_ms", S.avgReplanMs);
  num("total_exec_ms", S.totalExecMs);
  num("peak_memory_mb", S.peakMemoryMB);
  o << "  \"extra\": {\n";
  o << "    \"naive_states\": " << fmtDouble(S.naiveStates, 1) << ",\n";
  o << "    \"naive_routing_ratio_sampled\": " << fmtDouble(S.naiveRoutingRatio, 4) << ",\n";
  o << "    \"missions_total\": " << S.missionsTotal << ",\n";
  o << "    \"missions_on_time\": " << S.missionsOnTime << ",\n";
  o << "    \"missions_late\": " << S.missionsLate << ",\n";
  o << "    \"missions_expired\": " << S.missionsExpired << ",\n";
  o << "    \"missions_cancelled\": " << S.missionsCancelled << ",\n";
  o << "    \"missions_pending_at_end\": " << S.missionsPendingAtEnd << ",\n";
  o << "    \"proactive_missions\": " << S.proactiveMissions << ",\n";
  o << "    \"events_skipped\": " << S.eventsSkipped << ",\n";
  o << "    \"max_replan_ms\": " << fmtDouble(S.maxReplanMs, 3) << ",\n";
  o << "    \"load_ms\": " << fmtDouble(S.loadMs, 3) << ",\n";
  o << "    \"landmark_ms\": " << fmtDouble(S.landmarkMs, 3) << ",\n";
  o << "    \"validation_hard_violations\": " << S.validationHardViolations << ",\n";
  o << "    \"facilities_built\": " << S.facilitiesBuilt << ",\n";
  o << "    \"budget_spent\": " << fmtDouble(S.budgetSpent, 2) << ",\n";
  o << "    \"route_queries\": " << S.routeQueries << ",\n";
  o << "    \"cache_hits\": " << S.cacheHits << ",\n";
  o << "    \"cache_invalidations\": " << S.cacheInvalidations << ",\n";
  o << "    \"labels_pruned\": " << S.labelsPruned << ",\n";
  o << "    \"candidates_pruned\": " << S.candidatesPruned << ",\n";
  o << "    \"dominated_ships\": " << S.dominatedShips << ",\n";
  o << "    \"bnb_nodes\": " << S.bnbNodes << ",\n";
  o << "    \"alt_rebuilds\": " << S.altRebuilds << ",\n";
  o << "    \"alns_iterations\": " << S.alnsIterations << ",\n";
  o << "    \"days_simulated\": " << S.days << ",\n";
  o << "    \"routing_probe_alt_settled\": " << fmtDouble(S.probeAltSettled, 0) << ",\n";
  o << "    \"routing_probe_bidirectional_settled\": " << fmtDouble(S.probeBidirSettled, 0) << ",\n";
  o << "    \"routing_algorithm\": \"" << jsonEscape(S.routingChosen) << "\",\n";
  o << "    \"replan_mode\": \"" << jsonEscape(S.replanMode) << "\",\n";
  o << "    \"scheduler_last_cycle\": \"" << jsonEscape(S.schedulerUsed) << "\"\n";
  o << "  }\n}\n";
  return o.str();
}

void Simulator::writeOutputs(const std::string& dir) const {
  ensureDirectory(dir);
  const World& w = w_;
  auto P = [&](NodeId v) { return v >= 0 && v < w.numPlanets() ? w.planets[v].extId : -1; };
  auto Sh = [&](int s) { return s >= 0 && s < static_cast<int>(w.ships.size()) ? w.ships[s].extId : -1; };
  // 2) spacecraft assignments
  {
    std::ostringstream o;
    o << "ship_id,mission_id,seq_index,planned_start_h,planned_depart_h,planned_arrive_h,actual_depart_h,"
         "actual_arrive_h,status\n";
    for (const auto& M : w.missions) {
      int s = M.executedBy >= 0 ? M.executedBy : (M.plan.valid ? M.plan.ship : -1);
      if (s < 0) continue;
      o << Sh(s) << ',' << M.extId << ',' << M.plan.seqIndex << ',' << fmtDouble(M.plan.plannedStart, 2) << ','
        << fmtDouble(M.plan.depart, 2) << ',' << fmtDouble(M.plan.arrive, 2) << ','
        << (M.executedBy >= 0 ? fmtDouble(M.actualDepart, 2) : "") << ','
        << (M.executedBy >= 0 ? fmtDouble(M.actualArrive, 2) : "") << ',' << missionStatusName(M.status) << '\n';
    }
    writeTextFile(joinPath(dir, "spacecraft_assignments.csv"), o.str());
  }
  // 3) resource allocation plan
  {
    std::ostringstream o;
    o << "day,shipment_id,resource,from_planet,to_planet,amount,unit_cost,deadline_h,path_time_h,path_sigma_h,active\n";
    for (size_t i = 0; i < shipmentLog_.size(); ++i) {
      const auto& sh = shipmentLog_[i];
      o << shipmentDay_[i] << ',' << sh.id << ',' << resourceName(sh.resource) << ',' << P(sh.from) << ',' << P(sh.to)
        << ',' << fmtDouble(sh.amount, 2) << ',' << fmtDouble(sh.unitCost, 4) << ',' << fmtDouble(sh.deadline, 2) << ','
        << fmtDouble(sh.pathTime, 2) << ',' << fmtDouble(sh.pathSigma, 2) << ',' << (sh.active ? 1 : 0) << '\n';
    }
    writeTextFile(joinPath(dir, "resource_allocation.csv"), o.str());
  }
  // 4) mission schedule
  {
    std::ostringstream o;
    o << "mission_id,source,origin,destination,resource,amount,release_h,deadline_h,priority,importance,ship_id,"
         "planned_depart_h,planned_arrive_h,p_on_time,actual_depart_h,actual_arrive_h,delivered,status,late,delayed\n";
    for (const auto& M : w.missions) {
      int s = M.executedBy >= 0 ? M.executedBy : (M.plan.valid ? M.plan.ship : -1);
      bool delayed = M.plan.valid && (M.plan.depart > M.release + 24.0 || M.plan.arrive > M.deadline);
      o << M.extId << ',' << missionSourceName(M.source) << ',' << P(M.origin) << ',' << P(M.dest) << ','
        << resourceName(M.resource) << ',' << fmtDouble(M.amount, 2) << ',' << fmtDouble(M.release, 2) << ','
        << fmtDouble(M.deadline, 2) << ',' << M.priority << ',' << fmtDouble(M.importance, 3) << ',' << Sh(s) << ','
        << (M.plan.valid || M.executedBy >= 0 ? fmtDouble(M.plan.depart, 2) : "") << ','
        << (M.plan.valid || M.executedBy >= 0 ? fmtDouble(M.plan.arrive, 2) : "") << ','
        << fmtDouble(M.plan.pOnTime, 4) << ',' << (M.executedBy >= 0 ? fmtDouble(M.actualDepart, 2) : "") << ','
        << (M.executedBy >= 0 ? fmtDouble(M.actualArrive, 2) : "") << ',' << fmtDouble(M.delivered, 2) << ','
        << missionStatusName(M.status) << ',' << (M.late ? 1 : 0) << ',' << (delayed ? 1 : 0) << '\n';
    }
    writeTextFile(joinPath(dir, "mission_schedule.csv"), o.str());
  }
  // 5) route selections
  {
    std::ostringstream o;
    o << "mission_id,leg,profile,algorithm,nodes,distance,fuel,time_mean_h,time_sigma_h,risk,reliability,refuel_stops\n";
    for (const auto& M : w.missions) {
      if (!(M.executedBy >= 0 || M.plan.valid)) continue;
      for (int leg = 0; leg < 2; ++leg) {
        const auto& edges = leg ? M.plan.loadedPath : M.plan.emptyPath;
        int prof = leg ? M.plan.profile : PROFILE_CARGO;
        Path p;
        p.edges = edges;
        fillPathMetrics(w.graph, p, prof);
        NodeId start = edges.empty() ? M.origin : w.graph.from(edges.front());
        std::string algo = leg && !M.fixedPath.empty() ? "pareto_labels" : routeAlgoName(router_.algo());
        if (!M.plan.refuelNodes.empty()) algo += "+fuel_labels";
        o << M.extId << ',' << (leg ? "loaded" : "empty") << ',' << costProfile(prof).name << ',' << algo << ','
          << extPath(w, edges, start) << ',' << fmtDouble(p.dist, 2) << ',' << fmtDouble(p.fuel, 2) << ','
          << fmtDouble(p.timeMean, 2) << ',' << fmtDouble(std::sqrt(p.timeVar), 2) << ',' << fmtDouble(p.risk, 4)
          << ',' << fmtDouble(p.reliability, 4) << ',' << M.plan.refuelNodes.size() << '\n';
      }
    }
    writeTextFile(joinPath(dir, "route_selections.csv"), o.str());
  }
  // 6) infrastructure decisions
  {
    std::ostringstream o;
    o << "option_id,planet,type,level,cost,start_h,complete_h,value,method,trigger,built\n";
    for (const auto& r : infraLog_) {
      const auto& f = w.facilities[r.option];
      o << f.extId << ',' << P(f.planet) << ',' << facilityTypeName(f.type) << ',' << f.level << ','
        << fmtDouble(f.cost, 2) << ',' << fmtDouble(r.start, 2) << ',' << fmtDouble(f.completeAt, 2) << ','
        << fmtDouble(r.value, 3) << ',' << r.method << ',' << r.trigger << ',' << (f.built ? 1 : 0) << '\n';
    }
    writeTextFile(joinPath(dir, "infrastructure_decisions.csv"), o.str());
  }
  // 7) predicted shortages
  {
    std::ostringstream o;
    o << "day,planet,resource,stock,net_rate_per_day,time_to_shortage_h,time_to_overflow_h,lead_time_h,"
         "delivery_deadline_h,need,covered\n";
    for (const auto& r : predictions_) {
      const auto& p = r.p;
      o << r.day << ',' << P(p.planet) << ',' << resourceName(p.resource) << ',' << fmtDouble(p.stock, 2) << ','
        << fmtDouble(p.netRatePerDay, 3) << ',' << fmtDouble(p.timeToShortageH, 2) << ','
        << fmtDouble(p.timeToOverflowH, 2) << ',' << fmtDouble(p.leadTimeH, 2) << ','
        << fmtDouble(p.deliveryDeadlineH, 2) << ',' << fmtDouble(p.need, 2) << ',' << (r.covered ? 1 : 0) << '\n';
    }
    writeTextFile(joinPath(dir, "predicted_shortages.csv"), o.str());
  }
  // 8) re-planning log
  {
    std::ostringstream o;
    o << "event_index,time_h,type,target,mode,invalid,created,cancelled,removed,reinserted,unchanged,active,ms,"
         "cost_before,cost_after,escalated,note\n";
    for (const auto& r : replans_)
      o << r.eventIndex << ',' << fmtDouble(r.time, 2) << ',' << eventTypeName(r.type) << ',' << r.target << ','
        << replanModeName(r.mode) << ',' << r.invalid << ',' << r.created << ',' << r.cancelled << ',' << r.removed
        << ',' << r.reinserted << ',' << r.unchanged << ',' << r.active << ',' << fmtDouble(r.ms, 4) << ','
        << fmtDouble(r.costBefore, 2) << ',' << fmtDouble(r.costAfter, 2) << ',' << (r.escalated ? 1 : 0) << ','
        << r.note << '\n';
    writeTextFile(joinPath(dir, "replanning_log.csv"), o.str());
  }
  // 9) score, 10) statistics
  writeTextFile(joinPath(dir, "score.json"), scoreToJson(score_));
  writeTextFile(joinPath(dir, "stats.json"), statsToJson(stats_));
  // extras: per-cycle meta decisions and ALNS convergence
  {
    std::ostringstream o;
    o << "day,strategy,active,created,assigned,shipments,predicted,alns_iters,alloc_ms,sched_ms,infra_ms,cost,"
         "alloc_objective,hard_violations,late,reason\n";
    for (const auto& c : cycles_)
      o << c.day << ',' << c.strategy << ',' << c.active << ',' << c.created << ',' << c.assigned << ',' << c.shipments
        << ',' << c.predicted << ',' << c.alnsIters << ',' << fmtDouble(c.allocMs, 3) << ','
        << fmtDouble(c.schedMs, 3) << ',' << fmtDouble(c.infraMs, 3) << ',' << fmtDouble(c.cost, 2) << ','
        << fmtDouble(c.allocObjective, 2) << ',' << c.hardViolations << ',' << c.late << ",\"" << c.reason << "\"\n";
    writeTextFile(joinPath(dir, "cycles.csv"), o.str());
    std::ostringstream cv;
    cv << "ms,iteration,best_cost,current_cost\n";
    for (const auto& p : convergence_)
      cv << fmtDouble(p.ms, 3) << ',' << p.iter << ',' << fmtDouble(p.best, 3) << ',' << fmtDouble(p.current, 3) << '\n';
    writeTextFile(joinPath(dir, "convergence.csv"), cv.str());
  }
  // 1) civilization plan (human-readable summary of every decision layer)
  {
    std::ostringstream o;
    o << "# Civilization Plan\n\n";
    o << "Galaxy: " << w.graph.numNodes() << " locations, " << w.graph.numEdges() << " directed routes, "
      << w.ships.size() << " spacecraft, " << w.missions.size() << " missions (input + generated). Days simulated: "
      << stats_.days << ". Re-planning mode: " << stats_.replanMode << ".\n\n";
    o << "## Score\n\nTotal civilization score: " << fmtDouble(score_.total, 3)
      << " (positive " << fmtDouble(score_.positive, 3) << ", penalties " << fmtDouble(score_.negative, 3)
      << "). Population survival " << fmtDouble(100 * score_.populationSurvival, 2) << "%, resource security "
      << fmtDouble(100 * score_.resourceSecurity, 2) << "%, mission success " << fmtDouble(100 * score_.missionSuccess, 2)
      << "%.\n\n";
    o << "## Meta-optimizer decisions per cycle\n\n| day | scheduler | active | assigned | shipments | reason |\n"
         "|---|---|---|---|---|---|\n";
    for (const auto& c : cycles_)
      o << "| " << c.day << " | " << c.strategy << " | " << c.active << " | " << c.assigned << " | " << c.shipments
        << " | " << c.reason << " |\n";
    o << "\n## Resource allocation\n\n" << shipmentLog_.size()
      << " shipments planned by the reduced min-cost flow (see resource_allocation.csv); "
      << stats_.proactiveMissions << " proactive missions were generated from them.\n\n";
    o << "## Infrastructure\n\n" << infraLog_.size() << " facilities started, " << stats_.facilitiesBuilt
      << " completed, budget spent " << fmtDouble(stats_.budgetSpent, 2) << ", remaining " << fmtDouble(w.budget, 2)
      << " (see infrastructure_decisions.csv).\n\n";
    o << "## Missions\n\nCompleted " << stats_.missionsCompleted << " (on time " << stats_.missionsOnTime << ", late "
      << stats_.missionsLate << "), failed or expired " << stats_.missionsFailed << ", cancelled by re-planning "
      << stats_.missionsCancelled << ", still pending " << stats_.missionsPendingAtEnd << ".\n\n";
    o << "## Dynamic events\n\n" << stats_.eventsHandled << " events handled (average re-plan "
      << fmtDouble(stats_.avgReplanMs, 3) << " ms, max " << fmtDouble(stats_.maxReplanMs, 3) << " ms), "
      << stats_.eventsSkipped << " beyond the simulated horizon (see replanning_log.csv).\n";
    writeTextFile(joinPath(dir, "civilization_plan.md"), o.str());
  }
}

}  // namespace chronos
