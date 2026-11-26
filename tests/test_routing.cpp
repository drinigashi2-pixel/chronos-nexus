// test_routing.cpp - ALT A* / bidirectional == Dijkstra (also after dynamic edge changes),
// fuel-constrained labels, Pareto labels and targeted cache invalidation.
#include "chronos/routing.hpp"
#include "chronos/scheduler.hpp"

#include <unordered_map>
#include "test_util.hpp"

using namespace chronos;

namespace {
// Compare the three exact algorithms on random pairs for every profile.
int compareAll(Router& r, const World& w, Rng& rng, int pairs) {
  int mismatches = 0;
  for (int p = 0; p < NUM_PROFILES; ++p) {
    for (int k = 0; k < pairs; ++k) {
      NodeId s = rng.uniformInt(w.numPlanets()), t = rng.uniformInt(w.numPlanets());
      Path d = r.dijkstra(s, t, p);
      Path a = r.astar(s, t, p);
      Path b = r.bidirectional(s, t, p);
      CHECK(d.found == a.found);
      CHECK(d.found == b.found);
      if (d.found && a.found && b.found) {
        double tol = 1e-6 * std::max(1.0, d.cost);
        if (std::fabs(d.cost - a.cost) > tol || std::fabs(d.cost - b.cost) > tol) ++mismatches;
        CHECK(pathUsable(w.graph, a.edges, 0.0));
        CHECK(a.edges.empty() || (w.graph.from(a.edges.front()) == s && w.graph.to(a.edges.back()) == t));
      }
    }
  }
  return mismatches;
}

void mutate(World& w, Router& r, EdgeId e, int kind, Rng& rng) {
  auto old = r.edgeCosts(e);
  if (kind == 0) w.graph.setRisk(e, static_cast<float>(std::min(0.9, w.graph.risk(e) + rng.uniform(0.1, 0.5))));
  else if (kind == 1) w.graph.setAvailable(e, false);
  else if (kind == 2) w.graph.setAvailable(e, true);
  else w.graph.setRisk(e, static_cast<float>(w.graph.risk(e) * 0.2));  // decrease
  r.onEdgeChanged(e, old);
}
}  // namespace

TEST(routing_alt_equals_dijkstra_random_graphs) {
  for (uint64_t seed = 1; seed <= 4; ++seed) {
    World w = testing::makeWorld(250, 1200, seed, false, true);
    Router r(w.graph);
    r.setAlgo(RouteAlgo::ALT);
    r.alt().setLandmarkCount(8);
    for (int p = 0; p < NUM_PROFILES; ++p) r.ensureAlt(p);
    Rng rng(seed * 11);
    CHECK(compareAll(r, w, rng, 40) == 0);
  }
}

TEST(routing_alt_exact_after_edge_increases) {
  World w = testing::makeWorld(200, 900, 7);
  Router r(w.graph);
  r.alt().setLandmarkCount(8);
  for (int p = 0; p < NUM_PROFILES; ++p) r.ensureAlt(p);
  Rng rng(3);
  for (int k = 0; k < 120; ++k) mutate(w, r, rng.uniformInt(w.graph.numEdges()), rng.uniformInt(2), rng);
  // increases never make the stored landmark distances inadmissible: no rebuild needed
  for (int p = 0; p < NUM_PROFILES; ++p) CHECK(!r.alt().stale(p));
  CHECK(compareAll(r, w, rng, 40) == 0);
}

TEST(routing_alt_exact_after_decreases_and_new_edges) {
  World w = testing::makeWorld(200, 900, 9);
  Router r(w.graph);
  r.alt().setLandmarkCount(8);
  for (int p = 0; p < NUM_PROFILES; ++p) r.ensureAlt(p);
  Rng rng(5);
  // decreases accumulate an admissible slack; restores of edges usable at build time keep tables valid
  for (int k = 0; k < 40; ++k) mutate(w, r, rng.uniformInt(w.graph.numEdges()), 1, rng);
  for (int k = 0; k < 40; ++k) mutate(w, r, rng.uniformInt(w.graph.numEdges()), 2 + rng.uniformInt(2), rng);
  CHECK(compareAll(r, w, rng, 30) == 0);
  // a brand-new edge is repaired in place: tables stay valid (no stale fallback, no rebuild)
  EdgeAttr a = w.graph.attr(0);
  a.from = 3;
  a.to = 150;
  a.dist = 0.5f;
  a.tmean = a.tmin = a.tmax = 0.5f;
  a.fuel = 0.1f;
  a.risk = 0;
  a.rel = 1;
  EdgeId e = w.graph.addEdge(a);
  r.onEdgeAdded(e);
  CHECK(!r.alt().stale(PROFILE_BALANCED));
  CHECK(compareAll(r, w, rng, 20) == 0);
  CHECK(r.alt().lowerBound(PROFILE_BALANCED, 3, 150) <= r.dijkstra(3, 150, PROFILE_BALANCED).cost + 1e-9);
  r.maintain();
  CHECK(!r.alt().stale(PROFILE_BALANCED));
  CHECK(compareAll(r, w, rng, 20) == 0);
}

TEST(routing_alt_repair_new_node_and_reopened_edges) {
  World w = testing::makeWorld(220, 1000, 13);
  // close some edges before the tables are built: reopening them used to make the tables stale
  std::vector<EdgeId> closed;
  for (EdgeId e = 0; e < w.graph.numEdges(); e += 37) {
    w.graph.setAvailable(e, false);
    closed.push_back(e);
  }
  Router r(w.graph);
  r.setAlgo(RouteAlgo::ALT);
  r.alt().setLandmarkCount(8);
  for (int p = 0; p < NUM_PROFILES; ++p) r.ensureAlt(p);
  Rng rng(17);
  for (EdgeId e : closed) {
    auto old = r.edgeCosts(e);
    w.graph.setAvailable(e, true);
    r.onEdgeChanged(e, old);
  }
  for (int p = 0; p < NUM_PROFILES; ++p) CHECK(!r.alt().stale(p));
  CHECK(compareAll(r, w, rng, 30) == 0);
  // new colony: a node with short two-way links to three existing nodes (a shortcut hub)
  NodeId v = w.graph.addNode();
  r.onNodeAdded();
  for (NodeId u : {5, 60, 140})
    for (int dir = 0; dir < 2; ++dir) {
      EdgeAttr a = w.graph.attr(0);
      a.from = dir ? u : v;
      a.to = dir ? v : u;
      a.dist = 0.3f;
      a.tmean = a.tmin = a.tmax = 0.3f;
      a.fuel = 0.1f;
      a.risk = 0;
      a.rel = 1;
      a.cap = 1000.0f;
      r.onEdgeAdded(w.graph.addEdge(a));
    }
  for (int p = 0; p < NUM_PROFILES; ++p) CHECK(!r.alt().stale(p));
  CHECK(compareAll(r, w, rng, 30) == 0);
  for (int p = 0; p < NUM_PROFILES; ++p) {
    Path d1 = r.dijkstra(5, v, p), a1 = r.astar(5, v, p), d2 = r.dijkstra(v, 140, p), a2 = r.astar(v, 140, p);
    CHECK(d1.found && a1.found && std::fabs(d1.cost - a1.cost) <= 1e-6 * std::max(1.0, d1.cost));
    CHECK(d2.found && a2.found && std::fabs(d2.cost - a2.cost) <= 1e-6 * std::max(1.0, d2.cost));
  }
}

TEST(routing_cache_exact_after_decreases_and_new_edges) {
  World w = testing::makeWorld(200, 900, 23);
  Router r(w.graph);
  r.setAlgo(RouteAlgo::ALT);
  r.alt().setLandmarkCount(8);
  for (int p = 0; p < NUM_PROFILES; ++p) r.ensureAlt(p);
  Rng rng(29);
  std::vector<std::pair<NodeId, NodeId>> q;
  for (int k = 0; k < 150; ++k) q.push_back({rng.uniformInt(w.numPlanets()), rng.uniformInt(w.numPlanets())});
  auto warm = [&]() {
    for (auto [s, t] : q)
      for (int p = 0; p < NUM_PROFILES; ++p) r.route(s, t, p);
  };
  auto check = [&]() {
    int bad = 0;
    for (auto [s, t] : q)
      for (int p = 0; p < NUM_PROFILES; ++p) {
        Path c = r.route(s, t, p);  // copy: possibly from the cache
        Path d = r.dijkstra(s, t, p);
        if (c.found != d.found || (d.found && std::fabs(c.cost - d.cost) > 1e-6 * std::max(1.0, d.cost))) ++bad;
      }
    return bad;
  };
  warm();
  size_t before = r.cache().size();
  for (int k = 0; k < 30; ++k) mutate(w, r, rng.uniformInt(w.graph.numEdges()), 3, rng);  // decreases
  CHECK(r.cache().size() > 0);
  CHECK(r.cache().size() <= before);
  CHECK(check() == 0);
  warm();
  for (int k = 0; k < 10; ++k) {  // new shortcut edges
    EdgeAttr a = w.graph.attr(0);
    a.from = rng.uniformInt(w.numPlanets());
    a.to = rng.uniformInt(w.numPlanets());
    if (a.from == a.to) continue;
    a.dist = 0.4f;
    a.tmean = a.tmin = a.tmax = 0.4f;
    a.fuel = 0.1f;
    a.risk = 0;
    a.rel = 1;
    a.cap = 1000.0f;
    r.onEdgeAdded(w.graph.addEdge(a));
  }
  CHECK(check() == 0);
}

TEST(routing_lower_bound_admissible) {
  World w = testing::makeWorld(150, 700, 21);
  Router r(w.graph);
  r.alt().setLandmarkCount(6);
  r.ensureAlt(PROFILE_TIME);
  Rng rng(8);
  std::vector<double> dist;
  for (int k = 0; k < 10; ++k) {
    NodeId s = rng.uniformInt(w.numPlanets());
    r.fullDijkstra(s, PROFILE_TIME, false, dist);
    for (NodeId t = 0; t < w.numPlanets(); ++t)
      if (std::isfinite(dist[t])) CHECK(r.lowerBound(s, t, PROFILE_TIME) <= dist[t] + 1e-6);
  }
}

TEST(routing_fuel_constrained_labels) {
  World w = testing::makeWorld(120, 500, 31, true);
  Router r(w.graph);
  std::vector<uint8_t> refuel(w.numPlanets(), 0);
  for (int v = 0; v < w.numPlanets(); v += 5) refuel[v] = 1;
  Rng rng(4);
  int found = 0;
  for (int k = 0; k < 40; ++k) {
    NodeId s = rng.uniformInt(w.numPlanets()), t = rng.uniformInt(w.numPlanets());
    if (s == t) continue;
    double cap = 60.0;
    Path p = r.routeFuel(s, t, PROFILE_CARGO, cap, cap, refuel, 0, 32, 5.0);
    Path free = r.dijkstra(s, t, PROFILE_CARGO);
    if (!p.found) continue;
    ++found;
    CHECK(p.cost + 1e-9 >= free.cost);
    // simulate the fuel tank along the path with the returned refuel stops
    double fuel = cap;
    size_t ri = 0;
    NodeId v = s;
    bool ok = true;
    auto refuelAt = [&](NodeId x) {
      while (ri < p.refuelNodes.size() && p.refuelNodes[ri] == x) {
        fuel = cap;
        ++ri;
      }
    };
    refuelAt(v);
    for (EdgeId e : p.edges) {
      fuel -= w.graph.fuel(e);
      ok = ok && fuel >= -1e-6;
      v = w.graph.to(e);
      refuelAt(v);
    }
    CHECK(ok);
    CHECK(v == t);
  }
  CHECK(found > 0);
}

TEST(routing_pareto_front_valid) {
  World w = testing::makeWorld(150, 700, 41);
  Router r(w.graph);
  r.setBoundsEnabled(true);
  Rng rng(6);
  for (int k = 0; k < 10; ++k) {
    NodeId s = rng.uniformInt(w.numPlanets()), t = rng.uniformInt(w.numPlanets());
    if (s == t) continue;
    auto front = r.paretoPaths(s, t, 0, 6, 0.05);
    CHECK(!front.empty());
    for (const auto& p : front) {
      CHECK(!p.edges.empty() && w.graph.from(p.edges.front()) == s && w.graph.to(p.edges.back()) == t);
      CHECK(pathUsable(w.graph, p.edges, 0.0));
    }
    // keys used by the label search: time = sum(mean + 2 sigma), risk = sum(risk - ln rel)
    std::vector<std::pair<double, double>> keys;
    for (const auto& p : front) {
      double tt = 0, rk = 0;
      for (EdgeId e : p.edges) {
        tt += w.graph.tmean(e) + 2.0 * w.graph.sigma(e);
        rk += w.graph.risk(e) - std::log(static_cast<double>(w.graph.rel(e)));
      }
      keys.push_back({tt, rk});
    }
    // no member of the returned front is dominated by another member
    for (size_t i = 0; i < keys.size(); ++i)
      for (size_t j = 0; j < keys.size(); ++j)
        if (i != j) CHECK(!(keys[i].first < keys[j].first - 1e-9 && keys[i].second < keys[j].second - 1e-9));
  }
}

TEST(routing_cache_targeted_invalidation) {
  World w = testing::makeWorld(200, 900, 51);
  Router r(w.graph);
  Rng rng(9);
  for (int k = 0; k < 30; ++k) {
    NodeId s = rng.uniformInt(w.numPlanets()), t = rng.uniformInt(w.numPlanets());
    if (s == t) continue;
    Path first = r.route(s, t, PROFILE_BALANCED);
    if (first.edges.empty()) continue;
    EdgeId e = first.edges[first.edges.size() / 2];
    auto old = r.edgeCosts(e);
    w.graph.setAvailable(e, false);
    int dropped = r.onEdgeChanged(e, old);
    CHECK(dropped >= 1);
    Path again = r.route(s, t, PROFILE_BALANCED);
    Path truth = r.dijkstra(s, t, PROFILE_BALANCED);
    CHECK(again.found == truth.found);
    if (truth.found) CHECK_NEAR(again.cost, truth.cost, 1e-6);
    for (EdgeId x : again.edges) CHECK(x != e);
    auto old2 = r.edgeCosts(e);
    w.graph.setAvailable(e, true);
    r.onEdgeChanged(e, old2);
  }
}

TEST(flat_index_matches_unordered_map) {
  // FlatIndex (open addressing, backward-shift deletion) against std::unordered_map under random
  // put/overwrite/erase/find, with many collisions (small key range) and table growth
  FlatIndex fi;
  std::unordered_map<uint64_t, int32_t> ref;
  Rng rng(99);
  int bad = 0;
  for (int k = 0; k < 200000; ++k) {
    uint64_t key = static_cast<uint64_t>(rng.uniformInt(5000)) * 0x9E3779B97F4A7C15ULL;
    int op = rng.uniformInt(10);
    if (op < 5) {
      int32_t v = rng.uniformInt(1 << 20);
      fi.put(key, v);
      ref[key] = v;
    } else if (op < 8) {
      int32_t a = fi.erase(key);
      auto it = ref.find(key);
      int32_t b = it == ref.end() ? -1 : it->second;
      if (it != ref.end()) ref.erase(it);
      bad += a != b;
    } else {
      auto it = ref.find(key);
      bad += fi.find(key) != (it == ref.end() ? -1 : it->second);
    }
    if (k == 150000) {
      fi.clear();
      ref.clear();
    }
  }
  CHECK(bad == 0);
  CHECK(fi.size() == ref.size());
  size_t seen = 0;
  fi.forEach([&](uint64_t key, int32_t v) {
    ++seen;
    auto it = ref.find(key);
    bad += it == ref.end() || it->second != v;
  });
  CHECK(bad == 0);
  CHECK(seen == ref.size());
}

TEST(scheduler_refuel_along_route_feasible) {
  // every leg the refuel-along-route shortcut returns must be drivable with the stated stops, use
  // only refuel nodes, follow the best route, and cost it plus the refuel fees
  World w = testing::makeWorld(150, 600, 41, true);
  for (int v = 0; v < w.numPlanets(); ++v) w.planets[v].stock[static_cast<int>(Resource::Fuel)] = v % 3 ? 0.0 : 100.0;
  Router r(w.graph);
  SchedulerConfig cfg;
  Scheduler s(w, r, cfg, 5);
  s.syncWithWorld();
  const std::vector<uint8_t> refuel = w.refuelMask();
  Rng rng(6);
  int found = 0;
  for (int k = 0; k < 200; ++k) {
    NodeId a = rng.uniformInt(w.numPlanets()), b = rng.uniformInt(w.numPlanets());
    if (a == b) continue;
    double cap = 40.0 + rng.uniform() * 60.0, start = rng.uniform() * cap;
    Path p = s.refuelAlongRoute(a, b, PROFILE_CARGO, 0, start, cap);
    if (!p.found) continue;
    ++found;
    Path best = r.dijkstra(a, b, PROFILE_CARGO);
    CHECK(std::fabs(p.cost - (best.cost + cfg.refuelCost * p.refuelNodes.size())) <= 1e-6 * std::max(1.0, p.cost));
    double fuel = start;
    size_t ri = 0;
    bool ok = true;
    for (EdgeId e : p.edges) {
      NodeId u = w.graph.from(e);
      if (ri < p.refuelNodes.size() && p.refuelNodes[ri] == u) {
        ok = ok && refuel[u];
        fuel = cap;
        ++ri;
      }
      fuel -= w.graph.fuel(e);
      ok = ok && fuel >= -1e-6;
    }
    CHECK(ok);
    CHECK(ri == p.refuelNodes.size());
    CHECK(std::fabs(fuel - p.fuelAtArrival) <= 1e-6);
    CHECK(!p.edges.empty() && w.graph.to(p.edges.back()) == b);
  }
  CHECK(found > 20);
}
