// test_util.hpp - minimal dependency-free test framework + world builders shared by the tests.
#pragma once
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include "chronos/graph.hpp"

namespace testing {

struct TestCase {
  const char* name;
  std::function<void()> fn;
};
inline std::vector<TestCase>& registry() {
  static std::vector<TestCase> r;
  return r;
}
inline int& failures() {
  static int f = 0;
  return f;
}
inline int& checks() {
  static int c = 0;
  return c;
}
struct Registrar {
  Registrar(const char* n, std::function<void()> f) { registry().push_back({n, std::move(f)}); }
};

}  // namespace testing

#define TEST_CAT2(a, b) a##b
#define TEST_CAT(a, b) TEST_CAT2(a, b)
#define TEST(name)                                                                        \
  static void name();                                                                     \
  static testing::Registrar TEST_CAT(reg_, name)(#name, name);                            \
  static void name()

#define CHECK(cond)                                                                       \
  do {                                                                                    \
    ++testing::checks();                                                                  \
    if (!(cond)) {                                                                        \
      ++testing::failures();                                                              \
      std::printf("  FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);                    \
    }                                                                                     \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                             \
  do {                                                                                    \
    ++testing::checks();                                                                  \
    double _a = (a), _b = (b);                                                            \
    if (!(std::fabs(_a - _b) <= (tol) * std::max(1.0, std::fabs(_b)))) {                  \
      ++testing::failures();                                                              \
      std::printf("  FAILED %s:%d: %s = %.10g vs %s = %.10g\n", __FILE__, __LINE__, #a, _a, \
                  #b, _b);                                                                \
    }                                                                                     \
  } while (0)

namespace testing {

// Random strongly connected graph world: a bidirectional ring guarantees connectivity, plus
// random extra directed edges. `metric` makes all edge metrics proportional to the distance
// (time = dist, fuel = 0.5 dist, risk 0, reliability 1) so that cost/time obey the triangle
// inequality; `zeroWeights` makes ~10% of the extra edges free.
inline chronos::World makeWorld(int n, int extraEdges, uint64_t seed, bool metric = false, bool zeroWeights = false) {
  using namespace chronos;
  World w;
  Rng rng(seed);
  for (int i = 0; i < n; ++i) {
    Planet p;
    p.extId = i;
    double ang = 6.283185307179586 * i / n;
    p.x = 100 * std::cos(ang) + rng.uniform(-5, 5);
    p.y = 100 * std::sin(ang) + rng.uniform(-5, 5);
    p.population = rng.uniform(1e4, 1e6);
    p.initialPopulation = p.population;
    p.strategic = rng.uniform(0.5, 2.0);
    p.type = i % 7 == 0 ? PlanetType::Station : PlanetType::Planet;
    for (int r = 0; r < NUM_RESOURCES; ++r) {
      p.consumption[r] = rng.uniform(5, 20);
      bool prod = rng.uniform() < 0.45;
      p.production[r] = p.consumption[r] * (prod ? rng.uniform(1.5, 3.0) : rng.uniform(0.2, 0.9));
      p.capacity[r] = 60 * std::max(p.production[r], p.consumption[r]);
      p.stock[r] = p.consumption[r] * rng.uniform(3, 25);
    }
    p.stock[static_cast<int>(Resource::Fuel)] = 500;  // every planet can refuel
    w.planets.push_back(p);
    w.planetIdx[i] = i;
  }
  std::vector<EdgeAttr> edges;
  int64_t rid = 0;
  auto mk = [&](int u, int v, bool zero) {
    EdgeAttr a;
    a.from = u;
    a.to = v;
    double d = std::hypot(w.planets[u].x - w.planets[v].x, w.planets[u].y - w.planets[v].y) + 1.0;
    if (zero) d = 0;
    a.dist = static_cast<float>(d);
    if (metric) {
      a.tmean = a.tmin = a.tmax = static_cast<float>(d);
      a.fuel = static_cast<float>(0.5 * d);
      a.risk = 0;
      a.rel = 1;
    } else {
      a.tmean = static_cast<float>(zero ? 0 : d / rng.uniform(1.5, 3));
      a.tmin = a.tmean * 0.8f;
      a.tmax = a.tmean * static_cast<float>(rng.uniform(1.1, 1.6));
      a.fuel = static_cast<float>(zero ? 0 : d * rng.uniform(0.5, 1.0));
      a.risk = static_cast<float>(zero ? 0 : std::pow(rng.uniform(), 3) * 0.3);
      a.rel = static_cast<float>(zero ? 1 : 1 - rng.uniform() * 0.05);
    }
    a.cap = 5000;
    return a;
  };
  for (int i = 0; i < n; ++i) {
    int j = (i + 1) % n;
    EdgeAttr a = mk(i, j, false), b = mk(j, i, false);
    a.routeId = b.routeId = rid;
    w.routeEdges[rid].push_back(static_cast<EdgeId>(edges.size()));
    edges.push_back(a);
    w.routeEdges[rid].push_back(static_cast<EdgeId>(edges.size()));
    edges.push_back(b);
    ++rid;
  }
  for (int k = 0; k < extraEdges; ++k) {
    int u = rng.uniformInt(n), v = rng.uniformInt(n);
    if (u == v) continue;
    EdgeAttr a = mk(u, v, zeroWeights && rng.uniform() < 0.1);
    a.routeId = rid;
    w.routeEdges[rid].push_back(static_cast<EdgeId>(edges.size()));
    edges.push_back(a);
    ++rid;
  }
  w.nextRouteExt = rid;
  w.nextPlanetExt = n;
  w.graph.build(n, edges);
  w.finalizeIndexes();
  return w;
}

inline void addShips(chronos::World& w, int count, uint64_t seed, double cargo = 1000, double fuelCap = 1e6) {
  using namespace chronos;
  Rng rng(seed);
  for (int i = 0; i < count; ++i) {
    Ship s;
    s.extId = static_cast<int64_t>(w.ships.size());
    s.location = rng.uniformInt(w.numPlanets());
    s.cargoCap = cargo;
    s.fuelCap = fuelCap;
    s.fuel = fuelCap;
    s.speed = 1.0;
    s.reliability = 0.99;
    s.opCost = 1.0;
    s.caps = CAP_CARGO | CAP_MEDICAL;
    w.shipIdx[s.extId] = static_cast<int>(w.ships.size());
    w.ships.push_back(s);
  }
}

inline void addMissions(chronos::World& w, int count, uint64_t seed, int maxPriority = 3) {
  using namespace chronos;
  Rng rng(seed);
  for (int i = 0; i < count; ++i) {
    Mission m;
    m.origin = rng.uniformInt(w.numPlanets());
    m.dest = rng.uniformInt(w.numPlanets());
    if (m.dest == m.origin) m.dest = (m.origin + 1) % w.numPlanets();
    m.resource = rng.uniformInt(4);  // Food..Fuel (no medical capability needed)
    m.amount = std::round(rng.uniform(20, 200));
    m.release = 0;
    m.deadline = rng.uniform(100, 600);
    m.priority = 1 + rng.uniformInt(maxPriority);
    m.importance = rng.uniform(0.2, 1.0);
    w.addMission(m);
  }
}

}  // namespace testing
