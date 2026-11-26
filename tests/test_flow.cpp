// test_flow.cpp - min-cost flow vs brute force / reference SSP, conservation, warm-start repair,
// allocator sanity.
#include "chronos/flow.hpp"
#include "test_util.hpp"

using namespace chronos;

namespace {
struct Transport {
  int S, D;
  std::vector<int> supply, demand, penalty;
  std::vector<std::vector<int>> cap, cost;
};

Transport randomTransport(Rng& rng) {
  Transport t;
  t.S = 1 + rng.uniformInt(3);
  t.D = 1 + rng.uniformInt(3);
  for (int i = 0; i < t.S; ++i) t.supply.push_back(rng.uniformInt(4));
  for (int j = 0; j < t.D; ++j) {
    t.demand.push_back(rng.uniformInt(4));
    t.penalty.push_back(5 + rng.uniformInt(15));
  }
  t.cap.assign(t.S, std::vector<int>(t.D));
  t.cost.assign(t.S, std::vector<int>(t.D));
  for (int i = 0; i < t.S; ++i)
    for (int j = 0; j < t.D; ++j) {
      t.cap[i][j] = rng.uniformInt(4);
      t.cost[i][j] = rng.uniformInt(20);
    }
  return t;
}

// exhaustive search over integer transport flows
int64_t bruteForce(const Transport& t) {
  int64_t best = std::numeric_limits<int64_t>::max();
  std::vector<int> x(t.S * t.D, 0);
  std::function<void(int)> rec = [&](int k) {
    if (k == t.S * t.D) {
      int64_t c = 0;
      for (int i = 0; i < t.S; ++i) {
        int out = 0;
        for (int j = 0; j < t.D; ++j) out += x[i * t.D + j];
        if (out > t.supply[i]) return;
      }
      for (int j = 0; j < t.D; ++j) {
        int in = 0;
        for (int i = 0; i < t.S; ++i) in += x[i * t.D + j];
        if (in > t.demand[j]) return;
        c += static_cast<int64_t>(t.demand[j] - in) * t.penalty[j];
      }
      for (int i = 0; i < t.S; ++i)
        for (int j = 0; j < t.D; ++j) c += static_cast<int64_t>(x[i * t.D + j]) * t.cost[i][j];
      best = std::min(best, c);
      return;
    }
    int i = k / t.D, j = k % t.D;
    for (int v = 0; v <= t.cap[i][j]; ++v) {
      x[k] = v;
      rec(k + 1);
    }
    x[k] = 0;
  };
  rec(0);
  return best;
}

struct Built {
  MinCostFlow mcf;
  int s, t;
  std::vector<std::vector<int>> arc;
};

void build(const Transport& tr, Built& b) {
  b.mcf.reset(0);
  b.s = b.mcf.addNode();
  b.t = b.mcf.addNode();
  std::vector<int> sn, dn;
  for (int i = 0; i < tr.S; ++i) sn.push_back(b.mcf.addNode());
  for (int j = 0; j < tr.D; ++j) dn.push_back(b.mcf.addNode());
  for (int i = 0; i < tr.S; ++i) b.mcf.addArc(b.s, sn[i], tr.supply[i], 0);
  b.arc.assign(tr.S, std::vector<int>(tr.D));
  for (int i = 0; i < tr.S; ++i)
    for (int j = 0; j < tr.D; ++j) b.arc[i][j] = b.mcf.addArc(sn[i], dn[j], tr.cap[i][j], tr.cost[i][j]);
  for (int j = 0; j < tr.D; ++j) {
    b.mcf.addArc(b.s, dn[j], tr.demand[j], tr.penalty[j]);  // shortage arc
    b.mcf.addArc(dn[j], b.t, tr.demand[j], 0);
  }
}

// reference: unit-augmenting successive shortest paths with Bellman-Ford (no potentials)
int64_t referenceMinCost(int n, const std::vector<std::array<int64_t, 4>>& arcs, int s, int t, int64_t& flowOut) {
  struct A {
    int to;
    int64_t cap, cost;
  };
  std::vector<A> e;
  std::vector<std::vector<int>> adj(n);
  for (const auto& a : arcs) {
    adj[a[0]].push_back(static_cast<int>(e.size()));
    e.push_back({static_cast<int>(a[1]), a[2], a[3]});
    adj[a[1]].push_back(static_cast<int>(e.size()));
    e.push_back({static_cast<int>(a[0]), 0, -a[3]});
  }
  int64_t cost = 0, flow = 0;
  while (true) {
    std::vector<int64_t> d(n, std::numeric_limits<int64_t>::max() / 4);
    std::vector<int> pe(n, -1);
    d[s] = 0;
    for (int it = 0; it < n; ++it)
      for (int u = 0; u < n; ++u)
        if (d[u] < std::numeric_limits<int64_t>::max() / 4)
          for (int id : adj[u])
            if (e[id].cap > 0 && d[u] + e[id].cost < d[e[id].to]) {
              d[e[id].to] = d[u] + e[id].cost;
              pe[e[id].to] = id;
            }
    if (pe[t] < 0) break;
    for (int v = t; v != s; v = e[pe[v] ^ 1].to) {
      e[pe[v]].cap -= 1;
      e[pe[v] ^ 1].cap += 1;
    }
    cost += d[t];
    flow += 1;
  }
  flowOut = flow;
  return cost;
}
}  // namespace

TEST(flow_matches_brute_force_on_tiny_transport) {
  Rng rng(17);
  for (int k = 0; k < 300; ++k) {
    Transport t = randomTransport(rng);
    Built b;
    build(t, b);
    auto res = b.mcf.solve(b.s, b.t, 1000000, Deadline::never());
    int64_t demand = 0;
    for (int d : t.demand) demand += d;
    CHECK(res.flow == demand);  // shortage arcs make every demand satisfiable
    CHECK(res.cost == bruteForce(t));
    CHECK(b.mcf.totalCost() == res.cost);
    CHECK(b.mcf.checkConservation(b.s, b.t));
  }
}

TEST(flow_matches_reference_ssp_on_random_networks) {
  Rng rng(23);
  for (int k = 0; k < 150; ++k) {
    int n = 4 + rng.uniformInt(6);
    std::vector<std::array<int64_t, 4>> arcs;
    MinCostFlow mcf(n);
    int m = n + rng.uniformInt(3 * n);
    for (int i = 0; i < m; ++i) {
      int u = rng.uniformInt(n), v = rng.uniformInt(n);
      if (u == v) continue;
      int64_t cap = rng.uniformInt(5), cost = rng.uniformInt(30);
      arcs.push_back({u, v, cap, cost});
      mcf.addArc(u, v, cap, cost);
    }
    int64_t refFlow = 0;
    int64_t refCost = referenceMinCost(n, arcs, 0, n - 1, refFlow);
    auto res = mcf.solve(0, n - 1, 1000000, Deadline::never());
    CHECK(res.flow == refFlow);
    CHECK(res.cost == refCost);
    CHECK(mcf.checkConservation(0, n - 1));
  }
}

TEST(flow_warm_start_repair_equals_fresh_solve) {
  Rng rng(29);
  for (int k = 0; k < 200; ++k) {
    Transport t = randomTransport(rng);
    Built b;
    build(t, b);
    b.mcf.solve(b.s, b.t, 1000000, Deadline::never());
    // event: one transport arc loses capacity (route blocked / reduced)
    int i = rng.uniformInt(t.S), j = rng.uniformInt(t.D);
    int newCap = rng.uniformInt(t.cap[i][j] + 1);
    b.mcf.setCapacity(b.arc[i][j], newCap, b.s, b.t);
    b.mcf.solve(b.s, b.t, 1000000, Deadline::never());
    Transport t2 = t;
    t2.cap[i][j] = newCap;
    CHECK(b.mcf.totalCost() == bruteForce(t2));
    CHECK(b.mcf.checkConservation(b.s, b.t));
    // and a capacity increase back
    b.mcf.setCapacity(b.arc[i][j], t.cap[i][j] + 1, b.s, b.t);
    b.mcf.solve(b.s, b.t, 1000000, Deadline::never());
    Transport t3 = t;
    t3.cap[i][j] = t.cap[i][j] + 1;
    CHECK(b.mcf.totalCost() == bruteForce(t3));
  }
}

TEST(flow_allocator_respects_supply_and_conservation) {
  World w = testing::makeWorld(80, 300, 61);
  testing::addShips(w, 10, 3);
  Router r(w.graph);
  r.setBoundsEnabled(true);
  ResourceAllocator alloc(w, r);
  std::vector<double> zero(static_cast<size_t>(w.numPlanets()) * NUM_RESOURCES, 0.0);
  for (int method = 0; method < 2; ++method) {
    AllocConfig cfg;
    cfg.method = static_cast<AllocMethod>(method);
    cfg.fleetCapacity = 1e9;
    AllocationStats st = alloc.allocate(cfg, 0.0, zero, zero, Deadline::never());
    CHECK(st.deficits > 0);
    std::vector<double> out(static_cast<size_t>(w.numPlanets()) * NUM_RESOURCES, 0.0);
    for (const auto& sh : alloc.shipments()) {
      if (!sh.active) continue;
      CHECK(sh.amount >= cfg.minShipment - 1e-9);
      CHECK(sh.from != sh.to);
      CHECK(pathUsable(w.graph, sh.path, 0.0));
      CHECK(!sh.path.empty() && w.graph.from(sh.path.front()) == sh.from && w.graph.to(sh.path.back()) == sh.to);
      out[static_cast<size_t>(sh.from) * NUM_RESOURCES + sh.resource] += sh.amount;
    }
    for (int p = 0; p < w.numPlanets(); ++p)
      for (int res = 0; res < NUM_RESOURCES; ++res) {
        const Planet& P = w.planets[p];
        // a supplier never ships more than its stock above the reserve
        double reserve = P.consumption[res] * cfg.reserveDays;
        CHECK(out[static_cast<size_t>(p) * NUM_RESOURCES + res] <= std::max(0.0, P.stock[res] - reserve) + 1.0);
      }
    if (method == 0) CHECK(alloc.totalObjective() >= 0);
  }
}
