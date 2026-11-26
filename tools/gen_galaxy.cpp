// gen_galaxy.cpp - deterministic dataset generator.
//   gen_galaxy <small|medium|large|extreme> <out_dir> [seed]
// Connectivity is guaranteed by a geometric spanning tree (each node linked both ways to the
// nearest previously placed node), then random extra edges to k-nearest neighbours (+2% wormholes).
// Events are sorted by time and contain all 12 event types. O(P log P + R) time.
#include <cstdio>
#include <string>
#include <unordered_set>
#include <vector>

#include "chronos/common.hpp"
#include "chronos/io.hpp"

using namespace chronos;

namespace {

struct Size {
  int planets, routes, missions, events;
};

Size sizeOf(const std::string& s) {
  if (s == "small") return {100, 500, 50, 24};
  if (s == "medium") return {1000, 10000, 500, 60};
  if (s == "large") return {10000, 100000, 5000, 156};
  if (s == "extreme") return {100000, 1000000, 50000, 408};
  return {0, 0, 0, 0};
}

const char* RES[NUM_RESOURCES] = {"food", "water", "energy", "fuel", "medicine", "minerals", "construction"};
const double BASE_CONS[NUM_RESOURCES] = {10, 12, 8, 4, 1.5, 0.5, 3};

struct GP {
  double x, y, z, pop, tech, def, str;
  int type;
  double prod[NUM_RESOURCES], cons[NUM_RESOURCES], cap[NUM_RESOURCES], stock[NUM_RESOURCES];
};

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: gen_galaxy <small|medium|large|extreme> <out_dir> [seed]\n");
    return 2;
  }
  std::string name = argv[1], out = argv[2];
  uint64_t seed = argc > 3 ? std::strtoull(argv[3], nullptr, 10) : 42;
  Size sz = sizeOf(name);
  if (sz.planets == 0) {
    std::fprintf(stderr, "unknown size %s\n", name.c_str());
    return 2;
  }
  ensureDirectory(out);
  Rng rng(seed * 1000003ULL + static_cast<uint64_t>(sz.planets));
  const int P = sz.planets, DAYS = 14;
  const int S = std::max(20, sz.missions);
  const double side = std::sqrt(static_cast<double>(P)) * 20.0;

  // ------------------------------------------------------------ planets
  std::vector<GP> pl(P);
  for (int i = 0; i < P; ++i) {
    GP& p = pl[i];
    p.x = rng.uniform(0, side);
    p.y = rng.uniform(0, side);
    p.z = rng.uniform(-5, 5);
    double u = rng.uniform();
    p.type = u < 0.70 ? 0 : (u < 0.92 ? 1 : 2);
    if (p.type == 0) p.pop = std::exp(rng.uniform(std::log(1e5), std::log(1e8)));
    else if (p.type == 1) p.pop = std::exp(rng.uniform(std::log(1e3), std::log(1e5)));
    else p.pop = rng.uniform(100, 2000);
    p.tech = rng.uniform(1, 10);
    p.def = rng.uniform(1, 10);
    p.str = p.type == 0 ? rng.uniform(1, 3) : (p.type == 1 ? rng.uniform(0.5, 2) : rng.uniform(0.5, 1.5));
    for (int r = 0; r < NUM_RESOURCES; ++r) {
      p.cons[r] = BASE_CONS[r] * std::log10(p.pop + 10) * rng.uniform(0.6, 1.4);
      bool producer = rng.uniform() < 0.42;
      if (p.type == 2 && r == static_cast<int>(Resource::Fuel)) producer = true;
      p.prod[r] = p.cons[r] * (producer ? rng.uniform(1.5, 3.0) : rng.uniform(0.2, 0.95));
      p.cap[r] = std::max(p.cons[r], p.prod[r]) * rng.uniform(30, 60);
      p.stock[r] = std::min(p.cap[r], p.cons[r] * rng.uniform(4, 30));
    }
  }
  {
    FILE* f = std::fopen((out + "/planets.csv").c_str(), "w");
    if (!f) { std::fprintf(stderr, "cannot write to %s\n", out.c_str()); return 1; }
    std::fprintf(f, "id,type,x,y,z,population,tech,defense,strategic");
    for (int r = 0; r < NUM_RESOURCES; ++r)
      std::fprintf(f, ",prod_%s,cons_%s,cap_%s,stock_%s", RES[r], RES[r], RES[r], RES[r]);
    std::fprintf(f, "\n");
    const char* types[3] = {"planet", "colony", "station"};
    for (int i = 0; i < P; ++i) {
      const GP& p = pl[i];
      std::fprintf(f, "%d,%s,%.3f,%.3f,%.3f,%.1f,%.3f,%.3f,%.3f", i, types[p.type], p.x, p.y, p.z, p.pop, p.tech, p.def,
                   p.str);
      for (int r = 0; r < NUM_RESOURCES; ++r)
        std::fprintf(f, ",%.3f,%.3f,%.3f,%.3f", p.prod[r], p.cons[r], p.cap[r], p.stock[r]);
      std::fprintf(f, "\n");
    }
    std::fclose(f);
  }

  // ------------------------------------------------------------ routes
  std::vector<double> xs(P), ys(P);
  std::vector<int> ids(P);
  for (int i = 0; i < P; ++i) { xs[i] = pl[i].x; ys[i] = pl[i].y; ids[i] = i; }
  SpatialGrid2D grid;
  grid.build(xs, ys, ids, 2.0);
  struct RouteRow {
    int u, v;
    bool bidir, worm;
  };
  std::vector<RouteRow> rows;
  rows.reserve(sz.routes);
  std::unordered_set<uint64_t> used;
  auto key = [](int a, int b) { return (static_cast<uint64_t>(a) << 32) | static_cast<uint32_t>(b); };
  // spanning tree: random insertion order, connect to nearest placed node
  std::vector<int> order(P);
  for (int i = 0; i < P; ++i) order[i] = i;
  for (int i = P - 1; i > 0; --i) std::swap(order[i], order[rng.uniformInt(i + 1)]);
  std::vector<char> placed(P, 0);
  placed[order[0]] = 1;
  std::vector<int> near;
  for (int k = 1; k < P; ++k) {
    int u = order[k];
    grid.kNearest(pl[u].x, pl[u].y, 1, near, [&](int id) { return placed[id] != 0; });
    int v = near.empty() ? order[0] : near[0];
    placed[u] = 1;
    rows.push_back({u, v, true, false});
    used.insert(key(u, v));
    used.insert(key(v, u));
  }
  std::vector<int> wormRows;
  int guard = 0;
  while (static_cast<int>(rows.size()) < sz.routes && guard < sz.routes * 20) {
    ++guard;
    int u = rng.uniformInt(P);
    bool worm = rng.uniform() < 0.02;
    int v;
    if (worm) {
      v = rng.uniformInt(P);
    } else {
      grid.kNearest(pl[u].x, pl[u].y, 16, near, [&](int id) { return id != u; });
      if (near.empty()) continue;
      v = near[rng.uniformInt(static_cast<int>(near.size()))];
    }
    if (u == v || used.count(key(u, v))) continue;
    bool bidir = rng.uniform() < 0.7;
    if (bidir && used.count(key(v, u))) bidir = false;
    used.insert(key(u, v));
    if (bidir) used.insert(key(v, u));
    if (worm) wormRows.push_back(static_cast<int>(rows.size()));
    rows.push_back({u, v, bidir, worm});
  }
  {
    FILE* f = std::fopen((out + "/routes.csv").c_str(), "w");
    std::fprintf(f, "id,from,to,distance,time_mean,time_min,time_max,fuel,risk,capacity,reliability,available,wormhole,"
                    "bidirectional\n");
    for (size_t i = 0; i < rows.size(); ++i) {
      const RouteRow& r = rows[i];
      double dist, tm, fuel, risk, rel;
      if (r.worm) {
        dist = rng.uniform(1, 5);
        tm = rng.uniform(1, 3);
        fuel = rng.uniform(2, 8);
        risk = rng.uniform(0.05, 0.2);
        rel = rng.uniform(0.93, 0.99);
      } else {
        dist = std::max(1.0, std::hypot(pl[r.u].x - pl[r.v].x, pl[r.u].y - pl[r.v].y, pl[r.u].z - pl[r.v].z));
        tm = dist / rng.uniform(1.5, 3.0);
        fuel = dist * rng.uniform(0.6, 1.0);
        risk = std::pow(rng.uniform(), 3.0) * 0.3;
        rel = 1.0 - std::pow(rng.uniform(), 4.0) * 0.02;
      }
      double tmin = tm * rng.uniform(0.7, 0.95), tmax = tm * rng.uniform(1.1, 1.8);
      std::fprintf(f, "%zu,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.1f,%.4f,1,%d,%d\n", i, r.u, r.v, dist, tm, tmin, tmax,
                   fuel, risk, rng.uniform(300, 3000), rel, r.worm ? 1 : 0, r.bidir ? 1 : 0);
    }
    std::fclose(f);
  }

  // ------------------------------------------------------------ spacecraft
  {
    FILE* f = std::fopen((out + "/spacecraft.csv").c_str(), "w");
    std::fprintf(f, "id,location,cargo_capacity,fuel_capacity,fuel,speed,available_at,reliability,operating_cost,"
                    "capabilities\n");
    for (int i = 0; i < S; ++i) {
      int loc = rng.uniformInt(P);
      double cap = std::round(rng.uniform(200, 1200)), fcap = std::round(rng.uniform(150, 600));
      unsigned caps = CAP_CARGO;
      if (rng.uniform() < 0.4) caps |= CAP_MEDICAL;
      if (rng.uniform() < 0.2) caps |= CAP_ARMORED;
      if (rng.uniform() < 0.2) caps |= CAP_LONG_RANGE;
      std::fprintf(f, "%d,%d,%.0f,%.0f,%.0f,%.3f,%.2f,%.4f,%.3f,%u\n", i, loc, cap, fcap, fcap, rng.uniform(0.8, 1.5),
                   rng.uniform() < 0.8 ? 0.0 : rng.uniform(0, 24), rng.uniform(0.98, 0.999), rng.uniform(0.5, 3.0),
                   caps);
    }
    std::fclose(f);
  }

  // ------------------------------------------------------------ missions
  {
    FILE* f = std::fopen((out + "/missions.csv").c_str(), "w");
    std::fprintf(f, "id,origin,destination,resource,amount,release,deadline,priority,importance,required_caps\n");
    const double priW[5] = {0.15, 0.25, 0.30, 0.20, 0.10};
    for (int i = 0; i < sz.missions; ++i) {
      int r = rng.uniformInt(NUM_RESOURCES);
      int o = rng.uniformInt(P);
      for (int t = 0; t < 50; ++t) {
        int c = rng.uniformInt(P);
        if (pl[c].prod[r] > pl[c].cons[r]) { o = c; break; }
      }
      int d;
      if (rng.uniform() < 0.7) {
        grid.kNearest(pl[o].x, pl[o].y, 30, near, [&](int id) { return id != o; });
        d = near.empty() ? (o + 1) % P : near[rng.uniformInt(static_cast<int>(near.size()))];
      } else {
        d = rng.uniformInt(P);
        if (d == o) d = (o + 1) % P;
      }
      double rel = rng.uniform(0, DAYS * 24 * 0.6);
      double u = rng.uniform(), acc = 0;
      int pri = 5;
      for (int k = 0; k < 5; ++k) {
        acc += priW[k];
        if (u <= acc) { pri = k + 1; break; }
      }
      unsigned caps = CAP_CARGO;
      if (r == static_cast<int>(Resource::Medicine)) caps |= CAP_MEDICAL;
      if (rng.uniform() < 0.1) caps |= CAP_ARMORED;
      std::fprintf(f, "%d,%d,%d,%s,%.0f,%.2f,%.2f,%d,%.3f,%u\n", i + 1, o, d, RES[r], std::round(rng.uniform(50, 1500)),
                   rel, rel + rng.uniform(24, 240), pri, rng.uniform(0.1, 1.0), caps);
    }
    std::fclose(f);
  }

  // ------------------------------------------------------------ infrastructure
  double totalCost = 0;
  {
    FILE* f = std::fopen((out + "/infrastructure.csv").c_str(), "w");
    std::fprintf(f, "id,planet,type,level,cost,build_days");
    for (int r = 0; r < NUM_RESOURCES; ++r) std::fprintf(f, ",impact_%s", RES[r]);
    std::fprintf(f, ",population_impact,energy_impact,strategic_value\n");
    const char* tn[8] = {"EnergyPlant", "Farm", "Hospital", "Factory", "DefenseSystem", "ResearchStation", "Spaceport",
                         "MiningFacility"};
    const double baseCost[8] = {120, 80, 150, 140, 200, 180, 220, 130};
    const double baseDays[8] = {4, 3, 5, 5, 6, 6, 7, 4};
    int id = 0;
    for (int p = 0; p < P; ++p) {
      int groups = 1 + rng.uniformInt(2);
      int t0 = rng.uniformInt(8);
      for (int g = 0; g < groups; ++g) {
        int t = (t0 + g * (1 + rng.uniformInt(7))) % 8;
        int levels = 1 + rng.uniformInt(3);
        for (int l = 1; l <= levels; ++l) {
          double imp[NUM_RESOURCES] = {0, 0, 0, 0, 0, 0, 0};
          double popI = 0, energyI = 0, strat = 0, k = rng.uniform(0.7, 1.3) * l;
          switch (t) {
            case 0: imp[2] = 20 * k; break;
            case 1: imp[0] = 15 * k; imp[1] = 5 * k; break;
            case 2: imp[4] = 3 * k; popI = 1000 * k; break;
            case 3: imp[6] = 8 * k; energyI = -5 * k; break;
            case 4: strat = 5 * k; break;
            case 5: strat = 3 * k; break;
            case 6: imp[3] = 10 * k; strat = 2 * k; break;
            default: imp[5] = 10 * k; imp[6] = 3 * k; energyI = -3 * k; break;
          }
          double cost = std::round(baseCost[t] * l * rng.uniform(0.8, 1.2));
          totalCost += cost;
          std::fprintf(f, "%d,%d,%s,%d,%.0f,%.1f", id++, p, tn[t], l, cost, baseDays[t] + l);
          for (int r = 0; r < NUM_RESOURCES; ++r) std::fprintf(f, ",%.3f", imp[r]);
          std::fprintf(f, ",%.1f,%.3f,%.3f\n", popI, energyI, strat);
        }
      }
    }
    std::fclose(f);
  }

  // ------------------------------------------------------------ events (all 12 types, sorted)
  {
    struct Ev {
      double t;
      int type;
      long long target;
      double sev, a, b, c, d;
    };
    std::vector<Ev> ev;
    std::vector<int> types;
    for (int i = 0; i < sz.events; ++i) types.push_back(i % 12);
    for (int i = static_cast<int>(types.size()) - 1; i > 0; --i) std::swap(types[i], types[rng.uniformInt(i + 1)]);
    int R = static_cast<int>(rows.size());
    for (int t : types) {
      Ev e{rng.uniform(0, DAYS * 24.0 * 0.9), t, -1, 0, 0, 0, 0, 0};
      switch (t) {
        case 0:  // wormhole collapse
          e.target = wormRows.empty() ? rng.uniformInt(R) : wormRows[rng.uniformInt(static_cast<int>(wormRows.size()))];
          e.sev = 1.0;
          break;
        case 1:  // route unavailable
          e.target = rng.uniformInt(R);
          e.sev = 0.7;
          e.a = rng.uniform() < 0.3 ? 0.0 : rng.uniform(12, 72);
          break;
        case 2:  // pirate blockade
          e.target = rng.uniformInt(R);
          e.sev = rng.uniform(0.2, 0.5);
          e.a = rng.uniform(12, 48);
          break;
        case 3:  // ship destroyed
          e.target = rng.uniformInt(S);
          e.sev = 1.0;
          break;
        case 4:  // emergency medical
          e.target = rng.uniformInt(P);
          e.sev = 0.9;
          e.a = rng.uniform(50, 300);
          e.b = rng.uniform(24, 72);
          break;
        case 5:  // production drop 60%
          e.target = rng.uniformInt(P);
          e.sev = 0.6;
          e.b = rng.uniform() < 0.3 ? -1 : rng.uniformInt(NUM_RESOURCES);
          break;
        case 6: {  // new colony near an existing planet
          int base = rng.uniformInt(P);
          e.sev = 0.5;
          e.a = pl[base].x + rng.uniform(-15, 15);
          e.b = pl[base].y + rng.uniform(-15, 15);
          e.c = rng.uniform(-5, 5);
          e.d = rng.uniform(2000, 20000);
          break;
        }
        case 7:  // population surge
          e.target = rng.uniformInt(P);
          e.sev = rng.uniform(0.2, 0.8);
          break;
        case 8:  // resource temporarily unavailable
          e.target = rng.uniform() < 0.1 ? -1 : rng.uniformInt(P);
          e.sev = 0.5;
          e.a = rng.uniformInt(NUM_RESOURCES);
          e.b = rng.uniform(24, 96);
          break;
        case 9: {  // new high-priority mission
          int r = rng.uniformInt(NUM_RESOURCES);
          int o = rng.uniformInt(P);
          for (int k = 0; k < 50; ++k) {
            int c = rng.uniformInt(P);
            if (pl[c].prod[r] > pl[c].cons[r]) { o = c; break; }
          }
          int d = rng.uniformInt(P);
          if (d == o) d = (o + 1) % P;
          e.target = o;
          e.sev = rng.uniform(0.6, 1.0);
          e.a = d;
          e.b = r;
          e.c = std::round(rng.uniform(100, 600));
          e.d = rng.uniform(24, 96);
          break;
        }
        case 10:  // route more dangerous
          e.target = rng.uniformInt(R);
          e.sev = rng.uniform(0.2, 0.5);
          break;
        default:  // ship mechanical failure
          e.target = rng.uniformInt(S);
          e.sev = 0.5;
          e.a = rng.uniform(6, 48);
          break;
      }
      ev.push_back(e);
    }
    std::stable_sort(ev.begin(), ev.end(), [](const Ev& a, const Ev& b) { return a.t < b.t; });
    FILE* f = std::fopen((out + "/events.csv").c_str(), "w");
    std::fprintf(f, "time,type,target,severity,a,b,c,d\n");
    for (const auto& e : ev)
      std::fprintf(f, "%.3f,%s,%lld,%.3f,%.3f,%.3f,%.3f,%.3f\n", e.t, eventTypeName(static_cast<EventType>(e.type)),
                   e.target, e.sev, e.a, e.b, e.c, e.d);
    std::fclose(f);
  }
  {
    FILE* f = std::fopen((out + "/config.csv").c_str(), "w");
    std::fprintf(f, "key,value\nbudget,%.0f\ndays,%d\n", std::round(0.15 * totalCost), DAYS);
    std::fclose(f);
  }
  std::printf("generated %s galaxy in %s: %d planets, %zu route rows, %d spacecraft, %d missions, %d events (seed %llu)\n",
              name.c_str(), out.c_str(), P, rows.size(), S, sz.missions, sz.events,
              static_cast<unsigned long long>(seed));
  return 0;
}
