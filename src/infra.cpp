// infra.cpp - multiple-choice knapsack for construction decisions.
// prune O(n log n); DP O(n*B) time, O(G*B) bytes; greedy O(n log n) + LS O(passes*k*c);
// B&B exponential worst case with fractional bound, used for small instances / certification.
#include "chronos/infra.hpp"

#include <functional>
#include <map>
#include <queue>

namespace chronos {

const char* infraMethodName(InfraMethod m) {
  switch (m) {
    case InfraMethod::DP: return "dp";
    case InfraMethod::Greedy: return "greedy_ls";
    case InfraMethod::BranchAndBound: return "bnb";
    default: return "auto";
  }
}

double InfraPlanner::optionValue(const FacilityOption& f, double horizonDays) const {
  if (f.planet < 0 || f.planet >= w_.numPlanets()) return 0;
  const Planet& P = w_.planets[f.planet];
  if (!P.alive || horizonDays <= 0) return 0;
  double Heff = std::max(0.0, horizonDays - f.buildDays);
  if (Heff <= 0) return 0;
  double v = 0;
  for (int r = 0; r < NUM_RESOURCES; ++r) {
    double imp = f.resourceImpact[r];
    if (imp == 0) continue;
    double net = P.netRate(r);
    if (imp > 0) {
      double covered = std::min(imp, std::max(0.0, -net));  // closes an existing deficit: full value
      double surplus = imp - covered;                         // extra production: export value only
      v += RESOURCE_IMPORTANCE[r] * (covered + 0.25 * surplus) * Heff;
    } else {
      v += RESOURCE_IMPORTANCE[r] * imp * Heff;
    }
  }
  const int E = static_cast<int>(Resource::Energy);
  double eWeight = P.netRate(E) < 0 ? 1.0 : 0.25;
  v += RESOURCE_IMPORTANCE[E] * f.energyImpact * Heff * eWeight;
  v += 5.0 * std::log1p(std::max(0.0, f.populationImpact)) * Heff / horizonDays;
  v += 10.0 * f.strategicValue * Heff / horizonDays;
  v *= P.strategic;
  if (valuePerPoint_ > 0) {
    // Direct score terms (score.cpp): both are measured on end-of-run rates, so an option counts
    // in full once it completes before the end (Heff > 0), independent of when.
    //   EG = clamp(dProd / (0.25 prodStart)) * 12 points, prod = sum importance * production rate
    //   SP = clamp(dTech / (0.10 techStart)) * 10 points, tech = sum tech * strategic
    double dProd = RESOURCE_IMPORTANCE[E] * std::max(0.0, f.energyImpact);
    for (int r = 0; r < NUM_RESOURCES; ++r) dProd += RESOURCE_IMPORTANCE[r] * std::max(0.0, f.resourceImpact[r]);
    double dStrat = 0.01 * f.strategicValue;  // completion raises the planet's strategic weight
    double dTech = P.tech * dStrat;
    if (f.type == static_cast<int>(FacilityType::ResearchStation)) dTech += 0.5 * f.level * (P.strategic + dStrat);
    double pts = 0;
    if (prodStart_ > 0) pts += 12.0 * egHeadroom_ * dProd / (0.25 * prodStart_);
    if (techStart_ > 0) pts += 10.0 * spHeadroom_ * dTech / (0.10 * techStart_);
    v += pts * valuePerPoint_;
  }
  return std::max(0.0, v);
}

void InfraPlanner::refreshScoreState() {
  valuePerPoint_ = 0;
  if (days_ <= 0 || (prodStart_ <= 0 && techStart_ <= 0)) return;
  // exchange rate: resource security is worth 20 points over sum importance * demand unit-days,
  // and the deficit terms above are importance-weighted unit-days, so one point is
  // (weighted demand per day * days) / 20 value units
  double demandPerDay = 0, prodNow = 0, techNow = 0;
  const int E = static_cast<int>(Resource::Energy);
  for (const auto& P : w_.planets) {
    if (!P.alive) continue;
    for (int r = 0; r < NUM_RESOURCES; ++r) {
      demandPerDay += RESOURCE_IMPORTANCE[r] * P.consumption[r];
      prodNow += RESOURCE_IMPORTANCE[r] * P.production[r];
    }
    techNow += P.tech * P.strategic;
  }
  for (const auto& f : w_.facilities) {  // committed but not yet completed
    if (!f.started || f.built || f.planet < 0 || f.planet >= w_.numPlanets()) continue;
    const Planet& P = w_.planets[f.planet];
    prodNow += RESOURCE_IMPORTANCE[E] * std::max(0.0, f.energyImpact);
    for (int r = 0; r < NUM_RESOURCES; ++r) prodNow += RESOURCE_IMPORTANCE[r] * std::max(0.0, f.resourceImpact[r]);
    techNow += P.tech * 0.01 * f.strategicValue;
    if (f.type == static_cast<int>(FacilityType::ResearchStation)) techNow += 0.5 * f.level * P.strategic;
  }
  egHeadroom_ = prodStart_ > 0 ? clamp01(1.0 - (prodNow - prodStart_) / (0.25 * prodStart_)) : 0;
  spHeadroom_ = techStart_ > 0 ? clamp01(1.0 - (techNow - techStart_) / (0.10 * techStart_)) : 0;
  valuePerPoint_ = demandPerDay * days_ / 20.0;
}

std::vector<InfraItem> InfraPlanner::prune(const std::vector<InfraItem>& items, double budget) {
  std::map<int, std::vector<InfraItem>> groups;
  for (const auto& it : items)
    if (it.value > 0 && it.cost <= budget + 1e-9 && it.cost >= 0) groups[it.group].push_back(it);
  std::vector<InfraItem> out;
  for (auto& [g, v] : groups) {
    std::sort(v.begin(), v.end(), [](const InfraItem& a, const InfraItem& b) {
      return a.cost < b.cost || (a.cost == b.cost && a.value > b.value);
    });
    double bestV = -1;
    for (const auto& it : v) {
      if (it.value <= bestV) continue;  // dominated: costs at least as much, worth no more
      bestV = it.value;
      out.push_back(it);
    }
  }
  return out;
}

InfraResult InfraPlanner::solveDP(const std::vector<InfraItem>& items, double budget, int maxBuckets) {
  Timer t;
  InfraResult res;
  res.method = InfraMethod::DP;
  res.candidates = static_cast<int>(items.size());
  std::map<int, std::vector<int>> groups;
  for (size_t i = 0; i < items.size(); ++i)
    if (items[i].cost <= budget + 1e-9) groups[items[i].group].push_back(static_cast<int>(i));
  size_t G = groups.size();
  if (G == 0 || budget <= 0) {
    res.exact = true;
    res.ms = t.ms();
    return res;
  }
  int B = maxBuckets;
  bool integral = budget == std::floor(budget) && budget <= maxBuckets;
  if (integral) B = static_cast<int>(budget);
  while (static_cast<double>(G) * (B + 1) > 4e7 && B > 100) B /= 2;
  double unit = budget / B;
  bool exact = true;
  std::vector<int> cb(items.size(), 0);
  for (size_t i = 0; i < items.size(); ++i) {
    double c = items[i].cost / unit;
    cb[i] = static_cast<int>(std::ceil(c - 1e-9));
    if (std::fabs(c - std::round(c)) > 1e-9) exact = false;
  }
  std::vector<double> dp(B + 1, 0.0), nd(B + 1);
  std::vector<uint16_t> choice(G * static_cast<size_t>(B + 1), 0);
  std::vector<const std::vector<int>*> glist;
  size_t gi = 0;
  for (auto& [g, v] : groups) {
    glist.push_back(&v);
    nd = dp;
    for (size_t k = 0; k < v.size() && k < 65535; ++k) {
      int i = v[k];
      int c = cb[i];
      double val = items[i].value;
      for (int b = B; b >= c; --b) {
        double cand = dp[b - c] + val;
        if (cand > nd[b] + 1e-12) {
          nd[b] = cand;
          choice[gi * (B + 1) + b] = static_cast<uint16_t>(k + 1);
        }
      }
      counters().candidatesEvaluated++;
      counters().statesExplored += static_cast<uint64_t>(B + 1 - c);
    }
    dp.swap(nd);
    ++gi;
  }
  int b = B;
  for (size_t g = G; g-- > 0;) {
    int k = choice[g * (B + 1) + b];
    if (k > 0) {
      int i = (*glist[g])[k - 1];
      res.chosen.push_back(i);
      res.value += items[i].value;
      res.cost += items[i].cost;
      b -= cb[i];
    }
  }
  res.exact = exact;
  res.afterPruning = static_cast<int>(items.size());
  res.ms = t.ms();
  return res;
}

InfraResult InfraPlanner::solveGreedy(const std::vector<InfraItem>& items, double budget, bool localSearch) {
  Timer t;
  InfraResult res;
  res.method = InfraMethod::Greedy;
  res.candidates = static_cast<int>(items.size());
  // upper convex hull per group (LP-dominance), starting from the empty choice (0,0)
  std::map<int, std::vector<int>> groups;
  for (size_t i = 0; i < items.size(); ++i)
    if (items[i].cost <= budget + 1e-9 && items[i].value > 0) groups[items[i].group].push_back(static_cast<int>(i));
  struct Step {
    int group;
    size_t idx;
  };
  std::vector<std::vector<int>> hulls;
  for (auto& [g, v] : groups) {
    std::sort(v.begin(), v.end(), [&](int a, int b) {
      return items[a].cost < items[b].cost || (items[a].cost == items[b].cost && items[a].value > items[b].value);
    });
    std::vector<int> h;
    double lc = 0, lv = 0;
    for (int i : v) {
      if (items[i].value <= lv) continue;
      while (!h.empty()) {
        // remove last hull point if it lies below the segment (prev -> i)
        double pc = h.size() >= 2 ? items[h[h.size() - 2]].cost : 0.0;
        double pv = h.size() >= 2 ? items[h[h.size() - 2]].value : 0.0;
        double mc = items[h.back()].cost, mv = items[h.back()].value;
        double s1 = (mc - pc) > 1e-12 ? (mv - pv) / (mc - pc) : INF;
        double s2 = (items[i].cost - mc) > 1e-12 ? (items[i].value - mv) / (items[i].cost - mc) : INF;
        if (s2 >= s1) h.pop_back();
        else break;
      }
      h.push_back(i);
      lc = items[i].cost;
      lv = items[i].value;
    }
    (void)lc;
    hulls.push_back(h);
  }
  std::vector<int> level(hulls.size(), 0);  // number of hull steps taken per group
  using HItem = std::pair<double, int>;    // incremental efficiency, group
  std::priority_queue<HItem> heap;
  auto incr = [&](int g, double& dc, double& dv) {
    int k = level[g];
    double pc = k ? items[hulls[g][k - 1]].cost : 0.0, pv = k ? items[hulls[g][k - 1]].value : 0.0;
    dc = items[hulls[g][k]].cost - pc;
    dv = items[hulls[g][k]].value - pv;
  };
  for (size_t g = 0; g < hulls.size(); ++g) {
    if (hulls[g].empty()) continue;
    double dc, dv;
    incr(static_cast<int>(g), dc, dv);
    heap.push({dc > 1e-12 ? dv / dc : INF, static_cast<int>(g)});
  }
  double spent = 0, value = 0;
  while (!heap.empty()) {
    int g = heap.top().second;
    heap.pop();
    double dc, dv;
    incr(g, dc, dv);
    counters().candidatesEvaluated++;
    if (spent + dc > budget + 1e-9) continue;  // this upgrade does not fit: group frozen
    spent += dc;
    value += dv;
    level[g]++;
    if (level[g] < static_cast<int>(hulls[g].size())) {
      incr(g, dc, dv);
      heap.push({dc > 1e-12 ? dv / dc : INF, g});
    }
  }
  std::vector<int> chosenOf(hulls.size(), -1);  // item index chosen in each group
  for (size_t g = 0; g < hulls.size(); ++g)
    if (level[g] > 0) chosenOf[g] = hulls[g][level[g] - 1];
  // best single item fix (guarantees >= OPT/2 together with the hull greedy)
  int bestSingle = -1;
  for (size_t i = 0; i < items.size(); ++i)
    if (items[i].cost <= budget + 1e-9 && (bestSingle < 0 || items[i].value > items[bestSingle].value))
      bestSingle = static_cast<int>(i);
  std::map<int, int> gmap;
  {
    int k = 0;
    for (auto& [g, v] : groups) gmap[g] = k++;
  }
  if (bestSingle >= 0 && items[bestSingle].value > value) {
    std::fill(chosenOf.begin(), chosenOf.end(), -1);
    chosenOf[gmap[items[bestSingle].group]] = bestSingle;
    spent = items[bestSingle].cost;
    value = items[bestSingle].value;
  }
  // swap / fill local search over the most efficient unchosen options
  if (localSearch) {
    std::vector<int> pool;
    for (size_t i = 0; i < items.size(); ++i)
      if (items[i].cost <= budget + 1e-9 && items[i].value > 0) pool.push_back(static_cast<int>(i));
    std::sort(pool.begin(), pool.end(), [&](int a, int b) {
      return items[a].value * std::max(1e-9, items[b].cost) > items[b].value * std::max(1e-9, items[a].cost);
    });
    if (pool.size() > 300) pool.resize(300);
    for (int pass = 0; pass < 3; ++pass) {
      bool imp = false;
      for (int j : pool) {
        int gj = gmap[items[j].group];
        int cur = chosenOf[gj];
        if (cur == j) continue;
        double curC = cur >= 0 ? items[cur].cost : 0, curV = cur >= 0 ? items[cur].value : 0;
        // (a) replace the option in j's own group (or fill an empty group)
        if (spent - curC + items[j].cost <= budget + 1e-9 && items[j].value > curV + 1e-12) {
          spent += items[j].cost - curC;
          value += items[j].value - curV;
          chosenOf[gj] = j;
          imp = true;
          continue;
        }
        if (cur >= 0) continue;
        // (b) swap out one chosen option of another group
        for (size_t g = 0; g < chosenOf.size(); ++g) {
          int o = chosenOf[g];
          if (o < 0) continue;
          counters().candidatesEvaluated++;
          if (spent - items[o].cost + items[j].cost <= budget + 1e-9 && items[j].value > items[o].value + 1e-12) {
            spent += items[j].cost - items[o].cost;
            value += items[j].value - items[o].value;
            chosenOf[g] = -1;
            chosenOf[gj] = j;
            imp = true;
            break;
          }
        }
      }
      if (!imp) break;
    }
  }
  for (int c : chosenOf)
    if (c >= 0) {
      res.chosen.push_back(c);
      res.cost += items[c].cost;
      res.value += items[c].value;
    }
  res.afterPruning = static_cast<int>(items.size());
  res.ms = t.ms();
  return res;
}

InfraResult InfraPlanner::solveBnB(const std::vector<InfraItem>& items, double budget, uint64_t nodeLimit) {
  Timer t;
  InfraResult res;
  res.method = InfraMethod::BranchAndBound;
  res.candidates = static_cast<int>(items.size());
  std::vector<int> ord;
  for (size_t i = 0; i < items.size(); ++i)
    if (items[i].cost <= budget + 1e-9 && items[i].value > 0) ord.push_back(static_cast<int>(i));
  std::sort(ord.begin(), ord.end(), [&](int a, int b) {
    return items[a].value * std::max(1e-12, items[b].cost) > items[b].value * std::max(1e-12, items[a].cost);
  });
  std::map<int, int> gmap;
  for (int i : ord) gmap.emplace(items[i].group, static_cast<int>(gmap.size()));
  std::vector<char> used(gmap.size(), 0);
  std::vector<int> cur, best;
  double bestV = 0;
  uint64_t nodes = 0;
  bool aborted = false;
  // fractional knapsack bound over the remaining items (group constraints relaxed)
  auto bound = [&](size_t k, double cap, double val) {
    for (size_t i = k; i < ord.size(); ++i) {
      const auto& it = items[ord[i]];
      if (used[gmap[it.group]]) continue;
      if (it.cost <= cap) {
        cap -= it.cost;
        val += it.value;
      } else {
        val += it.value * (cap / std::max(1e-12, it.cost));
        break;
      }
    }
    return val;
  };
  std::function<void(size_t, double, double)> dfs = [&](size_t k, double cap, double val) {
    if (++nodes > nodeLimit) { aborted = true; return; }
    if (val > bestV + 1e-12) { bestV = val; best = cur; }
    if (k >= ord.size() || aborted) return;
    if (bound(k, cap, val) <= bestV + 1e-12) return;
    const auto& it = items[ord[k]];
    int g = gmap[it.group];
    if (!used[g] && it.cost <= cap + 1e-9) {
      used[g] = 1;
      cur.push_back(ord[k]);
      dfs(k + 1, cap - it.cost, val + it.value);
      cur.pop_back();
      used[g] = 0;
    }
    dfs(k + 1, cap, val);
  };
  dfs(0, budget, 0);
  res.chosen = best;
  for (int i : best) {
    res.value += items[i].value;
    res.cost += items[i].cost;
  }
  res.nodes = nodes;
  res.exact = !aborted;
  counters().bnbNodes += nodes;
  counters().statesExplored += nodes;
  res.ms = t.ms();
  return res;
}

InfraResult InfraPlanner::plan(const std::vector<int>& optionIds, double budget, double horizonDays, InfraMethod m,
                               const Deadline& dl) {
  (void)dl;
  Timer t;
  refreshScoreState();
  std::vector<InfraItem> items;
  std::map<std::pair<int, int>, int> gid;
  for (int o : optionIds) {
    if (o < 0 || o >= static_cast<int>(w_.facilities.size())) continue;
    const FacilityOption& f = w_.facilities[o];
    if (f.built || f.started) continue;
    auto key = std::make_pair(static_cast<int>(f.planet), f.type);
    auto it = gid.find(key);
    if (it == gid.end()) it = gid.emplace(key, static_cast<int>(gid.size())).first;
    InfraItem item;
    item.id = o;
    item.group = it->second;
    item.cost = f.cost;
    item.value = optionValue(f, horizonDays);
    items.push_back(item);
  }
  counters().candidatesEvaluated += items.size();
  std::vector<InfraItem> pruned = prune(items, budget);
  InfraMethod use = m;
  if (use == InfraMethod::Auto) use = pruned.size() <= 2000 ? InfraMethod::DP : InfraMethod::Greedy;
  if (use == InfraMethod::BranchAndBound && pruned.size() > 60) use = InfraMethod::DP;
  InfraResult r;
  if (use == InfraMethod::DP) r = solveDP(pruned, budget);
  else if (use == InfraMethod::BranchAndBound) r = solveBnB(pruned, budget);
  else r = solveGreedy(pruned, budget, true);
  for (auto& c : r.chosen) c = pruned[c].id;  // map to option indices
  r.candidates = static_cast<int>(items.size());
  r.afterPruning = static_cast<int>(pruned.size());
  r.method = use;
  r.ms = t.ms();
  return r;
}

}  // namespace chronos
