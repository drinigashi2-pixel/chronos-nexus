// predict.cpp - lazy min-heap of (planet, resource) shortage times.
// rebuild O(PR log PR); update(planet) O(R log n); upcoming O(k log n).
#include "chronos/predict.hpp"

namespace chronos {

void Predictor::computeKey(int key, double now) {
  int p = key / NUM_RESOURCES, r = key % NUM_RESOURCES;
  const Planet& P = w_.planets[p];
  double sAt = INF, oAt = INF;
  if (P.alive) {
    double net = P.netRate(r);
    double reserve = P.consumption[r] * reserveDays_;
    double stock = P.stock[r];
    if (net < -1e-9) sAt = now + std::max(0.0, (stock - reserve) / (-net)) * HOURS_PER_DAY;
    if (net > 1e-9) oAt = now + std::max(0.0, (P.capacity[r] - stock) / net) * HOURS_PER_DAY;
  }
  shortAt_[key] = sAt;
  overAt_[key] = oAt;
  ++ver_[key];
  if (std::isfinite(sAt)) heap_.push({sAt, key, ver_[key]});
}

void Predictor::rebuild(double now) {
  size_t n = w_.planets.size() * NUM_RESOURCES;
  shortAt_.assign(n, INF);
  overAt_.assign(n, INF);
  ver_.resize(n, 0);
  lead_.resize(w_.planets.size(), -1.0);
  heap_ = decltype(heap_)();
  for (size_t k = 0; k < n; ++k) computeKey(static_cast<int>(k), now);
  counters().statesExplored += n;
}

void Predictor::update(NodeId planet, double now) {
  size_t n = w_.planets.size() * NUM_RESOURCES;
  if (shortAt_.size() < n) {
    shortAt_.resize(n, INF);
    overAt_.resize(n, INF);
    ver_.resize(n, 0);
    lead_.resize(w_.planets.size(), -1.0);
  }
  if (planet < 0 || planet >= static_cast<NodeId>(w_.planets.size())) return;
  for (int r = 0; r < NUM_RESOURCES; ++r) computeKey(planet * NUM_RESOURCES + r, now);
}

void Predictor::setLeadTime(NodeId planet, double hours) {
  if (planet < 0) return;
  if (static_cast<size_t>(planet) >= lead_.size()) lead_.resize(planet + 1, -1.0);
  lead_[planet] = hours;
}

double Predictor::leadTime(NodeId planet) const {
  if (planet >= 0 && static_cast<size_t>(planet) < lead_.size() && lead_[planet] >= 0) return lead_[planet];
  return defaultLead_;
}

double Predictor::timeToShortageH(NodeId p, int r) const {
  size_t k = static_cast<size_t>(p) * NUM_RESOURCES + r;
  return k < shortAt_.size() ? shortAt_[k] : INF;
}

double Predictor::timeToOverflowH(NodeId p, int r) const {
  size_t k = static_cast<size_t>(p) * NUM_RESOURCES + r;
  return k < overAt_.size() ? overAt_[k] : INF;
}

std::vector<ShortagePrediction> Predictor::upcoming(double now, double horizonH) {
  std::vector<ShortagePrediction> out;
  std::vector<Entry> keep;
  while (!heap_.empty() && heap_.top().t <= now + horizonH) {
    Entry e = heap_.top();
    heap_.pop();
    if (e.ver != ver_[e.key] || e.t != shortAt_[e.key]) continue;  // stale (lazy deletion)
    keep.push_back(e);
    int p = e.key / NUM_RESOURCES, r = e.key % NUM_RESOURCES;
    const Planet& P = w_.planets[p];
    ShortagePrediction s;
    s.planet = p;
    s.resource = r;
    s.stock = P.stock[r];
    s.netRatePerDay = P.netRate(r);
    s.timeToShortageH = std::max(0.0, e.t - now);
    s.timeToOverflowH = std::isfinite(overAt_[e.key]) ? overAt_[e.key] - now : INF;
    s.leadTimeH = leadTime(p);
    s.deliveryDeadlineH = e.t - s.leadTimeH;
    s.need = std::max(0.0, -s.netRatePerDay) * coverDays_;
    out.push_back(s);
  }
  for (const auto& e : keep) heap_.push(e);
  return out;
}

}  // namespace chronos
