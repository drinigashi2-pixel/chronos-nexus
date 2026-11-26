// graph.cpp - CSR dynamic graph and World helpers.
// build/compact: O(V + E); addNode/addEdge/attribute updates: O(1) amortised.
#include "chronos/graph.hpp"

namespace chronos {

void DynamicGraph::build(int numNodes, const std::vector<EdgeAttr>& edges) {
  n_ = std::max(0, numNodes);
  size_t m = edges.size();
  from_.clear(); to_.clear(); dist_.clear(); tmean_.clear(); tmin_.clear(); tmax_.clear();
  fuel_.clear(); risk_.clear(); cap_.clear(); rel_.clear(); avail_.clear(); worm_.clear(); route_.clear();
  from_.reserve(m); to_.reserve(m); dist_.reserve(m); tmean_.reserve(m); tmin_.reserve(m); tmax_.reserve(m);
  fuel_.reserve(m); risk_.reserve(m); cap_.reserve(m); rel_.reserve(m); avail_.reserve(m); worm_.reserve(m);
  route_.reserve(m);
  for (const auto& a : edges) {
    from_.push_back(a.from); to_.push_back(a.to);
    dist_.push_back(a.dist); tmean_.push_back(a.tmean); tmin_.push_back(a.tmin); tmax_.push_back(a.tmax);
    fuel_.push_back(a.fuel); risk_.push_back(a.risk); cap_.push_back(a.cap); rel_.push_back(a.rel);
    avail_.push_back(a.available ? 1 : 0); worm_.push_back(a.wormhole ? 1 : 0); route_.push_back(a.routeId);
  }
  version_ = 0;
  log_.clear();
  rebuildCsr();
}

void DynamicGraph::rebuildCsr() {
  size_t m = from_.size();
  rowOut_.assign(static_cast<size_t>(n_) + 1, 0);
  rowIn_.assign(static_cast<size_t>(n_) + 1, 0);
  for (size_t e = 0; e < m; ++e) {
    rowOut_[from_[e] + 1]++;
    rowIn_[to_[e] + 1]++;
  }
  for (int v = 0; v < n_; ++v) {
    rowOut_[v + 1] += rowOut_[v];
    rowIn_[v + 1] += rowIn_[v];
  }
  adjOut_.assign(m, 0);
  adjIn_.assign(m, 0);
  std::vector<int> po(rowOut_.begin(), rowOut_.end() - 1), pi(rowIn_.begin(), rowIn_.end() - 1);
  for (size_t e = 0; e < m; ++e) {
    adjOut_[po[from_[e]]++] = static_cast<int>(e);
    adjIn_[pi[to_[e]]++] = static_cast<int>(e);
  }
  ovHeadOut_.assign(n_, -1);
  ovHeadIn_.assign(n_, -1);
  ovNextOut_.clear(); ovEdgeOut_.clear(); ovNextIn_.clear(); ovEdgeIn_.clear();
}

NodeId DynamicGraph::addNode() {
  rowOut_.push_back(rowOut_.back());
  rowIn_.push_back(rowIn_.back());
  ovHeadOut_.push_back(-1);
  ovHeadIn_.push_back(-1);
  return n_++;
}

EdgeId DynamicGraph::addEdge(const EdgeAttr& a) {
  EdgeId e = static_cast<EdgeId>(from_.size());
  from_.push_back(a.from); to_.push_back(a.to);
  dist_.push_back(a.dist); tmean_.push_back(a.tmean); tmin_.push_back(a.tmin); tmax_.push_back(a.tmax);
  fuel_.push_back(a.fuel); risk_.push_back(a.risk); cap_.push_back(a.cap); rel_.push_back(a.rel);
  avail_.push_back(a.available ? 1 : 0); worm_.push_back(a.wormhole ? 1 : 0); route_.push_back(a.routeId);
  ovEdgeOut_.push_back(e);
  ovNextOut_.push_back(ovHeadOut_[a.from]);
  ovHeadOut_[a.from] = static_cast<int>(ovEdgeOut_.size()) - 1;
  ovEdgeIn_.push_back(e);
  ovNextIn_.push_back(ovHeadIn_[a.to]);
  ovHeadIn_[a.to] = static_cast<int>(ovEdgeIn_.size()) - 1;
  log(e, EdgeChangeKind::Added);
  // Merge threshold: overflow larger than max(1024, 5% of edges) triggers a CSR rebuild.
  if (ovEdgeOut_.size() > std::max<size_t>(1024, from_.size() / 20)) compact();
  return e;
}

void DynamicGraph::compact() {
  if (ovEdgeOut_.empty()) return;
  rebuildCsr();
}

int DynamicGraph::outDegree(NodeId u) const {
  int d = rowOut_[u + 1] - rowOut_[u];
  for (int i = ovHeadOut_[u]; i >= 0; i = ovNextOut_[i]) ++d;
  return d;
}

EdgeAttr DynamicGraph::attr(EdgeId e) const {
  EdgeAttr a;
  a.from = from_[e]; a.to = to_[e]; a.dist = dist_[e]; a.tmean = tmean_[e]; a.tmin = tmin_[e]; a.tmax = tmax_[e];
  a.fuel = fuel_[e]; a.risk = risk_[e]; a.cap = cap_[e]; a.rel = rel_[e];
  a.available = avail_[e] != 0; a.wormhole = worm_[e] != 0; a.routeId = route_[e];
  return a;
}

void DynamicGraph::setAvailable(EdgeId e, bool on) {
  if ((avail_[e] != 0) == on) return;
  avail_[e] = on ? 1 : 0;
  log(e, on ? EdgeChangeKind::Enabled : EdgeChangeKind::Disabled);
}

void DynamicGraph::setRisk(EdgeId e, float risk) {
  risk = std::min(0.99f, std::max(0.0f, risk));
  EdgeChangeKind k = risk >= risk_[e] ? EdgeChangeKind::Increase : EdgeChangeKind::Decrease;
  risk_[e] = risk;
  log(e, k);
}

void DynamicGraph::setReliability(EdgeId e, float rel) {
  rel = std::min(1.0f, std::max(0.01f, rel));
  EdgeChangeKind k = rel <= rel_[e] ? EdgeChangeKind::Increase : EdgeChangeKind::Decrease;
  rel_[e] = rel;
  log(e, k);
}

void DynamicGraph::setTimes(EdgeId e, float tmin, float tmean, float tmax) {
  tmean = std::max(0.0f, tmean);
  tmin = std::min(std::max(0.0f, tmin), tmean);
  tmax = std::max(tmax, tmean);
  EdgeChangeKind k = (tmean >= tmean_[e] && tmax - tmin >= tmax_[e] - tmin_[e]) ? EdgeChangeKind::Increase
                                                                                  : EdgeChangeKind::Decrease;
  tmin_[e] = tmin; tmean_[e] = tmean; tmax_[e] = tmax;
  log(e, k);
}

size_t DynamicGraph::memoryBytes() const {
  size_t m = from_.size();
  return (rowOut_.size() + rowIn_.size() + adjOut_.size() + adjIn_.size()) * sizeof(int) +
         m * (2 * sizeof(NodeId) + 8 * sizeof(float) + 2 + sizeof(int64_t)) +
         (ovHeadOut_.size() + ovHeadIn_.size() + 2 * (ovEdgeOut_.size() + ovNextOut_.size())) * sizeof(int) +
         log_.size() * sizeof(EdgeChange);
}

// ------------------------------------------------------------------ World
int World::planetOf(int64_t ext) const {
  auto it = planetIdx.find(ext);
  return it == planetIdx.end() ? -1 : it->second;
}

int World::shipOf(int64_t ext) const {
  auto it = shipIdx.find(ext);
  return it == shipIdx.end() ? -1 : it->second;
}

const std::vector<EdgeId>* World::edgesOfRoute(int64_t ext) const {
  auto it = routeEdges.find(ext);
  return it == routeEdges.end() ? nullptr : &it->second;
}

bool World::isRefuelNode(NodeId v) const {
  if (v < 0 || v >= numPlanets()) return false;
  const Planet& p = planets[v];
  return p.alive && (p.type == PlanetType::Station || p.stock[static_cast<int>(Resource::Fuel)] > 50.0);
}

std::vector<uint8_t> World::refuelMask() const {
  std::vector<uint8_t> m(planets.size(), 0);
  for (int v = 0; v < numPlanets(); ++v) m[v] = isRefuelNode(v) ? 1 : 0;
  return m;
}

NodeId World::addPlanet(const Planet& p) {
  NodeId v = graph.addNode();
  planets.push_back(p);
  planetIdx[p.extId] = v;
  nextPlanetExt = std::max(nextPlanetExt, p.extId + 1);
  finalizeIndexes();
  return v;
}

int World::addMission(Mission m) {
  if (m.extId <= 0) m.extId = nextMissionExt;
  nextMissionExt = std::max(nextMissionExt, m.extId + 1);
  missions.push_back(std::move(m));
  return static_cast<int>(missions.size()) - 1;
}

void World::finalizeIndexes() {
  size_t need = planets.size() * NUM_RESOURCES;
  if (frozenUntil.size() < need) frozenUntil.resize(need, -INF);
  if (extraDemand.size() < need) extraDemand.resize(need, 0.0);
}

}  // namespace chronos
