// io.cpp - tolerant CSV loader. O(rows) time; duplicate edge detection uses a hash set
// (O(E) expected). Tolerates missing files, extra/missing columns, duplicate edges, zero or
// negative weights (clamped), unknown references (skipped and counted), empty mission lists.
#include "chronos/io.hpp"

#include <filesystem>
#include <unordered_set>

namespace chronos {

namespace {
const char* RES_KEYS[NUM_RESOURCES] = {"food", "water", "energy", "fuel", "medicine", "minerals", "construction"};

std::string lower(std::string s) {
  for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}
std::string trim(const std::string& s) {
  size_t a = 0, b = s.size();
  while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n')) ++a;
  while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r' || s[b - 1] == '\n')) --b;
  return s.substr(a, b - a);
}
double nonneg(double v) { return std::isfinite(v) && v > 0 ? v : 0.0; }
}  // namespace

std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  bool quoted = false;
  for (size_t i = 0; i < line.size(); ++i) {
    char c = line[i];
    if (quoted) {
      if (c == '"') {
        if (i + 1 < line.size() && line[i + 1] == '"') { cur.push_back('"'); ++i; }
        else quoted = false;
      } else cur.push_back(c);
    } else if (c == '"') quoted = true;
    else if (c == ',') { out.push_back(trim(cur)); cur.clear(); }
    else cur.push_back(c);
  }
  out.push_back(trim(cur));
  return out;
}

bool CsvReader::open(const std::string& path) {
  in_.reset(new std::ifstream(path));
  if (!in_->good()) { in_.reset(); return false; }
  std::string line;
  while (std::getline(*in_, line)) {
    if (trim(line).empty()) continue;
    header_ = splitCsvLine(line);
    for (auto& h : header_) h = lower(h);
    if (!header_.empty() && header_[0].size() >= 3 && static_cast<unsigned char>(header_[0][0]) == 0xEF)
      header_[0] = header_[0].substr(3);  // UTF-8 BOM
    return true;
  }
  in_.reset();
  return false;
}

bool CsvReader::next() {
  if (!in_) return false;
  std::string line;
  while (std::getline(*in_, line)) {
    ++rowNo_;
    if (trim(line).empty() || line[0] == '#') continue;
    row_ = splitCsvLine(line);
    return true;
  }
  return false;
}

int CsvReader::column(const std::string& name) const {
  std::string n = lower(name);
  for (size_t i = 0; i < header_.size(); ++i)
    if (header_[i] == n) return static_cast<int>(i);
  return -1;
}

std::string CsvReader::str(int col, const std::string& def) const {
  if (col < 0 || col >= static_cast<int>(row_.size()) || row_[col].empty()) return def;
  return row_[col];
}

double CsvReader::num(int col, double def) const {
  if (col < 0 || col >= static_cast<int>(row_.size()) || row_[col].empty()) return def;
  char* end = nullptr;
  double v = std::strtod(row_[col].c_str(), &end);
  if (end == row_[col].c_str() || !std::isfinite(v)) return def;
  // Magnitude guard: no physical quantity in a dataset comes near 1e12, while values such as
  // 1e308 overflow to inf in the first sum or product (score NaN, unbounded allocations). Clamp
  // instead of rejecting, so a huge-but-meaningful value keeps its sign and order. Subnormals
  // (|v| < 1e-12) become 0.
  constexpr double kMaxInput = 1e12;
  if (v > kMaxInput) return kMaxInput;
  if (v < -kMaxInput) return -kMaxInput;
  if (std::fabs(v) < 1e-12) return 0.0;
  return v;
}

int64_t CsvReader::integer(int col, int64_t def) const {
  double v = num(col, static_cast<double>(def));
  return static_cast<int64_t>(std::llround(v));
}

std::string joinPath(const std::string& a, const std::string& b) {
  if (a.empty()) return b;
  if (a.back() == '/' || a.back() == '\\') return a + b;
  return a + "/" + b;
}

bool ensureDirectory(const std::string& dir) {
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  return std::filesystem::is_directory(dir, ec);
}

bool writeTextFile(const std::string& path, const std::string& content) {
  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  f << content;
  return static_cast<bool>(f);
}

// ------------------------------------------------------------------ loaders
static void loadConfig(const std::string& dir, World& w, LoadReport& rep) {
  CsvReader r;
  if (!r.open(joinPath(dir, "config.csv"))) return;
  int ck = r.column("key"), cv = r.column("value");
  if (ck < 0) ck = 0;
  if (cv < 0) cv = 1;
  while (r.next()) {
    std::string k = lower(r.str(ck));
    double v = r.num(cv, NAN);
    if (!std::isfinite(v)) { rep.skippedRows++; continue; }
    if (k == "budget") w.budget = std::max(0.0, v);
    else if (k == "days") w.days = std::max(1, static_cast<int>(v));
  }
}

static bool loadPlanets(const std::string& dir, World& w, LoadReport& rep) {
  CsvReader r;
  if (!r.open(joinPath(dir, "planets.csv"))) {
    rep.warnings.push_back("planets.csv missing or empty");
    rep.missingFiles++;
    return false;
  }
  int cid = r.column("id"), ctype = r.column("type"), cx = r.column("x"), cy = r.column("y"), cz = r.column("z"),
      cpop = r.column("population"), ctech = r.column("tech"), cdef = r.column("defense"),
      cstr = r.column("strategic");
  int cp[NUM_RESOURCES], cc[NUM_RESOURCES], ccap[NUM_RESOURCES], cs[NUM_RESOURCES];
  for (int k = 0; k < NUM_RESOURCES; ++k) {
    std::string key = RES_KEYS[k];
    cp[k] = r.column("prod_" + key);
    cc[k] = r.column("cons_" + key);
    ccap[k] = r.column("cap_" + key);
    cs[k] = r.column("stock_" + key);
  }
  int64_t rowIdx = 0;
  while (r.next()) {
    Planet p;
    p.extId = r.integer(cid, rowIdx);
    ++rowIdx;
    if (w.planetIdx.count(p.extId)) { rep.skippedRows++; rep.warnings.push_back("duplicate planet id"); continue; }
    std::string t = lower(r.str(ctype, "planet"));
    p.type = (t == "colony" || t == "1") ? PlanetType::Colony : (t == "station" || t == "2") ? PlanetType::Station
                                                                                             : PlanetType::Planet;
    p.x = r.num(cx, 0); p.y = r.num(cy, 0); p.z = r.num(cz, 0);
    p.population = nonneg(r.num(cpop, 1000));
    p.initialPopulation = p.population;
    p.tech = nonneg(r.num(ctech, 1));
    p.defense = nonneg(r.num(cdef, 1));
    p.strategic = std::max(0.01, nonneg(r.num(cstr, 1)));
    for (int k = 0; k < NUM_RESOURCES; ++k) {
      p.production[k] = nonneg(r.num(cp[k], 0));
      p.consumption[k] = nonneg(r.num(cc[k], 0));
      double defCap = std::max(1000.0, 60.0 * std::max(p.production[k], p.consumption[k]));
      p.capacity[k] = std::max(1.0, nonneg(r.num(ccap[k], defCap)));
      p.stock[k] = std::min(p.capacity[k], nonneg(r.num(cs[k], 10.0 * p.consumption[k])));
    }
    w.planetIdx[p.extId] = static_cast<int>(w.planets.size());
    w.nextPlanetExt = std::max(w.nextPlanetExt, p.extId + 1);
    w.planets.push_back(p);
  }
  rep.planets = static_cast<int>(w.planets.size());
  return !w.planets.empty();
}

static void loadRoutes(const std::string& dir, World& w, LoadReport& rep, std::vector<EdgeAttr>& edges) {
  CsvReader r;
  if (!r.open(joinPath(dir, "routes.csv"))) {
    rep.warnings.push_back("routes.csv missing: graph has no edges");
    rep.missingFiles++;
    return;
  }
  int cid = r.column("id"), cf = r.column("from"), ct = r.column("to"), cd = r.column("distance"),
      ctm = r.column("time_mean"), ctl = r.column("time_min"), cth = r.column("time_max"), cfu = r.column("fuel"),
      crk = r.column("risk"), ccap = r.column("capacity"), crel = r.column("reliability"),
      cav = r.column("available"), cw = r.column("wormhole"), cb = r.column("bidirectional");
  if (ctm < 0) ctm = r.column("time");
  if (crel < 0) crel = r.column("reliability_pct");
  std::unordered_set<uint64_t> seen;
  seen.reserve(1024);
  int64_t rowIdx = 0;
  auto addDirected = [&](const EdgeAttr& a) {
    uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(a.from)) << 32) | static_cast<uint32_t>(a.to);
    if (!seen.insert(key).second) { rep.duplicateEdges++; return; }
    w.routeEdges[a.routeId].push_back(static_cast<EdgeId>(edges.size()));
    edges.push_back(a);
  };
  while (r.next()) {
    int64_t rid = r.integer(cid, rowIdx);
    ++rowIdx;
    rep.routeRows++;
    int u = w.planetOf(r.integer(cf, -1)), v = w.planetOf(r.integer(ct, -1));
    if (u < 0 || v < 0) { rep.unknownRefs++; continue; }
    if (u == v) { rep.skippedRows++; continue; }
    if (w.routeEdges.count(rid)) rid = -(rowIdx + 1000000000LL);  // duplicate route id: synthesize
    EdgeAttr a;
    a.from = u; a.to = v; a.routeId = rid;
    a.dist = static_cast<float>(nonneg(r.num(cd, 1.0)));
    double tm = nonneg(r.num(ctm, a.dist));
    double tl = nonneg(r.num(ctl, 0.8 * tm)), th = nonneg(r.num(cth, 1.3 * tm));
    tl = std::min(tl, tm);
    th = std::max(th, tm);
    a.tmean = static_cast<float>(tm); a.tmin = static_cast<float>(tl); a.tmax = static_cast<float>(th);
    a.fuel = static_cast<float>(nonneg(r.num(cfu, a.dist)));
    a.risk = static_cast<float>(std::min(0.99, nonneg(r.num(crk, 0.0))));
    a.cap = static_cast<float>(nonneg(r.num(ccap, 1e6)));
    double rel = r.num(crel, 1.0);
    if (rel > 1.0 && rel <= 100.0) rel /= 100.0;
    a.rel = static_cast<float>(std::min(1.0, std::max(0.01, rel)));
    a.available = r.num(cav, 1) != 0;
    a.wormhole = r.num(cw, 0) != 0;
    addDirected(a);
    if (r.num(cb, 0) != 0) {
      EdgeAttr b = a;
      std::swap(b.from, b.to);
      addDirected(b);
    }
    w.nextRouteExt = std::max(w.nextRouteExt, rid + 1);
  }
  rep.edges = static_cast<int>(edges.size());
}

static void loadShips(const std::string& dir, World& w, LoadReport& rep) {
  CsvReader r;
  if (!r.open(joinPath(dir, "spacecraft.csv"))) {
    rep.warnings.push_back("spacecraft.csv missing: no fleet");
    rep.missingFiles++;
    return;
  }
  int cid = r.column("id"), cl = r.column("location"), cc = r.column("cargo_capacity"), cfc = r.column("fuel_capacity"),
      cf = r.column("fuel"), csp = r.column("speed"), ca = r.column("available_at"), crel = r.column("reliability"),
      coc = r.column("operating_cost"), ccap = r.column("capabilities");
  int64_t rowIdx = 0;
  while (r.next()) {
    Ship s;
    s.extId = r.integer(cid, rowIdx);
    ++rowIdx;
    int loc = w.planetOf(r.integer(cl, -1));
    if (loc < 0 || w.shipIdx.count(s.extId)) { rep.unknownRefs++; continue; }
    s.location = loc;
    s.cargoCap = nonneg(r.num(cc, 500));
    s.fuelCap = std::max(1.0, nonneg(r.num(cfc, 1000)));
    s.fuel = std::min(s.fuelCap, nonneg(r.num(cf, s.fuelCap)));
    s.speed = std::max(0.05, nonneg(r.num(csp, 1.0)));
    s.availableAt = nonneg(r.num(ca, 0));
    double rel = r.num(crel, 0.99);
    s.reliability = std::min(1.0, std::max(0.01, rel > 1 ? rel / 100.0 : rel));
    s.opCost = nonneg(r.num(coc, 1.0));
    s.caps = static_cast<uint32_t>(std::max<int64_t>(0, r.integer(ccap, CAP_CARGO))) | CAP_CARGO;
    w.shipIdx[s.extId] = static_cast<int>(w.ships.size());
    w.ships.push_back(s);
  }
  rep.ships = static_cast<int>(w.ships.size());
}

static void loadMissions(const std::string& dir, World& w, LoadReport& rep) {
  CsvReader r;
  if (!r.open(joinPath(dir, "missions.csv"))) {
    rep.warnings.push_back("missions.csv missing: only proactive missions will be generated");
    rep.missingFiles++;
    return;
  }
  int cid = r.column("id"), co = r.column("origin"), cd = r.column("destination"), cr = r.column("resource"),
      ca = r.column("amount"), crel = r.column("release"), cdl = r.column("deadline"), cp = r.column("priority"),
      ci = r.column("importance"), ccap = r.column("required_caps");
  if (cd < 0) cd = r.column("dest");
  int64_t rowIdx = 0;
  std::unordered_set<int64_t> ids;
  while (r.next()) {
    Mission m;
    m.extId = r.integer(cid, rowIdx + 1);
    ++rowIdx;
    int o = w.planetOf(r.integer(co, -1)), d = w.planetOf(r.integer(cd, -1));
    int res = resourceFromString(r.str(cr, "0"));
    if (o < 0 || d < 0 || res < 0 || o == d || !ids.insert(m.extId).second) { rep.unknownRefs++; continue; }
    m.origin = o; m.dest = d; m.resource = res;
    m.amount = nonneg(r.num(ca, 100));
    if (m.amount <= 0) { rep.skippedRows++; continue; }
    m.release = nonneg(r.num(crel, 0));
    m.deadline = std::max(m.release, r.num(cdl, m.release + 240));
    m.priority = static_cast<int>(std::min<int64_t>(5, std::max<int64_t>(1, r.integer(cp, 3))));
    double imp = r.num(ci, 0.5);
    if (imp > 1) imp /= 10.0;
    m.importance = std::min(1.0, std::max(0.01, imp));
    m.requiredCaps = static_cast<uint32_t>(std::max<int64_t>(0, r.integer(ccap, CAP_CARGO))) | CAP_CARGO;
    m.source = MissionSource::Input;
    w.addMission(m);
  }
  rep.missions = static_cast<int>(w.missions.size());
}

static void loadFacilities(const std::string& dir, World& w, LoadReport& rep) {
  CsvReader r;
  if (!r.open(joinPath(dir, "infrastructure.csv"))) {
    rep.warnings.push_back("infrastructure.csv missing: no construction options");
    rep.missingFiles++;
    return;
  }
  int cid = r.column("id"), cp = r.column("planet"), ct = r.column("type"), cl = r.column("level"),
      cc = r.column("cost"), cb = r.column("build_days"), cpi = r.column("population_impact"),
      cei = r.column("energy_impact"), csv = r.column("strategic_value");
  int ci[NUM_RESOURCES];
  for (int k = 0; k < NUM_RESOURCES; ++k) ci[k] = r.column(std::string("impact_") + RES_KEYS[k]);
  int64_t rowIdx = 0;
  while (r.next()) {
    FacilityOption f;
    f.extId = r.integer(cid, rowIdx);
    ++rowIdx;
    int p = w.planetOf(r.integer(cp, -1));
    int t = facilityTypeFromString(r.str(ct, "0"));
    if (p < 0 || t < 0) { rep.unknownRefs++; continue; }
    f.planet = p; f.type = t;
    f.level = static_cast<int>(std::max<int64_t>(1, r.integer(cl, 1)));
    f.cost = nonneg(r.num(cc, 100));
    f.buildDays = nonneg(r.num(cb, 5));
    for (int k = 0; k < NUM_RESOURCES; ++k) f.resourceImpact[k] = r.num(ci[k], 0);
    f.populationImpact = r.num(cpi, 0);
    f.energyImpact = r.num(cei, 0);
    f.strategicValue = nonneg(r.num(csv, 0));
    w.facilities.push_back(f);
  }
  rep.facilities = static_cast<int>(w.facilities.size());
}

static void loadEvents(const std::string& dir, World& w, LoadReport& rep) {
  CsvReader r;
  if (!r.open(joinPath(dir, "events.csv"))) {
    rep.warnings.push_back("events.csv missing: static galaxy");
    rep.missingFiles++;
    return;
  }
  int ctime = r.column("time"), ctype = r.column("type"), ctar = r.column("target"), csev = r.column("severity"),
      ca = r.column("a"), cb = r.column("b"), cc = r.column("c"), cd = r.column("d");
  while (r.next()) {
    int t = eventTypeFromString(r.str(ctype, ""));
    double time = r.num(ctime, NAN);
    if (t < 0 || !std::isfinite(time)) { rep.skippedRows++; continue; }
    Event e;
    e.time = std::max(0.0, time);
    e.type = static_cast<EventType>(t);
    e.target = r.integer(ctar, -1);
    e.severity = r.num(csev, 0.5);
    e.a = r.num(ca, 0); e.b = r.num(cb, 0); e.c = r.num(cc, 0); e.d = r.num(cd, 0);
    e.index = static_cast<int>(w.events.size());
    w.events.push_back(e);
  }
  std::stable_sort(w.events.begin(), w.events.end(), [](const Event& a, const Event& b) { return a.time < b.time; });
  for (size_t i = 0; i < w.events.size(); ++i) w.events[i].index = static_cast<int>(i);
  rep.events = static_cast<int>(w.events.size());
}

bool loadDataset(const std::string& dir, World& w, LoadReport& rep) {
  Timer t;
  w = World();
  loadConfig(dir, w, rep);
  if (!loadPlanets(dir, w, rep)) {
    rep.ms = t.ms();
    return false;
  }
  std::vector<EdgeAttr> edges;
  loadRoutes(dir, w, rep, edges);
  w.graph.build(static_cast<int>(w.planets.size()), edges);
  w.finalizeIndexes();
  loadShips(dir, w, rep);
  loadMissions(dir, w, rep);
  loadFacilities(dir, w, rep);
  loadEvents(dir, w, rep);
  if (w.budget <= 0) {
    double total = 0;
    for (const auto& f : w.facilities) total += f.cost;
    w.budget = 0.1 * total;  // documented default when config.csv gives no budget
  }
  rep.ms = t.ms();
  return true;
}

}  // namespace chronos
