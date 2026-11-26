// common.hpp - shared constants, deterministic RNG, timers, memory probes,
// global search counters and a 2-D spatial grid.
// All helpers are O(1) except SpatialGrid2D (build O(n), kNearest ~O(k + cells)).
#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#if defined(__unix__) || defined(__APPLE__)
#include <sys/resource.h>
#include <unistd.h>
#endif

namespace chronos {

using NodeId = int32_t;
using EdgeId = int32_t;

constexpr double INF = std::numeric_limits<double>::infinity();
constexpr double HOURS_PER_DAY = 24.0;
constexpr int NUM_RESOURCES = 7;
// Deterministic work model: search states (settled nodes, labels, evaluations, ...) processed per
// millisecond on a typical core. Optimisation stages are bounded by work = ms * this constant, so
// two runs with the same seed take identical decisions; the wall clock is only a guard.
constexpr double WORK_UNITS_PER_MS = 1500.0;

enum class Resource : uint8_t { Food = 0, Water, Energy, Fuel, Medicine, RareMinerals, Construction };

// Importance weight per resource (Food..Construction). Used in penalties and the score.
constexpr std::array<double, NUM_RESOURCES> RESOURCE_IMPORTANCE = {1.0, 1.0, 0.7, 0.6, 1.2, 0.3, 0.4};
// Criticality order used by the sequential min-cost-flow allocation.
constexpr std::array<int, NUM_RESOURCES> CRITICALITY_ORDER = {4, 1, 0, 2, 3, 6, 5};
// Resources whose shortage kills population (Food, Water, Medicine).
inline bool isCriticalResource(int r) { return r == 0 || r == 1 || r == 4; }
inline const char* resourceName(int r) {
  static const char* names[NUM_RESOURCES] = {"Food", "Water", "Energy", "Fuel", "Medicine", "RareMinerals",
                                             "Construction"};
  return (r >= 0 && r < NUM_RESOURCES) ? names[r] : "Unknown";
}
// Accepts a resource name (case-insensitive prefix match) or a numeric index; -1 if unknown.
inline int resourceFromString(const std::string& s) {
  if (s.empty()) return -1;
  bool numeric = std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; });
  if (numeric) {
    int v = std::atoi(s.c_str());
    return (v >= 0 && v < NUM_RESOURCES) ? v : -1;
  }
  std::string l;
  for (char c : s) l.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  for (int r = 0; r < NUM_RESOURCES; ++r) {
    std::string n;
    for (const char* p = resourceName(r); *p; ++p) n.push_back(static_cast<char>(std::tolower(*p)));
    if (n == l || (l.size() >= 3 && n.compare(0, l.size(), l) == 0)) return r;
  }
  if (l == "rare_minerals" || l == "rare minerals" || l == "minerals") return 5;
  if (l == "construction_materials" || l == "construction materials" || l == "materials") return 6;
  return -1;
}

// Ship capability bits.
constexpr uint32_t CAP_CARGO = 1u, CAP_MEDICAL = 2u, CAP_ARMORED = 4u, CAP_LONG_RANGE = 8u;

// ---------------------------------------------------------------- RNG
// xoshiro256** seeded by splitmix64: deterministic across platforms.
class Rng {
 public:
  explicit Rng(uint64_t seed = 1) { reseed(seed); }
  void reseed(uint64_t seed) {
    uint64_t z = seed + 0x9E3779B97F4A7C15ULL;
    for (auto& v : s_) {
      z += 0x9E3779B97F4A7C15ULL;
      uint64_t x = z;
      x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
      x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
      v = x ^ (x >> 31);
    }
  }
  uint64_t next() {
    const uint64_t result = rotl(s_[1] * 5, 7) * 9;
    const uint64_t t = s_[1] << 17;
    s_[2] ^= s_[0];
    s_[3] ^= s_[1];
    s_[1] ^= s_[2];
    s_[0] ^= s_[3];
    s_[2] ^= t;
    s_[3] = rotl(s_[3], 45);
    return result;
  }
  double uniform() { return (next() >> 11) * (1.0 / 9007199254740992.0); }  // [0,1)
  double uniform(double a, double b) { return a + (b - a) * uniform(); }
  int uniformInt(int n) { return n <= 0 ? 0 : static_cast<int>(next() % static_cast<uint64_t>(n)); }
  double normal() {  // Box-Muller, one value per call (deterministic)
    double u1 = uniform(), u2 = uniform();
    if (u1 < 1e-300) u1 = 1e-300;
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(6.283185307179586 * u2);
  }
  double triangular(double a, double mode, double b) {
    if (b <= a) return a;
    mode = std::min(std::max(mode, a), b);
    double u = uniform(), f = (mode - a) / (b - a);
    return u < f ? a + std::sqrt(u * (b - a) * (mode - a)) : b - std::sqrt((1 - u) * (b - a) * (b - mode));
  }
  bool bernoulli(double p) { return uniform() < p; }

 private:
  static uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
  uint64_t s_[4];
};

// ---------------------------------------------------------------- Timing
class Timer {
 public:
  Timer() : start_(std::chrono::steady_clock::now()) {}
  void reset() { start_ = std::chrono::steady_clock::now(); }
  double ms() const {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_).count();
  }

 private:
  std::chrono::steady_clock::time_point start_;
};

// Absolute wall-clock deadline used for time boxes.
struct Deadline {
  std::chrono::steady_clock::time_point at;
  static Deadline inMs(double ms) {
    Deadline d;
    if (ms > 1e12) ms = 1e12;
    if (ms < 0) ms = 0;
    d.at = std::chrono::steady_clock::now() +
           std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double, std::milli>(ms));
    return d;
  }
  static Deadline never() { return inMs(1e12); }
  bool expired() const { return std::chrono::steady_clock::now() >= at; }
  double remainingMs() const {
    return std::chrono::duration<double, std::milli>(at - std::chrono::steady_clock::now()).count();
  }
  Deadline earlier(const Deadline& o) const { return o.at < at ? o : *this; }
};

// High-water resident set size of the process in MB (getrusage), 0 if unsupported.
inline double peakMemoryMB() {
#if defined(__APPLE__)
  struct rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  return ru.ru_maxrss / (1024.0 * 1024.0);  // bytes on macOS
#elif defined(__unix__)
  struct rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  return ru.ru_maxrss / 1024.0;  // kilobytes on Linux
#else
  return 0.0;
#endif
}
// Current resident set size in MB (/proc/self/statm on Linux, else the peak).
inline double currentMemoryMB() {
#if defined(__linux__)
  std::ifstream f("/proc/self/statm");
  long pages = 0, resident = 0;
  if (f >> pages >> resident) return resident * (static_cast<double>(sysconf(_SC_PAGESIZE)) / (1024.0 * 1024.0));
#endif
  return peakMemoryMB();
}

// ---------------------------------------------------------------- Counters
// Global search-effort counters (single-threaded program).
struct Counters {
  uint64_t nodesExplored = 0;        // vertices settled by any graph search
  uint64_t statesExplored = 0;       // all search states (nodes, labels, B&B nodes, insertion states)
  uint64_t candidatesEvaluated = 0;  // (mission,ship,position), infra options, flow arcs evaluated
  double naiveStates = 0;            // baseline effort (measured on samples or counted analytically)
  uint64_t routeQueries = 0, cacheHits = 0, cacheMisses = 0, cacheInvalidations = 0;
  uint64_t labelsCreated = 0, labelsPruned = 0;
  uint64_t candidatesPruned = 0, dominatedShips = 0, bnbNodes = 0;
  uint64_t routingSettled = 0;       // part of nodesExplored spent on route queries
  double sampleActual = 0, sampleNaive = 0;  // sampled plain-Dijkstra comparison
  uint64_t samples = 0;
  uint64_t altRebuilds = 0;
  uint64_t insertionEvals = 0;       // (mission, ship, position) insertion evaluations
  double replanNaive = 0;            // analytic cost of a full re-solve per event (insertion evaluations)
  void reset() { *this = Counters(); }
  uint64_t refuelAlongRoute = 0;  // fuel legs solved by refuelling along the best route
  uint64_t fuelSearches = 0, fuelSearchesFound = 0, fuelSearchesCapped = 0, fuelLabels = 0;  // diagnostics
  uint64_t paretoSearches = 0, paretoLabels = 0;
};
inline Counters& counters() {
  static Counters c;
  return c;
}

// ---------------------------------------------------------------- FlatIndex
// Open-addressing hash index uint64 key -> int32 value (linear probing, backward-shift
// deletion, power-of-two table kept at most 50% full). Replaces std::unordered_map on the
// memo lookups that run tens of millions of times per run: one cache line per probe instead
// of a node allocation per entry. Keys must not equal EMPTY (all ones).
class FlatIndex {
 public:
  static constexpr uint64_t EMPTY = ~0ULL;
  FlatIndex() { rehash(16); }
  size_t size() const { return size_; }
  size_t capacityBytes() const { return keys_.size() * (sizeof(uint64_t) + sizeof(int32_t)); }
  // returns the stored value or -1
  int32_t find(uint64_t k) const {
    size_t i = slot(k);
    while (true) {
      uint64_t c = keys_[i];
      if (c == k) return vals_[i];
      if (c == EMPTY) return -1;
      i = (i + 1) & mask_;
    }
  }
  // inserts or overwrites
  void put(uint64_t k, int32_t v) {
    if (2 * (size_ + 1) > keys_.size()) rehash(keys_.size() * 2);
    size_t i = slot(k);
    while (keys_[i] != EMPTY && keys_[i] != k) i = (i + 1) & mask_;
    if (keys_[i] == EMPTY) ++size_;
    keys_[i] = k;
    vals_[i] = v;
  }
  // removes k; returns its value or -1
  int32_t erase(uint64_t k) {
    size_t i = slot(k);
    while (keys_[i] != k) {
      if (keys_[i] == EMPTY) return -1;
      i = (i + 1) & mask_;
    }
    int32_t v = vals_[i];
    // backward-shift deletion keeps every probe chain unbroken without tombstones
    size_t j = i;
    while (true) {
      j = (j + 1) & mask_;
      if (keys_[j] == EMPTY) break;
      size_t h = slot(keys_[j]);
      // move keys_[j] into the hole at i unless its home h lies cyclically in (i, j]
      bool inRange = i <= j ? (h > i && h <= j) : (h > i || h <= j);
      if (!inRange) {
        keys_[i] = keys_[j];
        vals_[i] = vals_[j];
        i = j;
      }
    }
    keys_[i] = EMPTY;
    --size_;
    return v;
  }
  void clear() {
    if (keys_.size() > 1024) {  // shrink: drop the big table instead of re-filling it
      keys_.assign(16, EMPTY);
      vals_.assign(16, -1);
      mask_ = 15;
    } else {
      std::fill(keys_.begin(), keys_.end(), EMPTY);
    }
    size_ = 0;
  }
  template <class F>
  void forEach(F&& f) const {
    for (size_t i = 0; i < keys_.size(); ++i)
      if (keys_[i] != EMPTY) f(keys_[i], vals_[i]);
  }

 private:
  size_t slot(uint64_t k) const {
    k ^= k >> 33;
    k *= 0xff51afd7ed558ccdULL;
    k ^= k >> 33;
    return static_cast<size_t>(k) & mask_;
  }
  void rehash(size_t n) {
    std::vector<uint64_t> ok;
    std::vector<int32_t> ov;
    ok.swap(keys_);
    ov.swap(vals_);
    keys_.assign(n, EMPTY);
    vals_.assign(n, -1);
    mask_ = n - 1;
    size_ = 0;
    for (size_t i = 0; i < ok.size(); ++i)
      if (ok[i] != EMPTY) put(ok[i], ov[i]);
  }
  std::vector<uint64_t> keys_;
  std::vector<int32_t> vals_;
  size_t mask_ = 0, size_ = 0;
};

// ---------------------------------------------------------------- Utilities
inline uint64_t hashCombine(uint64_t h, uint64_t v) {
  v *= 0x9E3779B97F4A7C15ULL;
  v ^= v >> 32;
  return h ^ (v + 0x9E3779B97F4A7C15ULL + (h << 6) + (h >> 2));
}
inline double clamp01(double x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }
// Standard normal CDF.
inline double normalCdf(double z) { return 0.5 * std::erfc(-z / std::sqrt(2.0)); }
inline std::string jsonEscape(const std::string& s) {
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') { o.push_back('\\'); o.push_back(c); }
    else if (c == '\n') o += "\\n";
    else if (static_cast<unsigned char>(c) < 0x20) o.push_back(' ');
    else o.push_back(c);
  }
  return o;
}
// Finite-safe number formatting for CSV/JSON (non-finite values become -1).
inline std::string fmtDouble(double v, int prec = 4) {
  if (!std::isfinite(v)) return "-1";
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", prec, v);
  return buf;
}

// ---------------------------------------------------------------- Spatial grid
// Uniform 2-D bucket grid over (x,y). Build O(n); kNearest expands rings of
// cells until k items are found and the ring lower bound exceeds the k-th distance.
class SpatialGrid2D {
 public:
  void build(const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<int>& ids,
             double targetPerCell = 2.0) {
    ids_.clear(); px_.clear(); py_.clear(); start_.assign(2, 0); nx_ = ny_ = 1;
    size_t n = ids.size();
    if (n == 0) return;
    double maxX = -INF, maxY = -INF;
    minX_ = INF; minY_ = INF;
    for (size_t i = 0; i < n; ++i) {
      minX_ = std::min(minX_, xs[i]); maxX = std::max(maxX, xs[i]);
      minY_ = std::min(minY_, ys[i]); maxY = std::max(maxY, ys[i]);
    }
    double w = std::max(maxX - minX_, 1e-9), h = std::max(maxY - minY_, 1e-9);
    double cells = std::max(1.0, static_cast<double>(n) / std::max(0.5, targetPerCell));
    cell_ = std::max(std::sqrt(w * h / cells), 1e-6);
    nx_ = std::min(4096, std::max(1, static_cast<int>(w / cell_) + 1));
    ny_ = std::min(4096, std::max(1, static_cast<int>(h / cell_) + 1));
    cell_ = std::max(w / nx_, h / ny_) * 1.000001;
    std::vector<int> cellIdx(n);
    start_.assign(static_cast<size_t>(nx_) * ny_ + 1, 0);
    for (size_t i = 0; i < n; ++i) {
      cellIdx[i] = cellOf(xs[i], minX_, nx_) * ny_ + cellOf(ys[i], minY_, ny_);
      start_[cellIdx[i] + 1]++;
    }
    for (size_t c = 1; c < start_.size(); ++c) start_[c] += start_[c - 1];
    std::vector<int> pos(start_.begin(), start_.end() - 1);
    ids_.resize(n); px_.resize(n); py_.resize(n);
    for (size_t i = 0; i < n; ++i) {
      int p = pos[cellIdx[i]]++;
      ids_[p] = ids[i]; px_[p] = xs[i]; py_[p] = ys[i];
    }
  }
  // ids of up to k nearest items accepted by filter (filter(id) -> bool); sorted by distance.
  template <class F>
  void kNearest(double x, double y, int k, std::vector<int>& out, F&& filter) const;
  bool empty() const { return ids_.empty(); }

 private:
  int cellOf(double v, double lo, int n) const {
    int c = static_cast<int>((v - lo) / cell_);
    return c < 0 ? 0 : (c >= n ? n - 1 : c);
  }
  double minX_ = 0, minY_ = 0, cell_ = 1;
  int nx_ = 1, ny_ = 1;
  std::vector<int> start_;   // nx*ny+1 offsets
  std::vector<int> ids_;     // ids grouped by cell
  std::vector<double> px_, py_;
};

template <class F>
void SpatialGrid2D::kNearest(double x, double y, int k, std::vector<int>& out, F&& filter) const {
  out.clear();
  if (ids_.empty() || k <= 0) return;
  std::vector<std::pair<double, int>> found;
  int cx = cellOf(x, minX_, nx_), cy = cellOf(y, minY_, ny_);
  int maxR = std::max(nx_, ny_);
  for (int r = 0; r <= maxR; ++r) {
    for (int gx = cx - r; gx <= cx + r; ++gx) {
      if (gx < 0 || gx >= nx_) continue;
      for (int gy = cy - r; gy <= cy + r; ++gy) {
        if (gy < 0 || gy >= ny_) continue;
        if (std::max(std::abs(gx - cx), std::abs(gy - cy)) != r) continue;  // ring only
        int c = gx * ny_ + gy;
        for (int i = start_[c]; i < start_[c + 1]; ++i) {
          int id = ids_[i];
          if (!filter(id)) continue;
          double dx = px_[i] - x, dy = py_[i] - y;
          found.emplace_back(dx * dx + dy * dy, id);
        }
      }
    }
    if (static_cast<int>(found.size()) >= k) {
      std::nth_element(found.begin(), found.begin() + (k - 1), found.end());
      double kth = found[k - 1].first;
      double ringLb = r * cell_;  // anything outside ring r is at least this far
      if (ringLb * ringLb >= kth) break;
    }
  }
  std::sort(found.begin(), found.end());
  for (int i = 0; i < static_cast<int>(found.size()) && i < k; ++i) out.push_back(found[i].second);
}

}  // namespace chronos
