// predict.hpp - proactive shortage / overflow prediction.
//
// For each (planet, resource): time-to-shortage = (stock - reserve) / net daily loss,
// time-to-overflow = (capacity - stock) / net daily gain. Entries live in a min-heap
// with version stamps (lazy deletion): rebuild O(P R log(P R)); update of one planet
// O(R log n); query of the k entries inside a horizon O(k log n).
// Delivery deadline = shortage time - lead time, lead = route time + z*sigma + margin.
#pragma once
#include <queue>
#include <vector>

#include "chronos/graph.hpp"

namespace chronos {

struct ShortagePrediction {
  NodeId planet = 0;
  int resource = 0;
  double stock = 0, netRatePerDay = 0;
  double timeToShortageH = INF, timeToOverflowH = INF;
  double leadTimeH = 0, deliveryDeadlineH = INF;
  double need = 0;  // units to cover coverDays beyond the shortage point
};

class Predictor {
 public:
  explicit Predictor(const World& w) : w_(w) {}
  void setReserveDays(double d) { reserveDays_ = d; }
  void setDefaultLead(double h) { defaultLead_ = h; }
  void setCoverDays(double d) { coverDays_ = d; }
  void rebuild(double now);
  void update(NodeId planet, double now);
  void setLeadTime(NodeId planet, double hours);
  double leadTime(NodeId planet) const;
  double timeToShortageH(NodeId p, int r) const;  // absolute hours (INF = never)
  double timeToOverflowH(NodeId p, int r) const;
  // All predicted shortages with time-to-shortage <= now + horizonH, earliest first.
  std::vector<ShortagePrediction> upcoming(double now, double horizonH);
  size_t heapSize() const { return heap_.size(); }

 private:
  struct Entry {
    double t;
    int key;
    uint32_t ver;
    bool operator>(const Entry& o) const { return t > o.t; }
  };
  void computeKey(int key, double now);
  const World& w_;
  double reserveDays_ = 2, defaultLead_ = 48, coverDays_ = 10;
  std::vector<double> shortAt_, overAt_, lead_;
  std::vector<uint32_t> ver_;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> heap_;
};

}  // namespace chronos
