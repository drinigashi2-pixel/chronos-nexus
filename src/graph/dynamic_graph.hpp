#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
#include "graph/graph.hpp"

namespace chronos {

// Directed multigraph with O(1) edge updates. Adjacency stores edge ids, so an edge
// mutation never touches adjacency lists. Every mutation bumps the epoch and is appended
// to a change log so decision layers can find what became invalid since their last
// snapshot (keep a log mark, read change_log()[mark..]).
class DynamicGraph {
public:
    enum class ChangeKind : uint8_t { ADDED, AVAILABILITY, RISK, TIME, RELIABILITY };
    struct Change { Id edge; ChangeKind kind; uint64_t epoch; };

    Id add_vertex(double x, double y, double z) {
        coords_.push_back({x, y, z});
        out_.emplace_back();
        in_.emplace_back();
        return static_cast<Id>(coords_.size() - 1);
    }

    Id add_edge(const Edge& e) {
        Id id = static_cast<Id>(edges_.size());
        edges_.push_back(e);
        out_[e.from].push_back(id);
        in_[e.to].push_back(id);
        if (e.available) ++active_;
        log(id, ChangeKind::ADDED);
        return id;
    }

    size_t num_vertices() const { return coords_.size(); }
    size_t num_edges() const { return edges_.size(); }
    size_t num_active_edges() const { return active_; }

    const Edge& edge(Id e) const { return edges_[e]; }
    const Point& coord(Id v) const { return coords_[v]; }
    const std::vector<Id>& out_edges(Id v) const { return out_[v]; }
    const std::vector<Id>& in_edges(Id v) const { return in_[v]; }

    void set_available(Id e, bool a) {
        if ((edges_[e].available != 0) == a) return;
        edges_[e].available = a;
        active_ += a ? 1 : -1;
        log(e, ChangeKind::AVAILABILITY);
    }
    void set_risk(Id e, float r) { edges_[e].risk = r; log(e, ChangeKind::RISK); }
    void set_time(Id e, float t) { edges_[e].time = t; log(e, ChangeKind::TIME); }
    void set_reliability(Id e, float r) { edges_[e].reliability = r; log(e, ChangeKind::RELIABILITY); }

    uint64_t epoch() const { return epoch_; }
    const std::vector<Change>& change_log() const { return log_; }

private:
    void log(Id e, ChangeKind k) { log_.push_back({e, k, ++epoch_}); }

    std::vector<Point> coords_;
    std::vector<Edge> edges_;
    std::vector<std::vector<Id>> out_, in_;
    size_t active_ = 0;
    uint64_t epoch_ = 0;
    std::vector<Change> log_;
};

}  // namespace chronos
