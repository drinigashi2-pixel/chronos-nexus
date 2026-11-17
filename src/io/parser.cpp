#include "io/parser.hpp"
#include <fstream>

namespace chronos {
namespace {

bool fail(std::string* err, const std::string& m) { if (err) *err = m; return false; }

bool expect(std::ifstream& in, const char* word, size_t& count) {
    std::string w;
    if (!(in >> w) || w != word) return false;
    return static_cast<bool>(in >> count);
}

}  // namespace

bool load_galaxy(const std::string& path, Galaxy& g, std::string* err) {
    std::ifstream in(path);
    if (!in) return fail(err, "cannot open " + path);
    std::string magic; int ver;
    if (!(in >> magic >> ver) || magic != "CHRONOS") return fail(err, "bad header");

    size_t n = 0, m = 0, k = 0, q = 0, f = 0;

    if (!expect(in, "PLANETS", n)) return fail(err, "expected PLANETS");
    g.planets.resize(n);
    for (size_t i = 0; i < n; ++i) {
        Planet& p = g.planets[i];
        int type;
        if (!(in >> p.id >> type >> p.x >> p.y >> p.z >> p.population >> p.tech
              >> p.defense >> p.strategic)) return fail(err, "bad planet line");
        p.type = static_cast<LocationType>(type);
        for (int r = 0; r < NUM_RES; ++r) in >> p.production[r];
        for (int r = 0; r < NUM_RES; ++r) in >> p.consumption[r];
        for (int r = 0; r < NUM_RES; ++r) in >> p.stock[r];
        for (int r = 0; r < NUM_RES; ++r) in >> p.storage_cap[r];
        if (!in || p.id != static_cast<Id>(i)) return fail(err, "planet ids must be dense 0..n-1");
        g.graph.add_vertex(p.x, p.y, p.z);
    }

    if (!expect(in, "ROUTES", m)) return fail(err, "expected ROUTES");
    for (size_t i = 0; i < m; ++i) {
        Edge e; int av, wh;
        if (!(in >> e.from >> e.to >> e.distance >> e.time >> e.time_min >> e.time_max
              >> e.fuel >> e.risk >> e.capacity >> e.reliability >> av >> wh))
            return fail(err, "bad route line");
        if (e.from < 0 || e.to < 0 || e.from >= static_cast<Id>(n) || e.to >= static_cast<Id>(n))
            return fail(err, "route endpoint out of range");
        e.available = static_cast<uint8_t>(av);
        e.is_wormhole = static_cast<uint8_t>(wh);
        g.graph.add_edge(e);
    }

    if (!expect(in, "SPACECRAFT", k)) return fail(err, "expected SPACECRAFT");
    g.spacecraft.resize(k);
    for (auto& s : g.spacecraft)
        if (!(in >> s.id >> s.location >> s.cargo_cap >> s.fuel_cap >> s.speed >> s.fuel
              >> s.avail_time >> s.reliability >> s.op_cost >> s.caps))
            return fail(err, "bad spacecraft line");

    if (!expect(in, "MISSIONS", q)) return fail(err, "expected MISSIONS");
    g.missions.resize(q);
    for (auto& ms : g.missions)
        if (!(in >> ms.id >> ms.origin >> ms.dest >> ms.resource >> ms.amount >> ms.release
              >> ms.deadline >> ms.priority >> ms.importance >> ms.required_caps))
            return fail(err, "bad mission line");

    std::string w;
    if (!(in >> w >> g.budget) || w != "BUDGET") return fail(err, "expected BUDGET");

    if (!expect(in, "FACILITIES", f)) return fail(err, "expected FACILITIES");
    g.facility_catalog.resize(f);
    for (auto& fs : g.facility_catalog) {
        int t;
        if (!(in >> t >> fs.cost >> fs.pop_impact >> fs.energy_impact >> fs.strategic_value
              >> fs.build_time)) return fail(err, "bad facility line");
        fs.type = static_cast<FacilityType>(t);
        for (int r = 0; r < NUM_RES; ++r) in >> fs.resource_impact[r];
        if (!in) return fail(err, "bad facility line");
    }
    return true;
}

bool load_events(const std::string& path, std::vector<Event>& events, std::string* err) {
    std::ifstream in(path);
    if (!in) return fail(err, "cannot open " + path);
    size_t n = 0;
    if (!expect(in, "EVENTS", n)) return fail(err, "expected EVENTS");
    events.resize(n);
    for (auto& e : events) {
        int t;
        if (!(in >> e.time >> t >> e.target >> e.severity >> e.p1 >> e.p2))
            return fail(err, "bad event line");
        e.type = static_cast<EventType>(t);
    }
    return true;
}

}  // namespace chronos
