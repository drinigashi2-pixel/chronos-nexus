// Temporary driver: loads a dataset and prints a summary (sanity check for the generator).
#include <cstdio>
#include <queue>
#include "io/parser.hpp"

using namespace chronos;

int main(int argc, char** argv) {
    if (argc < 3) { std::fprintf(stderr, "usage: %s galaxy.txt events.txt\n", argv[0]); return 1; }
    Galaxy g; std::string err;
    if (!load_galaxy(argv[1], g, &err) || !load_events(argv[2], g.events, &err)) {
        std::fprintf(stderr, "error: %s\n", err.c_str()); return 1;
    }
    // BFS over active edges from vertex 0 to check connectivity.
    std::vector<char> seen(g.graph.num_vertices(), 0);
    std::queue<Id> q; q.push(0); seen[0] = 1; size_t reached = 1;
    while (!q.empty()) {
        Id v = q.front(); q.pop();
        for (Id e : g.graph.out_edges(v)) {
            const Edge& ed = g.graph.edge(e);
            if (ed.available && !seen[ed.to]) { seen[ed.to] = 1; ++reached; q.push(ed.to); }
        }
    }
    size_t deficits = 0;
    for (auto& p : g.planets) for (int r = 0; r < NUM_RES; ++r)
        if (p.days_to_shortage(r) < 30) ++deficits;
    std::printf("planets=%zu routes=%zu spacecraft=%zu missions=%zu events=%zu budget=%.0f\n",
                g.planets.size(), g.graph.num_edges(), g.spacecraft.size(),
                g.missions.size(), g.events.size(), g.budget);
    std::printf("reachable from 0: %zu / %zu\n", reached, g.graph.num_vertices());
    std::printf("(planet,resource) pairs with shortage < 30 days: %zu\n", deficits);
    return 0;
}
