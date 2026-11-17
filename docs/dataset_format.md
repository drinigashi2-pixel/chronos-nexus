# Dataset format

Whitespace-separated text. `galaxy.txt` sections in this order, each with a count:

```
CHRONOS 1
PLANETS n
  id type x y z population tech defense strategic  prod[7] cons[7] stock[7] cap[7]
ROUTES m
  from to distance time time_min time_max fuel risk capacity reliability available is_wormhole
SPACECRAFT k
  id location cargo_cap fuel_cap speed fuel avail_time reliability op_cost caps
MISSIONS q
  id origin dest resource amount release deadline priority importance required_caps
BUDGET b
FACILITIES f
  type cost pop_impact energy_impact strategic_value build_days  resource_impact[7]
```
`events.txt`: `EVENTS e` then `time type target severity p1 p2` (sorted by time).

Resource order: Food Water Energy Fuel Medicine RareMinerals Construction.
Planet `type`: 0 planet, 1 colony, 2 station. Planet id == graph vertex id.
Capabilities bitmask: 1 cargo, 2 medical, 4 armed, 8 heavy. Rates are per day, times in hours.
Edge `time` is for a speed-1.0 ship; actual time = time / ship.speed.
Event types and the meaning of target/p1/p2: `src/model/event.hpp`.
Routes are directed; the generator guarantees weak connectivity through a bidirectional backbone.
Wormholes have distance 10-30% of euclidean, so a euclidean A* heuristic is not admissible there.
