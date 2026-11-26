# Dataset format

A dataset is a directory of CSV files with a header row. Column order does not matter, column
names are case-insensitive, unknown columns are ignored and missing columns fall back to the
defaults below. Lines starting with `#` and empty lines are skipped. Fields may be double-quoted.

Units: time in **hours**, planet rates in **units per day**, distances/fuel in abstract units.

| File | Required | Notes |
|---|---|---|
| `planets.csv` | yes | without at least one planet the run aborts |
| `routes.csv` | no | missing → graph without edges (everything unreachable, still runs) |
| `spacecraft.csv` | no | missing → no fleet |
| `missions.csv` | no | missing/empty → only proactive missions from the predictor/flow |
| `infrastructure.csv` | no | missing → no construction |
| `events.csv` | no | missing → static galaxy |
| `config.csv` | no | `key,value` rows: `budget`, `days` |

Robustness rules applied by the loader (`src/io.cpp`):

* duplicate planet / ship / mission ids → the later row is skipped and counted;
* rows referencing unknown planets → skipped and counted (`unknownRefs`);
* duplicate directed edges (same `from`,`to`) → only the first is kept (`duplicateEdges`);
* self loops → skipped; negative or non-numeric weights → clamped to 0 (zero weights are allowed);
* reliability given in percent (1..100) is divided by 100; clamped to [0.01, 1];
* `time_min`/`time_max` default to 0.8/1.3 × `time_mean`, and are ordered around the mean;
* events are stably sorted by time; references inside events are resolved when the event fires,
  unknown targets are ignored (no crash).

## planets.csv

| column | default | meaning |
|---|---|---|
| `id` | row index | external id (any integer) |
| `type` | `planet` | `planet`, `colony` or `station` (or 0/1/2). Stations always allow refuelling |
| `x`,`y`,`z` | 0 | coordinates (x,y drive the spatial grids) |
| `population` | 1000 | |
| `tech`, `defense` | 1 | technology and defense level |
| `strategic` | 1 | strategic importance (> 0) |
| `prod_<r>` | 0 | production per day of resource `<r>` |
| `cons_<r>` | 0 | consumption per day |
| `cap_<r>` | max(1000, 60 × max(prod, cons)) | storage capacity |
| `stock_<r>` | 10 × cons | current stock (clamped to capacity) |

`<r>` ∈ `food, water, energy, fuel, medicine, minerals, construction`.

## routes.csv

| column | default | meaning |
|---|---|---|
| `id` | row index | route id (events refer to it); duplicated ids get a synthetic id |
| `from`, `to` | — | planet ids (required) |
| `distance` | 1 | |
| `time_mean`, `time_min`, `time_max` | dist, 0.8×, 1.3× mean | travel time distribution (hours, at speed 1). σ = (max − min)/4 |
| `fuel` | distance | fuel consumed per traversal |
| `risk` | 0 | risk level in [0, 0.99] |
| `capacity` | 1e6 | max units per planning cycle (and per single shipment) |
| `reliability` | 1 | 1 − failure probability of one traversal |
| `available` | 1 | 0 = closed at start |
| `wormhole` | 0 | flag (informational; wormholes are ordinary edges with their own weights) |
| `bidirectional` | 0 | 1 = also create the reverse directed edge with the same attributes |

## spacecraft.csv

| column | default | meaning |
|---|---|---|
| `id` | row index | |
| `location` | — | planet id (required) |
| `cargo_capacity` | 500 | units per mission |
| `fuel_capacity`, `fuel` | 1000, full | tank size and current fuel |
| `speed` | 1 | travel time = route time / speed |
| `available_at` | 0 | hours |
| `reliability` | 0.99 | per mission |
| `operating_cost` | 1 | cost per travel hour |
| `capabilities` | 1 | bit mask: 1 cargo, 2 medical, 4 armored, 8 long range (cargo is always set) |

## missions.csv

| column | default | meaning |
|---|---|---|
| `id` | row number | |
| `origin`, `destination` | — | planet ids (required, different) |
| `resource` | 0 | name (`Food`, `Water`, …, `Construction`, case-insensitive prefix) or index 0..6 |
| `amount` | 100 | units; missions larger than the biggest capable ship are chunked at start |
| `release` | 0 | hour at which the mission becomes known/schedulable |
| `deadline` | release + 240 | hour |
| `priority` | 3 | 1..5 (5 = highest; ≥ 4 gets the 90 % on-time chance constraint) |
| `importance` | 0.5 | 0..1 (values > 1 are divided by 10) |
| `required_caps` | 1 | capability bit mask (see spacecraft) |

## infrastructure.csv

| column | default | meaning |
|---|---|---|
| `id` | row index | |
| `planet` | — | planet id |
| `type` | — | `EnergyPlant, Farm, Hospital, Factory, DefenseSystem, ResearchStation, Spaceport, MiningFacility` or 0..7 |
| `level` | 1 | options with the same (planet, type) are mutually exclusive (multiple-choice knapsack group) |
| `cost` | 100 | construction cost (budget units) |
| `build_days` | 5 | construction time |
| `impact_<r>` | 0 | change of production per day of resource `<r>` (negative = extra consumption) |
| `population_impact` | 0 | population added on completion |
| `energy_impact` | 0 | energy production change per day (negative = consumption) |
| `strategic_value` | 0 | |

Construction consumes `0.05 × cost` construction materials at the planet; this demand is fed to
the resource flow when the construction starts.

## events.csv

`time,type,target,severity,a,b,c,d` — `type` is the name below or its index.

| # | type | target | severity | a | b | c | d |
|---|---|---|---|---|---|---|---|
| 0 | `WormholeCollapse` | route id | — | — | — | — | — |
| 1 | `RouteUnavailable` | route id | — | duration h (0 = permanent) | | | |
| 2 | `PirateBlockade` | route id | risk added after the blockade ends (×0.5) | duration h (default 48) | | | |
| 3 | `ShipDestroyed` | ship id | — | | | | |
| 4 | `EmergencyMedical` | planet id | — | amount (default 100) | deadline offset h (default 48) | | |
| 5 | `ProductionDrop` | planet id | drop fraction (default 0.6) | | resource index (default 0, −1 = all) | | |
| 6 | `NewColony` | — | — | x | y | z | population (default 5000) |
| 7 | `PopulationSurge` | planet id | relative increase (0.5 = +50 %) | | | | |
| 8 | `ResourceUnavailable` | planet id (−1 = everywhere) | — | resource index | duration h (default 72) | | |
| 9 | `NewHighPriorityMission` | origin planet id | priority = 3 + round(2·severity), importance = max(0.5, severity) | destination planet id | resource index | amount (default 200) | deadline offset h (default 72) |
| 10 | `RouteDanger` | route id | added risk; reliability × (1 − severity/2) | | | | |
| 11 | `ShipFailure` | ship id | — | repair duration h (default 24) | | | |

Timed closures (1, 2) create an internal `RouteRestore` follow-up event.

## config.csv

```
key,value
budget,120000
days,14
```

If no budget is given the default is 10 % of the summed option costs; `days` defaults to 14 and
can be overridden with `--days`.

## Outputs (written to `--out`)

| file | content |
|---|---|
| `civilization_plan.md` | human-readable plan: meta decisions per cycle, allocation, infrastructure, missions, events, score |
| `spacecraft_assignments.csv` | ship → mission sequence with planned and actual times |
| `resource_allocation.csv` | every shipment of the min-cost-flow plan per day |
| `mission_schedule.csv` | every mission: ship, planned/actual times, P(on time), delivered, status, late/delayed flags |
| `route_selections.csv` | empty and loaded leg of each planned/executed mission: profile, algorithm, node list, metrics |
| `infrastructure_decisions.csv` | started facilities: method, trigger, start/complete, value |
| `predicted_shortages.csv` | per day: predicted shortages, lead time, delivery deadline, covered flag |
| `replanning_log.csv` | per event: invalid/created/cancelled/removed/reinserted/unchanged missions, time, cost before/after |
| `score.json` | the 12 normalized terms, weights, total |
| `stats.json` | the 19 required statistics + `extra` diagnostics |
| `cycles.csv`, `convergence.csv` | meta-layer decision per cycle, ALNS convergence of cycle 0 |
