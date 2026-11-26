# TODO

Implementation is complete. What remains is running the experiments and writing up the results,
plus some optional improvements.

## Required before submission

- [ ] Run `scripts/run_experiments.py` (add `--extreme` if the machine allows), then
      `scripts/plot_results.py`.
- [ ] Save the hardware description to `results/hardware.txt` (README §13), then run
      `python3 scripts/fill_readme.py`: it fills every results table and the observed column of §9
      from the measurements (re-runnable; it only replaces its own marked blocks).
- [ ] Write the remaining prose placeholders (scalability discussion §15, conclusions §17) from the
      filled tables.
- [ ] Commit `docs/figures/*.svg` once they have been generated (they are git-ignored by default).

## Possible improvements

- [x] ~~Exact empty-leg costs through cached one-to-many reverse trees per mission origin~~: tried
      and rejected. One reverse search out to the farthest of K candidate ships settles more nodes
      than K point-to-point searches (large: 65.4 -> 64.1).
- [ ] Bidirectional ALT (symmetric potentials), so ALT wins on wormhole-heavy graphs as well.
- [ ] A multi-commodity flow with shared fleet capacity, instead of solving each resource in turn.
- [ ] Ships carrying several cargoes at once (pickup-and-delivery with capacity).
- [x] Landmark rebuild restricted to the affected region when a new colony connects (in-place
      decrease-only repair, `AltIndex::repairEdge`).
- [ ] Calibrating the ALNS work model per machine (would trade away run-to-run determinism).
- [ ] Parallel evaluation of insertion candidates. Everything is single-threaded today, which
      keeps runs deterministic.
