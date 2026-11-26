#!/bin/sh
# Quick regression check (~12 min on a 12-thread laptop):
#   extreme seed 1, 3 days (same per-day budget as the full 900 s run), with the DIAG work split
#   large   seed 1 at its official budget (180 s)
#   medium  seeds 1-3 at their official budget (60 s)
# Extra arguments are passed to every run.
set -e
cd "$(dirname "$0")/.."
[ -x build/chronos ] || sh scripts/build.sh
[ -d data/extreme_s1 ] || build/gen_galaxy extreme data/extreme_s1 1 >/dev/null
[ -d data/large_s1 ] || build/gen_galaxy large data/large_s1 1 >/dev/null
for s in 1 2 3; do [ -d data/medium_s$s ] || build/gen_galaxy medium data/medium_s$s $s >/dev/null; done


echo "== extreme seed 1, 3 days"
CHRONOS_DIAG=1 build/chronos --dataset data/extreme_s1 --out /tmp/bq_extreme --time-ms 193000 --mem-mb 8192 \
  --days 3 "$@" 2>&1 | grep -E "^DIAG day 2|score="
echo "== large seed 1, 180 s"
build/chronos --dataset data/large_s1 --out /tmp/bq_large --time-ms 180000 --mem-mb 4096 --quiet "$@"
echo "== medium seeds 1-3, 60 s"
for s in 1 2 3; do
  build/chronos --dataset data/medium_s$s --out /tmp/bq_medium_$s --time-ms 60000 --mem-mb 2048 --quiet "$@"
done
