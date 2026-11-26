#!/usr/bin/env sh
# Configure, build (Release, -O2 -Wall -Wextra) and run the unit tests.
#   scripts/build.sh            build + tests
#   scripts/build.sh --no-tests build only
set -e
cd "$(dirname "$0")/.."
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j "$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)"
if [ "$1" != "--no-tests" ]; then
  ./build/run_tests
fi
echo "binaries: build/chronos build/gen_galaxy build/run_tests"
