// test_main.cpp - runs every registered test (optionally only names containing argv[1]).
#include <cstring>

#include "test_util.hpp"

int main(int argc, char** argv) {
  const char* filter = argc > 1 ? argv[1] : nullptr;
  int ran = 0, failedTests = 0;
  for (const auto& t : testing::registry()) {
    if (filter && !std::strstr(t.name, filter)) continue;
    int before = testing::failures();
    chronos::Timer timer;
    t.fn();
    bool ok = testing::failures() == before;
    std::printf("[%s] %s (%.1f ms)\n", ok ? " OK " : "FAIL", t.name, timer.ms());
    ++ran;
    if (!ok) ++failedTests;
  }
  std::printf("\n%d tests, %d checks, %d failed checks, %d failed tests\n", ran, testing::checks(), testing::failures(),
              failedTests);
  return failedTests == 0 ? 0 : 1;
}
