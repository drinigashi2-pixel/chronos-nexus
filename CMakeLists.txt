cmake_minimum_required(VERSION 3.13)
project(chronos_nexus LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
  add_compile_options(/O2 /W4)
else()
  add_compile_options(-O2 -Wall -Wextra)
endif()

add_library(chronos_core STATIC
  src/graph.cpp
  src/io.cpp
  src/routing.cpp
  src/flow.cpp
  src/scheduler.cpp
  src/infra.cpp
  src/predict.cpp
  src/replan.cpp
  src/meta.cpp
  src/score.cpp
  src/sim.cpp)
target_include_directories(chronos_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# std::filesystem needs an extra library on GCC < 9
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
  target_link_libraries(chronos_core PUBLIC stdc++fs)
endif()

add_executable(chronos src/main.cpp)
target_link_libraries(chronos PRIVATE chronos_core)

add_executable(gen_galaxy tools/gen_galaxy.cpp)
target_link_libraries(gen_galaxy PRIVATE chronos_core)

add_executable(run_tests
  tests/test_main.cpp
  tests/test_routing.cpp
  tests/test_flow.cpp
  tests/test_infra.cpp
  tests/test_scheduler.cpp
  tests/test_replan.cpp)
target_include_directories(run_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(run_tests PRIVATE chronos_core)

enable_testing()
add_test(NAME unit_tests COMMAND run_tests)
