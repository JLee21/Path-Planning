project(behavior-planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11 -O3)

set(sources
    src/main.cpp
    src/road.cpp
    src/vehicle.cpp
    src/cost.cpp)

add_executable(run ${sources})
