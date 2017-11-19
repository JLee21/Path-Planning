project(hybrid-a-star)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11 -O3)

set(sources
    src/main.cpp
    src/hybrid_breadth_first.cpp)

add_executable(hybrid ${sources})
