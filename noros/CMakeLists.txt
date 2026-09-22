cmake_minimum_required(VERSION 3.10)
project(override_sim_noros VERSION 1.0.0 LANGUAGES CXX)

# Zero-dependency Override simulator (no ROS 2 / Gazebo / Nav2 required).
# Requires only a C++17 compiler.
#
#   cmake -S noros -B noros/build
#   cmake --build noros/build -j
#   ./noros/build/override_sim_noros --map maps/override_field_map.pgm --auton north_goal

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

add_executable(override_sim_noros
  src/main.cpp
  src/map_loader.cpp
  src/scoring.cpp
  src/sim.cpp
  src/render.cpp
)

target_include_directories(override_sim_noros PRIVATE include)
