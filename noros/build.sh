#!/usr/bin/env bash
# Zero-dependency build for override_sim_noros (no cmake required).
# Uses clang++/g++ directly. Produces ./build/override_sim_noros
set -euo pipefail
cd "$(dirname "$0")"

CXX="${CXX:-clang++}"
mkdir -p build
"$CXX" -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  -Iinclude \
  src/main.cpp src/map_loader.cpp src/scoring.cpp src/sim.cpp src/render.cpp \
  -o build/override_sim_noros

echo "built: $(pwd)/build/override_sim_noros"
