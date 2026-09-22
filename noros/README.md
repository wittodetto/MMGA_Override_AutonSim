# override_sim_noros — Zero-Dependency Override Simulator

A **no-ROS** C++17 port of the `override_sim` ROS 2 package: same VEX V5RC
Override (2026-27) field, same SC1–SC7 scoring rules, same standardized Nav2
maps — but it builds and runs with **only a C++17 compiler**. No ROS 2, no
Gazebo, no Nav2, no colcon, no Python.

```
override_sim_noros/            (this directory)
├── CMakeLists.txt             standard CMake build (cmake >= 3.10)
├── build.sh                   zero-cmake build: clang++/g++ directly
├── include/noros_sim/
│   ├── field.h                field model: goals/loaders/toggles/quadrants,
│   │                          midfield diamond, 20-element starting layout
│   ├── map_loader.h           standard nav2_map_server PGM(P5)+YAML loader
│   ├── scoring.h              pure SC1–SC7 scoring (ported from scoring.cpp)
│   ├── sim.h                  match simulator core (robot, elements, phases)
│   └── render.h               ASCII field view + PPM snapshots
└── src/
    ├── main.cpp               CLI + match loop + score.csv logger
    ├── map_loader.cpp         PGM/YAML parsing, occupancy math
    ├── scoring.cpp            SC1–SC7 port (identical logic to ROS version)
    ├── sim.cpp                kinematics, collision, auton controller, actions
    └── render.cpp             ASCII + PPM P6 renderer
```

## Build

```bash
# option A: plain script (no cmake required)
./noros/build.sh

# option B: standard cmake
cmake -S noros -B noros/build && cmake --build noros/build -j
```

## Run

From the repository root (the map path is relative to the repo root):

```bash
./noros/build/override_sim_noros \
    --map maps/override_field_map.pgm \
    --auton north_goal \
    --duration 105 \
    --out score.csv \
    --frames frames/
```

### Built-in autonomous routines

All routines start from the red alliance side (x < 0). Verified final scores
(match log, 105 s, `--opponent none`):

| Routine | Verified score | Description |
|---|---|---|
| `north_goal` | **RED 27–0** | Climb the left lane, intake pin_ry_1, score on north neutral goal 1, flip north toggle red (+15 auton, +12 bonus) |
| `south_route` | **RED 22–5** | Intake pin_by_4, score on south red goal 4, flip south toggle red (blue keeps the blue top terminal) |
| `midfield_push` | **RED 14–6** | Park inside the Midfield diamond (+8 pts, tie bonus 6/6) |
| `west_goal` | **RED 27–0** | Intake pin_ry_2 at the west wall, score on west neutral goal 2, flip west toggle red |
| `center_goal` | **RED 40–0** | Intake pin_yy_1, score on the tall center goal, park in Midfield → red owns both yellow terminals (+20) +8 midfield +12 bonus |
| `cup_cover_test` | **RED 17–0** | Rule demo: score pin_ry_1 + flip north toggle (15 pts), then cover the pin with cup_1 (opaque half hides the yellow bottom terminal → 5 pts). Proves cup-coverage causality in one run |

### Options

```
--map <pgm>           standard nav2 PGM map (auto-detects maps/… or ../maps/…)
--yaml <yaml>         map metadata (default: same basename .yaml)
--auton <name>        routine: north_goal | south_route | midfield_push |
                      west_goal | center_goal | cup_cover_test
--autonomous on|off   enable the 15 s autonomous period (default on)
--elements on|off     spawn the official 20-element layout (default on)
--opponent <mode>     none | park | midfield (blue alliance bot)
--duration <s>        match length (default 105)
--dt <s>              simulation step (default 0.05)
--out <csv>           per-second score log (default score.csv)
--frames <dir>        PPM snapshots, one per second (default frames/)
--ascii               print the ASCII field view at the end
```

## Outputs

- **score.csv** — one row per 0.5 s:
  `time,phase,red,blue,red_pins,blue_pins,red_yellow,blue_yellow,red_mid,blue_mid,bonus_red,bonus_blue`
- **frames/frame_XXX.ppm** — 366×366 P6 snapshots (walls, goals, toggles,
  elements, robots). Convert with any image tool: `sips`/PIL/ImageMagick.
- **console** — auton action log (intake / place / flip), score detail,
  goal stacks, toggle states, final match summary.

## Fidelity notes

The simulator shares the ROS package's documented simplifications: cup
terminal-coverage uses cup yaw (opaque half), the field starts with the
official 20-element VEXcode VR layout, and the robot is a kinematic model
(waypoint following with a P-controller and **tangent-slide obstacle
avoidance** around goal bodies, map-based collision). The scoring code is a
direct port of `src/scoring.cpp`, so SC1–SC7 behave identically to the ROS 2
version. Auton waypoints are driven at up to 0.85 m/s with a 2.0·d braking
curve, so every competitive routine finishes inside the 15 s autonomous
period.

Known simplification: `cup_cover_test` proves the coverage rule but its cup
placement (intake → cover) is not a competitive 15 s strategy — it is a rules
verification scenario.
