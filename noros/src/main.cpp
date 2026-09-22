// main.cpp - override_sim_noros: zero-dependency Override (VEX V5RC 2026-27)
// autonomous-routine simulator. No ROS 2 / Gazebo / Nav2 needed.
//
// Usage (from the repository root):
//   ./noros/build/override_sim_noros --map maps/override_field_map.pgm \
//       --auton north_goal --duration 105 --out score.csv --frames frames/
//
// Outputs:
//   - score.csv : per-second score log (SC1-SC7)
//   - frames/   : PPM snapshots (one per second) — convert with any image tool
//   - console   : ASCII field view + match summary
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "noros_sim/field.h"
#include "noros_sim/map_loader.h"
#include "noros_sim/render.h"
#include "noros_sim/scoring.h"
#include "noros_sim/sim.h"

using namespace noros_sim;

namespace
{

struct Options
{
  std::string map_pgm;
  std::string map_yaml;
  std::string auton = "north_goal";
  bool autonomous = true;
  bool with_elements = true;
  std::string opponent = "none";
  double duration = 105.0;
  double dt = 0.05;
  std::string out_csv = "score.csv";
  std::string frames_dir = "frames";
  bool ascii = false;
};

Options parse_args(int argc, char ** argv)
{
  Options o;
  auto need = [&](int & i, const std::string & flag) -> std::string {
    if (i + 1 >= argc) {
      std::fprintf(stderr, "missing value for %s\n", flag.c_str());
      std::exit(2);
    }
    return argv[++i];
  };
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--map") o.map_pgm = need(i, a);
    else if (a == "--yaml") o.map_yaml = need(i, a);
    else if (a == "--auton") o.auton = need(i, a);
    else if (a == "--autonomous") o.autonomous = need(i, a) == "on";
    else if (a == "--elements") o.with_elements = need(i, a) == "on";
    else if (a == "--opponent") o.opponent = need(i, a);
    else if (a == "--duration") o.duration = std::atof(need(i, a).c_str());
    else if (a == "--dt") o.dt = std::atof(need(i, a).c_str());
    else if (a == "--out") o.out_csv = need(i, a);
    else if (a == "--frames") o.frames_dir = need(i, a);
    else if (a == "--ascii") o.ascii = true;
    else if (a == "--help" || a == "-h") {
      std::printf(
        "override_sim_noros — zero-dependency Override auton simulator\n"
        "  --map <pgm>       standard nav2 PGM map (default: auto-detect)\n"
        "  --yaml <yaml>     map metadata (default: same basename .yaml)\n"
        "  --auton <name>    routine: north_goal | south_route | midfield_push\n"
        "  --autonomous on|off   enable 15 s auton period (default on)\n"
        "  --elements on|off     spawn the 20-element layout (default on)\n"
        "  --opponent <mode> none | park | midfield\n"
        "  --duration <s>    match length (default 105)\n"
        "  --dt <s>          simulation step (default 0.05)\n"
        "  --out <csv>       score log path (default score.csv)\n"
        "  --frames <dir>    PPM snapshot directory (default frames)\n"
        "  --ascii           print the ASCII field view at the end\n");
      std::exit(0);
    } else {
      std::fprintf(stderr, "unknown option: %s\n", a.c_str());
      std::exit(2);
    }
  }
  return o;
}

std::string find_map(const Options & o)
{
  if (!o.map_pgm.empty()) {
    return o.map_pgm;
  }
  const char * candidates[] = {
    "maps/override_field_map.pgm",
    "../maps/override_field_map.pgm",
  };
  for (const char * c : candidates) {
    std::ifstream f(c);
    if (f.good()) {
      return c;
    }
  }
  return "";
}

}  // namespace

int main(int argc, char ** argv)
{
  const Options o = parse_args(argc, argv);

  const AutonRoutine * auton = find_auton(o.auton);
  if (auton == nullptr) {
    std::fprintf(stderr, "unknown auton routine '%s'\n", o.auton.c_str());
    return 2;
  }
  std::printf("== override_sim_noros ==\n  auton: %s (%s)\n", auton->name.c_str(),
    auton->description.c_str());

  // ---- load the standardized map ----
  MapData map;
  std::string pgm = find_map(o);
  if (!pgm.empty()) {
    std::string yaml = o.map_yaml;
    if (yaml.empty()) {
      yaml = pgm.substr(0, pgm.size() - 4) + ".yaml";
    }
    map = load_map(pgm, yaml);
    std::printf("  map: %s (%dx%d, res %.3f m/px, origin %.2f %.2f)\n",
      pgm.c_str(), map.width, map.height, map.resolution,
      map.origin_x, map.origin_y);
  } else {
    std::printf("  map: none (field-geometry collision only)\n");
  }

  // ---- run the match ----
  Sim sim(map.empty() ? nullptr : &map, o.with_elements, o.autonomous,
    o.opponent != "none", o.opponent);
  sim.reset(*auton, o.opponent);

  std::FILE * csv = std::fopen(o.out_csv.c_str(), "w");
  if (csv == nullptr) {
    std::fprintf(stderr, "cannot open %s\n", o.out_csv.c_str());
    return 2;
  }
  std::fprintf(csv,
    "time,phase,red,blue,red_pins,blue_pins,red_yellow,blue_yellow,"
    "red_mid,blue_mid,bonus_red,bonus_blue\n");

  if (!o.frames_dir.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(o.frames_dir, ec);
    if (ec) {
      std::fprintf(stderr, "cannot create frames dir: %s\n",
        ec.message().c_str());
    }
  }

  double last_log = -1.0;
  while (sim.t < o.duration) {
    sim.step(o.dt);
    if (sim.t - last_log >= 0.5 || sim.t >= o.duration) {
      const ScoreDetail d = sim.current_score();
      std::fprintf(csv, "%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
        sim.t, sim.phase, d.red, d.blue,
        d.red_pins, d.blue_pins, d.red_yellow, d.blue_yellow,
        d.red_mid, d.blue_mid, d.auton_bonus_red, d.auton_bonus_blue);
      last_log = sim.t;
    }
    if (!o.frames_dir.empty() && sim.t < 60.0) {
      static int last_frame = -1;
      const int sec = static_cast<int>(sim.t);
      if (sec != last_frame) {
        last_frame = sec;
        save_ppm(sim, map, o.frames_dir + "/frame_" +
          std::string(3 - std::to_string(sec).size(), '0') +
          std::to_string(sec) + ".ppm");
      }
    }
  }
  std::fclose(csv);

  // ---- summary ----
  const ScoreDetail final = sim.current_score();
  std::printf("\n== match summary (t = %.1f s) ==\n", sim.t);
  std::printf("  phase: %s\n", sim.phase == 0 ? "AUTONOMOUS" : "DRIVER");
  std::printf("  score: RED %d  -  BLUE %d\n", final.red, final.blue);
  std::printf("    red pins %d | blue pins %d | red yellow %d | blue yellow %d\n",
    final.red_pins, final.blue_pins, final.red_yellow, final.blue_yellow);
  std::printf("    midfield: red %d | blue %d\n", final.red_mid, final.blue_mid);
  std::printf("    auton bonus: red %d | blue %d (auton snap red %d / blue %d)\n",
    final.auton_bonus_red, final.auton_bonus_blue,
    sim.auton_red_pins, sim.auton_blue_pins);
  std::printf("  robot intake: %zu\n", sim.robot.intake.size());
  std::printf("  elements remaining: %d field + stacked on goals\n",
    static_cast<int>(sim.elements.size()));
  for (const auto & kv : kGoals) {
    const int n = sim.stack_count(kv.first);
    if (n > 0) {
      std::printf("    goal %d (%s): %d element(s)\n",
        kv.first, kv.second.quadrant.c_str(), n);
    }
  }
  std::printf("  toggles: N=%d E=%d S=%d W=%d (0=yellow 1=red 2=blue)\n",
    sim.toggles[0], sim.toggles[1], sim.toggles[2], sim.toggles[3]);

  if (o.ascii) {
    std::printf("\n%s\n", render_ascii(sim, map).c_str());
  }
  std::printf("\nscore log: %s\n", o.out_csv.c_str());
  return 0;
}
