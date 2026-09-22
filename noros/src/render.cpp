// render.cpp - ASCII field view + PPM P6 snapshot renderer.
#include "noros_sim/render.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

#include "noros_sim/field.h"

namespace noros_sim
{

// ---- ASCII -----------------------------------------------------------------
std::string render_ascii(
  const Sim & sim, const MapData & map, int cols, int rows)
{
  const double span = 2.0 * kFieldHalf;
  std::vector<std::string> g(rows, std::string(cols, ' '));
  auto put = [&](double x, double y, char c) {
    const int cx = static_cast<int>(((x + kFieldHalf) / span) * (cols - 1));
    const int cy = static_cast<int>(((kFieldHalf - y) / span) * (rows - 1));
    if (cx >= 0 && cx < cols && cy >= 0 && cy < rows) {
      g[cy][cx] = c;
    }
  };

  // map occupancy as background dots
  for (int py = 0; py < map.height; py += std::max(1, map.height / (rows * 2))) {
    for (int px = 0; px < map.width; px += std::max(1, map.width / (cols * 2))) {
      if (map.cell(px, py) == 1) {
        const double wx = map.origin_x + px * map.resolution;
        const double wy = map.origin_y + (map.height - 1 - py) * map.resolution;
        put(wx, wy, '#');
      }
    }
  }

  // goals
  for (const auto & kv : kGoals) {
    const GoalInfo & g = kv.second;
    char c = 'G';
    if (g.type == 0) c = 'C';
    else if (g.type == 2) c = 'R';
    else if (g.type == 3) c = 'B';
    put(g.x, g.y, c);
  }
  // toggles
  for (const auto & kv : kToggles) {
    put(kv.second[0], kv.second[1], 'T');
  }
  // field elements
  for (const auto & e : sim.elements) {
    const char c = e.type == 1 ? 'p' : 'c';
    put(e.x, e.y, c);
  }
  // robots
  put(sim.robot.x, sim.robot.y, 'R');
  put(sim.opponent.x, sim.opponent.y, 'O');

  std::string out;
  for (const auto & row : g) {
    out += row;
    out += '\n';
  }
  return out;
}

// ---- PPM P6 ----------------------------------------------------------------
std::string save_ppm(const Sim & sim, const MapData & map, const std::string & path)
{
  const int W = map.width;
  const int H = map.height;
  std::vector<uint8_t> img(static_cast<size_t>(W) * H * 3);

  // base layer: free = dark floor, occupied = walls
  for (int py = 0; py < H; ++py) {
    for (int px = 0; px < W; ++px) {
      const size_t o = (static_cast<size_t>(py) * W + px) * 3;
      const int c = map.cell(px, py);
      if (c == 1) {
        img[o] = 90; img[o + 1] = 90; img[o + 2] = 100;      // walls
      } else if (c == 2) {
        img[o] = 70; img[o + 1] = 70; img[o + 2] = 75;       // unknown
      } else {
        img[o] = 30; img[o + 1] = 30; img[o + 2] = 34;       // floor
      }
    }
  }

  auto paint = [&](double wx, double wy, int r, int g_, int b, int radius) {
    const double px_f = (wx - map.origin_x) / map.resolution;
    const double py_f = (map.height - 1) - (wy - map.origin_y) / map.resolution;
    const int cx = static_cast<int>(std::lround(px_f));
    const int cy = static_cast<int>(std::lround(py_f));
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (dx * dx + dy * dy > radius * radius) continue;
        const int px = cx + dx;
        const int py = cy + dy;
        if (px < 0 || py < 0 || px >= W || py >= H) continue;
        const size_t o = (static_cast<size_t>(py) * W + px) * 3;
        img[o] = static_cast<uint8_t>(r);
        img[o + 1] = static_cast<uint8_t>(g_);
        img[o + 2] = static_cast<uint8_t>(b);
      }
    }
  };

  const int rad = std::max(2, static_cast<int>(0.05 / map.resolution));
  for (const auto & kv : kGoals) {
    const GoalInfo & g = kv.second;
    paint(g.x, g.y, 200, 180, 60, rad);          // goals: amber
  }
  for (const auto & kv : kToggles) {
    paint(kv.second[0], kv.second[1], 240, 140, 20, rad);   // toggles: orange
  }
  for (const auto & e : sim.elements) {
    int col[3] = {200, 200, 200};
    if (e.type == 1) {
      if (e.top == 1) { col[0] = 230; col[1] = 60; col[2] = 60; }
      else if (e.top == 2) { col[0] = 60; col[1] = 110; col[2] = 230; }
      else { col[0] = 240; col[1] = 220; col[2] = 60; }
    } else {
      col[0] = 150; col[1] = 150; col[2] = 190;
    }
    paint(e.x, e.y, col[0], col[1], col[2], rad);
  }
  // robots: red = R, blue = O, radius larger
  const int rrad = std::max(4, static_cast<int>(0.12 / map.resolution));
  paint(sim.robot.x, sim.robot.y, 235, 40, 40, rrad);
  paint(sim.opponent.x, sim.opponent.y, 40, 90, 235, rrad);

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    return "";
  }
  f << "P6\n" << W << " " << H << "\n255\n";
  f.write(reinterpret_cast<const char *>(img.data()),
    static_cast<std::streamsize>(img.size()));
  return path;
}

}  // namespace noros_sim
