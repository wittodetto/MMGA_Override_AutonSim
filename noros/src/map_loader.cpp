// map_loader.cpp - minimal P5-PGM + YAML loader (nav2 map_server format).
#include "noros_sim/map_loader.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace noros_sim
{

int MapData::cell(int px, int py) const
{
  if (px < 0 || py < 0 || px >= width || py >= height) {
    return 1;   // outside the map: treat as occupied (field is wall-bounded)
  }
  const int v = pixels[static_cast<size_t>(py) * width + px];
  // nav2 map_server convention: 0 (black) = occupied, 254 (white) = free.
  // occupancy probability p = (255 - v)/255 when negate=false, else v/255.
  const double p = negate ? v / 255.0 : (255.0 - v) / 255.0;
  if (p >= occupied_thresh) {
    return 1;   // occupied
  }
  if (p <= free_thresh) {
    return 0;   // free
  }
  return 2;     // unknown
}

int MapData::occupied(double wx, double wy) const
{
  // map_server convention: the image's top row is the highest world y.
  const double px_f = (wx - origin_x) / resolution;
  const double py_f = (wy - origin_y) / resolution;
  const int px = static_cast<int>(std::lround(px_f));
  const int py = static_cast<int>(std::lround(static_cast<double>(height - 1) - py_f));
  return cell(px, py);
}

// ---- PGM P5 ----
static std::vector<uint8_t> load_pgm(const std::string & path, int * w, int * h)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    throw std::runtime_error("cannot open PGM: " + path);
  }
  auto next_token = [&f]() -> std::string {
    std::string tok;
    char c;
    while (f.get(c)) {
      if (c == '#') {   // skip comment to end of line
        while (f.get(c) && c != '\n') {}
        continue;
      }
      if (isspace(static_cast<unsigned char>(c))) {
        if (!tok.empty()) return tok;
        continue;
      }
      tok.push_back(c);
    }
    return tok;
  };

  const std::string magic = next_token();
  if (magic != "P5") {
    throw std::runtime_error("expected P5 PGM, got '" + magic + "' in " + path);
  }
  const std::string ws = next_token();
  const std::string hs = next_token();
  const std::string ms = next_token();
  try {
    *w = std::stoi(ws);
    *h = std::stoi(hs);
    (void)std::stoi(ms);   // maxval; we assume <= 255
  } catch (const std::exception &) {
    throw std::runtime_error("bad PGM header in " + path);
  }
  if (*w <= 0 || *h <= 0 || *w > 10000 || *h > 10000) {
    throw std::runtime_error("implausible PGM dimensions in " + path);
  }

  std::vector<uint8_t> data(static_cast<size_t>(*w) * (*h));
  f.read(reinterpret_cast<char *>(data.data()),
    static_cast<std::streamsize>(data.size()));
  if (f.gcount() != static_cast<std::streamsize>(data.size())) {
    throw std::runtime_error("truncated PGM data in " + path);
  }
  return data;
}

// ---- minimal YAML (only "key: value" scalar lines we care about) ----
static double yaml_double(
  const std::string & line, const std::string & key, double def)
{
  const std::string prefix = key + ":";
  const auto p = line.find(prefix);
  if (p == std::string::npos) {
    return def;
  }
  std::string v = line.substr(p + prefix.size());
  // strip comments and whitespace
  const auto hash = v.find('#');
  if (hash != std::string::npos) {
    v = v.substr(0, hash);
  }
  while (!v.empty() && isspace(static_cast<unsigned char>(v.front()))) {
    v.erase(v.begin());
  }
  while (!v.empty() && isspace(static_cast<unsigned char>(v.back()))) {
    v.pop_back();
  }
  if (v.empty()) {
    return def;
  }
  return std::stod(v);
}

static std::string yaml_str(
  const std::string & line, const std::string & key, const std::string & def)
{
  const std::string prefix = key + ":";
  const auto p = line.find(prefix);
  if (p == std::string::npos) {
    return def;
  }
  std::string v = line.substr(p + prefix.size());
  const auto hash = v.find('#');
  if (hash != std::string::npos) {
    v = v.substr(0, hash);
  }
  while (!v.empty() && isspace(static_cast<unsigned char>(v.front()))) {
    v.erase(v.begin());
  }
  while (!v.empty() && isspace(static_cast<unsigned char>(v.back()))) {
    v.pop_back();
  }
  if (v.size() >= 2 && v.front() == '"' && v.back() == '"') {
    v = v.substr(1, v.size() - 2);
  }
  return v;
}

MapData load_map(const std::string & pgm_path, const std::string & yaml_path)
{
  MapData m;
  m.pixels = load_pgm(pgm_path, &m.width, &m.height);

  if (!yaml_path.empty()) {
    std::ifstream f(yaml_path);
    if (!f) {
      throw std::runtime_error("cannot open map YAML: " + yaml_path);
    }
    std::string line;
    while (std::getline(f, line)) {
      const std::string trimmed =
        line.substr(0, line.find_first_of(" \t") == std::string::npos
          ? line.size() : line.find_first_of(" \t"));
      if (trimmed == "image:") {   // skip the image key itself
        continue;
      }
      if (line.find("resolution") == 0) {
        m.resolution = yaml_double(line, "resolution", m.resolution);
      } else if (line.find("origin") == 0) {
        // origin: [x, y, yaw]
        const auto ob = line.find('[');
        const auto oe = line.find(']');
        if (ob != std::string::npos && oe != std::string::npos) {
          std::string nums = line.substr(ob + 1, oe - ob - 1);
          std::istringstream iss(nums);
          std::string tok;
          int i = 0;
          while (std::getline(iss, tok, ',')) {
            while (!tok.empty() && isspace(static_cast<unsigned char>(tok.front()))) {
              tok.erase(tok.begin());
            }
            if (i == 0) m.origin_x = std::stod(tok);
            else if (i == 1) m.origin_y = std::stod(tok);
            ++i;
          }
        }
      } else if (line.find("occupied_thresh") == 0) {
        m.occupied_thresh = yaml_double(line, "occupied_thresh", m.occupied_thresh);
      } else if (line.find("free_thresh") == 0) {
        m.free_thresh = yaml_double(line, "free_thresh", m.free_thresh);
      } else if (line.find("negate") == 0) {
        m.negate = yaml_double(line, "negate", m.negate ? 1.0 : 0.0) != 0.0;
      } else if (line.find("mode") == 0) {
        m.mode = yaml_str(line, "mode", m.mode);
      }
    }
  }
  return m;
}

}  // namespace noros_sim
