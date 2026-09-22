// map_loader.h - load a standard nav2_map_server PGM (P5 binary) + YAML pair
// without ROS. Same file format the ROS package ships in maps/:
//   override_field_map.pgm/.yaml, override_keepout.pgm/.yaml
#ifndef NOROS_SIM_MAP_LOADER_HPP_
#define NOROS_SIM_MAP_LOADER_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace noros_sim
{

struct MapData
{
  int width = 0;
  int height = 0;
  double resolution = 0.01;   // meters per pixel
  double origin_x = 0.0;      // world x of pixel (0,0) bottom row
  double origin_y = 0.0;      // world y of pixel (0,0) bottom row
  double occupied_thresh = 0.65;
  double free_thresh = 0.196;
  bool negate = false;
  std::string mode = "trinary";
  std::vector<uint8_t> pixels;   // raw PGM values (0..255), row-major top-down

  bool empty() const { return pixels.empty(); }

  // occupancy status at a pixel: 0 = free, 1 = occupied, 2 = unknown
  int cell(int px, int py) const;

  // occupancy status at world coordinates (meters)
  int occupied(double wx, double wy) const;
};

// Loads the .pgm (P5 binary, comments tolerated) and its sibling .yaml
// (simple "key: value" lines; only the fields we need are read).
// The .yaml path may be omitted: then defaults are used and only the pgm
// pixel data is trusted.
MapData load_map(const std::string & pgm_path, const std::string & yaml_path);

}  // namespace noros_sim

#endif  // NOROS_SIM_MAP_LOADER_HPP_
