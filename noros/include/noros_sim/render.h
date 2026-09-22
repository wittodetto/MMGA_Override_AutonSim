// render.h - zero-dependency visualization: ASCII field view + PPM snapshots.
#ifndef NOROS_SIM_RENDER_HPP_
#define NOROS_SIM_RENDER_HPP_

#include <string>

#include "noros_sim/map_loader.h"
#include "noros_sim/sim.h"

namespace noros_sim
{

// ASCII rendering of the field (map occupancy as background, robots,
// goals, toggles and field elements drawn on top). cols x rows characters.
std::string render_ascii(const Sim & sim, const MapData & map, int cols = 61, int rows = 31);

// PPM P6 snapshot (RGB, 8 bit). Renders the map with field overlays.
// Returns the output path.
std::string save_ppm(const Sim & sim, const MapData & map, const std::string & path);

}  // namespace noros_sim

#endif  // NOROS_SIM_RENDER_HPP_
