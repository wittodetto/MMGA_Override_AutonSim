// field_constants.h - shared Override field data + quaternion helpers
// for the C++ override_sim nodes. Mirrors the constants previously defined
// in the Python nodes (field_location.py / world_services.py).
#ifndef OVERRIDE_SIM_FIELD_CONSTANTS_HPP_
#define OVERRIDE_SIM_FIELD_CONSTANTS_HPP_

#include <array>
#include <cmath>
#include <map>
#include <string>
#include <utility>

namespace override_sim
{

// goal_id -> (x, y, type, quadrant, height)
// type: 0 = tall neutral center goal, 1 = short neutral goal,
//       2 = red alliance goal, 3 = blue alliance goal
struct GoalInfo
{
  double x;
  double y;
  int type;
  std::string quadrant;
  double height;
};

inline const std::map<int, GoalInfo> kGoals = {
  {0, {0.0, 0.0, 0, "C", 0.2227}},
  {1, {-0.6, 1.2, 1, "N", 0.1465}},
  {2, {-1.2, 0.6, 1, "W", 0.1465}},
  {3, {-1.2, -0.6, 2, "W", 0.0825}},
  {4, {-0.6, -1.2, 2, "S", 0.0825}},
  {5, {0.6, 1.2, 3, "N", 0.0825}},
  {6, {1.2, 0.6, 3, "E", 0.0825}},
  {7, {1.2, -0.6, 1, "E", 0.1465}},
  {8, {0.6, -1.2, 1, "S", 0.1465}},
};

// loader_id -> (x, y)
inline const std::map<int, std::pair<double, double>> kLoaders = {
  {0, {-1.74, 1.49}},
  {1, {-1.74, -1.49}},
  {2, {1.74, 1.49}},
  {3, {1.74, -1.49}},
};

// toggle_id -> (x, y, base_yaw)
inline const std::map<int, std::array<double, 3>> kToggles = {
  {0, {0.0, 1.78, 0.0}},
  {1, {1.78, 0.0, M_PI / 2.0}},
  {2, {0.0, -1.78, 0.0}},
  {3, {-1.78, 0.0, M_PI / 2.0}},
};

inline const std::array<std::string, 4> kQuadrantByToggle = {"N", "E", "S", "W"};

inline int QuadrantToToggle(const std::string & quadrant)
{
  for (size_t i = 0; i < kQuadrantByToggle.size(); ++i) {
    if (kQuadrantByToggle[i] == quadrant) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

// ---- quaternion helpers (x, y, z, w convention) ----
// result = a * b (Hamilton product)
inline void QuatMul(
  double ax, double ay, double az, double aw,
  double bx, double by, double bz, double bw,
  double * cx, double * cy, double * cz, double * cw)
{
  *cx = aw * bx + ax * bw + ay * bz - az * by;
  *cy = aw * by - ax * bz + ay * bw + az * bx;
  *cz = aw * bz + ax * by - ay * bx + az * bw;
  *cw = aw * bw - ax * bx - ay * by - az * bz;
}

inline void QuatFromYaw(double yaw, double * x, double * y, double * z, double * w)
{
  *x = 0.0;
  *y = 0.0;
  *z = std::sin(yaw / 2.0);
  *w = std::cos(yaw / 2.0);
}

// yaw of a (mostly) yaw-only quaternion; matches scipy as_euler('xyz')[2]
// for quaternions with x = y = 0.
inline double YawFromQuat(double x, double y, double z, double w)
{
  double n = std::sqrt(x * x + y * y + z * z + w * w);
  if (n < 1e-9) {
    n = 1.0;
  }
  x /= n; y /= n; z /= n; w /= n;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace override_sim

#endif  // OVERRIDE_SIM_FIELD_CONSTANTS_HPP_
