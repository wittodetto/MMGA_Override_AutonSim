// field.h - Zero-dependency Override (VEX V5RC 2026-27) field model.
//
// This header is the noros (no-ROS) mirror of the ROS package's
// include/override_sim/field_constants.h: same goal / loader / toggle data,
// same quadrant convention, plus field-geometry helpers used by the
// simulator (walls, midfield diamond, quadrant classification, collision).
//
// All units are meters; field origin is the center of the 12' x 12' field.
#ifndef NOROS_SIM_FIELD_HPP_
#define NOROS_SIM_FIELD_HPP_

#include <array>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace noros_sim
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

// loader_id -> (x, y) ; loaders sit at the corners next to alliance stations
inline const std::map<int, std::pair<double, double>> kLoaders = {
  {0, {-1.74, 1.49}},
  {1, {-1.74, -1.49}},
  {2, {1.74, 1.49}},
  {3, {1.74, -1.49}},
};

// toggle_id -> (x, y, base_yaw) ; one wall toggle per quadrant
inline const std::map<int, std::array<double, 3>> kToggles = {
  {0, {0.0, 1.78, 0.0}},
  {1, {1.78, 0.0, M_PI / 2.0}},
  {2, {0.0, -1.78, 0.0}},
  {3, {-1.78, 0.0, M_PI / 2.0}},
};

inline const std::array<std::string, 4> kQuadrantByToggle = {"N", "E", "S", "W"};

// elements remaining at match start: pin-ry, pin-by, pin-yy, pin-rb, cup
inline const std::array<int, 5> kElementsLeftStart = {20, 20, 19, 4, 56};

inline int QuadrantToToggle(const std::string & quadrant)
{
  for (size_t i = 0; i < kQuadrantByToggle.size(); ++i) {
    if (kQuadrantByToggle[i] == quadrant) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

// ---- field geometry ----

// inner faces of the perimeter walls
constexpr double kFieldHalf = 1.8288;              // 6 ft in meters
constexpr double kRobotRadius = 0.15;
constexpr double kMidfieldHalf = 0.6;              // |x|+|y| <= 0.6 m
constexpr double kGoalXyTol = 0.13;                // element-on-goal xy tol
constexpr double kGoalEngageTol = 0.02;
constexpr double kGoalMaxHeight = 0.35;
constexpr double kStackStepZ = 0.17;               // per-element z step
constexpr double kIntakeRange = 0.12;              // robot intake zone radius
constexpr int kIntakeCapacity = 10;

// quadrant of a point on the field:
// N (top)  y > |x| ; S (bottom) -y > |x| ; E (right) x > |y| ; W (left) -x > |y|
inline std::string QuadrantOf(double x, double y)
{
  if (std::abs(x) + std::abs(y) < 1e-6) {
    return "C";
  }
  if (y > std::abs(x)) return "N";
  if (-y > std::abs(x)) return "S";
  if (x > std::abs(y)) return "E";
  return "W";
}

inline bool InMidfield(double x, double y)
{
  return std::abs(x) + std::abs(y) <= kMidfieldHalf + kRobotRadius;
}

inline bool InsideField(double x, double y)
{
  return std::abs(x) <= kFieldHalf && std::abs(y) <= kFieldHalf;
}

// nearest goal id within max_dist; -1 if none
inline int NearestGoal(double x, double y, double max_dist)
{
  int best = -1;
  double best_d = max_dist;
  for (const auto & kv : kGoals) {
    const double d = std::hypot(x - kv.second.x, y - kv.second.y);
    if (d < best_d) {
      best = kv.first;
      best_d = d;
    }
  }
  return best;
}

inline int NearestToggle(double x, double y, double max_dist)
{
  int best = -1;
  double best_d = max_dist;
  for (const auto & kv : kToggles) {
    const double d = std::hypot(x - kv.second[0], y - kv.second[1]);
    if (d < best_d) {
      best = kv.first;
      best_d = d;
    }
  }
  return best;
}

// ---- official 20-element starting layout (VEXcode VR, 12 pins + 8 cups) ----
// {model, top_color, bottom_color, x, y} ; colors: 1=red, 2=blue, 3=yellow
struct StartElement
{
  const char * name;
  int type;          // 1 = pin, 2 = cup
  int top;
  int bottom;
  double x;
  double y;
};

inline const std::vector<StartElement> kStartElements = {
  {"pin_ry_1", 1, 3, 1, -0.60, 1.745},
  {"pin_by_1", 1, 3, 2, 0.60, 1.745},
  {"cup_1",    2, 0, 0, -1.20, 1.20},
  {"cup_2",    2, 0, 0, 1.20, 1.20},
  {"pin_ry_2", 1, 3, 1, -1.745, 0.60},
  {"cup_3",    2, 0, 0, -0.60, 0.60},
  {"pin_yy_1", 1, 3, 3, 0.00, 0.60},
  {"cup_4",    2, 0, 0, 0.60, 0.60},
  {"pin_by_2", 1, 3, 2, 1.745, 0.60},
  {"pin_rb_1", 1, 2, 1, -0.60, 0.00},
  {"pin_rb_2", 1, 2, 1, 0.60, 0.00},
  {"pin_by_3", 1, 3, 2, -1.745, -0.60},
  {"cup_5",    2, 0, 0, -0.60, -0.60},
  {"pin_yy_2", 1, 3, 3, 0.00, -0.60},
  {"cup_6",    2, 0, 0, 0.60, -0.60},
  {"pin_ry_3", 1, 3, 1, 1.745, -0.60},
  {"cup_7",    2, 0, 0, -1.20, -1.20},
  {"cup_8",    2, 0, 0, 1.20, -1.20},
  {"pin_by_4", 1, 3, 2, -0.60, -1.745},
  {"pin_ry_4", 1, 3, 1, 0.60, -1.745},
};

}  // namespace noros_sim

#endif  // NOROS_SIM_FIELD_HPP_
