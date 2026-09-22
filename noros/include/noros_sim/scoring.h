// scoring.h - pure Override scoring (VEX V5RC 2026-27, SC1-SC7), no ROS.
// Direct port of the ROS package's src/scoring.cpp scoring logic:
//   - each pin has two terminals (top/bottom color halves), 5 pts each
//     red/blue, 10 pts each yellow
//   - a terminal hidden by a cup's opaque half does not score
//     (simplified: cup yaw < pi hides bottom, >= pi hides top)
//   - yellow pins in a quadrant are owned by the alliance matching the
//     quadrant toggle; yellow pins on the center goal are owned by the
//     alliance with more robots in the Midfield (tie = nobody)
//   - robots in the Midfield (|x|+|y| <= 0.6 + robot radius) score 8 pts
//   - autonomous bonus: +12 to the alliance with more auton pin points,
//     +6 each on a tie (SC7: midfield points excluded from auton snapshot)
#ifndef NOROS_SIM_SCORING_HPP_
#define NOROS_SIM_SCORING_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace noros_sim
{

// element colors: 0 = none, 1 = red, 2 = blue, 3 = yellow
struct Pin
{
  double z = 0.0;
  int8_t top = 0;
  int8_t bottom = 0;
};

struct Cup
{
  double z = 0.0;
  double yaw = 0.0;
};

struct GoalStack
{
  int goal_id = 0;
  int type = 0;                 // 0 center, 1 neutral, 2 red, 3 blue
  std::string quadrant;
  int toggle_state = 0;         // 0 = yellow, 1 = red, 2 = blue (mod 3)
  std::vector<Pin> pins;        // sorted bottom-up by z
  std::vector<Cup> cups;
};

struct ScoreDetail
{
  int red = 0;
  int blue = 0;
  int red_pins = 0;      // red/blue terminal points from pins
  int blue_pins = 0;
  int red_yellow = 0;    // yellow-owned terminal points
  int blue_yellow = 0;
  int red_mid = 0;       // midfield robot points
  int blue_mid = 0;
  int auton_bonus_red = 0;
  int auton_bonus_blue = 0;
};

// Computes the current score. `robot`/`opponent` are the two robot poses
// (red alliance robot and blue alliance opponent). `bonus_red`/`bonus_blue`
// are the settled autonomous bonuses (0 during auton, +12/+6 after).
ScoreDetail compute_score(
  const std::vector<GoalStack> & goals,
  double robot_x, double robot_y,
  double opp_x, double opp_y,
  int bonus_red, int bonus_blue);

// SC7: settles the autonomous bonus at the auton -> driver switch.
// +12 to the alliance with more autonomous pin points, +6 each on a tie.
void settle_auton_bonus(
  int auton_red_pins, int auton_blue_pins,
  int & bonus_red, int & bonus_blue);

}  // namespace noros_sim

#endif  // NOROS_SIM_SCORING_HPP_
