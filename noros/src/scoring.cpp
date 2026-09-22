// scoring.cpp - pure Override scoring (port of src/scoring.cpp, SC1-SC7).
#include "noros_sim/scoring.h"

#include <algorithm>
#include <cmath>

#include "noros_sim/field.h"

namespace noros_sim
{

namespace
{

constexpr int kPinTerminalPoints = 5;
constexpr int kYellowPinPoints = 10;
constexpr int kRobotInMidfieldPoints = 8;
constexpr int kAutonBonus = 12;
constexpr int kAutonBonusTie = 6;

struct StackEntry
{
  double z;
  int kind;       // 0 = pin, 1 = cup
  Pin pin;
  Cup cup;
};

// 0 = none, 1 = bottom terminal hidden, 2 = top terminal hidden
int cup_covering_half(const Pin & pin, const std::vector<StackEntry> & stack)
{
  double best_d = 0.0;
  bool found = false;
  const Cup * covering = nullptr;
  for (const auto & e : stack) {
    if (e.kind != 1) {
      continue;
    }
    const double dz = e.z - pin.z;
    if (dz >= 0.10 && dz <= 0.28) {
      if (!found || dz < best_d) {
        found = true;
        best_d = dz;
        covering = &e.cup;
      }
    }
  }
  if (!found || covering == nullptr) {
    return 0;
  }
  return covering->yaw < M_PI ? 1 : 2;
}

// 1 (red), 2 (blue) or 0 (no one) for a yellow terminal on this goal
int yellow_owner(const GoalStack & gs, int red_mid, int blue_mid)
{
  if (gs.quadrant == "C") {
    if (red_mid > blue_mid) return 1;
    if (blue_mid > red_mid) return 2;
    return 0;
  }
  return gs.toggle_state;
}

}  // namespace

ScoreDetail compute_score(
  const std::vector<GoalStack> & goals,
  double robot_x, double robot_y,
  double opp_x, double opp_y,
  int bonus_red, int bonus_blue)
{
  ScoreDetail d;
  const int red_mid = InMidfield(robot_x, robot_y) ? 1 : 0;
  const int blue_mid = InMidfield(opp_x, opp_y) ? 1 : 0;

  for (const auto & gs : goals) {
    // merge pins and cups into one stack, sorted bottom-up
    std::vector<StackEntry> stack;
    for (const auto & pin : gs.pins) {
      stack.push_back({pin.z, 0, pin, Cup{}});
    }
    for (const auto & cup : gs.cups) {
      stack.push_back({cup.z, 1, Pin{}, cup});
    }
    std::sort(stack.begin(), stack.end(),
      [](const StackEntry & a, const StackEntry & b) { return a.z < b.z; });

    for (const auto & e : stack) {
      if (e.kind != 0) {
        continue;
      }
      const Pin & pin = e.pin;
      const int covered = cup_covering_half(pin, stack);
      const int terminals[2] = {
        covered == 2 ? 0 : pin.top,
        covered == 1 ? 0 : pin.bottom,
      };
      for (const int color : terminals) {
        if (color == 1) {
          d.red += kPinTerminalPoints;
          d.red_pins += kPinTerminalPoints;
        } else if (color == 2) {
          d.blue += kPinTerminalPoints;
          d.blue_pins += kPinTerminalPoints;
        } else if (color == 3) {
          const int owner = yellow_owner(gs, red_mid, blue_mid);
          if (owner == 1) {
            d.red += kYellowPinPoints;
            d.red_yellow += kYellowPinPoints;
          } else if (owner == 2) {
            d.blue += kYellowPinPoints;
            d.blue_yellow += kYellowPinPoints;
          }
        }
      }
    }
  }

  // robots in the midfield
  d.red_mid = red_mid * kRobotInMidfieldPoints;
  d.blue_mid = blue_mid * kRobotInMidfieldPoints;
  d.red += d.red_mid;
  d.blue += d.blue_mid;

  // settled autonomous bonus
  d.auton_bonus_red = bonus_red;
  d.auton_bonus_blue = bonus_blue;
  d.red += d.auton_bonus_red;
  d.blue += d.auton_bonus_blue;

  return d;
}

// helper used by the simulator to settle the auton bonus at phase switch
void settle_auton_bonus(int auton_red, int auton_blue, int & bonus_red, int & bonus_blue)
{
  if (auton_red > auton_blue) {
    bonus_red = kAutonBonus;
    bonus_blue = 0;
  } else if (auton_blue > auton_red) {
    bonus_red = 0;
    bonus_blue = kAutonBonus;
  } else {
    bonus_red = kAutonBonusTie;
    bonus_blue = kAutonBonusTie;
  }
}

}  // namespace noros_sim
