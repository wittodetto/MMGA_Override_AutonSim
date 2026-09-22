// sim.h - zero-dependency Override match simulator core.
//
// Owns: the 20-element field state, robot + opponent poses, toggle states,
// match phase (auton 15 s / driver), the autonomous-routine controller and
// scoring. Everything is plain C++17; no ROS, no physics engine — a light
// kinematic robot model with waypoint following and map-based collision.
#ifndef NOROS_SIM_SIM_HPP_
#define NOROS_SIM_SIM_HPP_

#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "noros_sim/field.h"
#include "noros_sim/map_loader.h"
#include "noros_sim/scoring.h"

namespace noros_sim
{

// one scoring object: on the field (goal_id = -1) or stacked on a goal
struct Element
{
  int id = 0;
  int type = 1;        // 1 = pin, 2 = cup
  int top = 0;         // 0 none, 1 red, 2 blue, 3 yellow
  int bottom = 0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  int goal_id = -1;
  std::string name;
};

struct Robot
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::deque<std::array<int, 3>> intake;   // {type, top, bottom}
};

// an autonomous step: drive to (x, y), then run `action`
// 0 = nothing (continue), 1 = flip nearest toggle,
// 2 = place a pin on nearest goal, 3 = intake nearest field element
struct AutonStep
{
  double x = 0.0;
  double y = 0.0;
  int action = 0;
};

struct AutonRoutine
{
  std::string name;
  std::string description;
  double start_x = -1.6;
  double start_y = 0.5;
  double start_yaw = 0.0;
  std::vector<AutonStep> steps;
};

// built-in autonomous routines (red alliance, x < 0 starting side)
const std::vector<AutonRoutine> & builtin_autons();
const AutonRoutine * find_auton(const std::string & name);

class Sim
{
public:
  Sim(const MapData * map, bool with_elements = true, bool autonomous = true,
    bool with_opponent = false, const std::string & opponent_mode = "none");

  void reset(const AutonRoutine & auton, const std::string & opponent_mode);
  void step(double dt);

  // --- state ---
  double t = 0.0;                       // match time, seconds
  int phase = 0;                        // 0 = autonomous, 1 = driver control
  Robot robot;
  Robot opponent;
  std::vector<Element> elements;        // elements on the open field
  std::array<int, 4> toggles{};         // 0 = yellow, 1 = red, 2 = blue
  std::array<int, 5> elements_left{};   // ry, by, yy, rb, cups
  int auton_red_pins = 0;               // SC7 auton snapshot (pin points only)
  int auton_blue_pins = 0;
  int bonus_red = 0;
  int bonus_blue = 0;
  bool auton_active = false;

  // --- queries ---
  std::vector<GoalStack> build_goal_stacks() const;
  ScoreDetail current_score() const;
  int stack_count(int goal_id) const;

  // flags used by the renderer
  bool with_elements = true;
  bool autonomous = true;

private:
  const MapData * map_;
  std::vector<Element> goal_stacks_[9];       // per-goal stacked elements
  std::map<int, int> goal_count_;
  size_t auton_step_ = 0;
  bool step_done_ = false;
  bool with_opponent_ = false;
  std::string opponent_mode_ = "none";
  int next_element_id_ = 100;
  const AutonRoutine * active_routine_ = nullptr;

  void place_elements_start();
  void controller(double dt, Robot & r, const AutonRoutine & auton);
  void drive_toward(Robot & r, double tx, double ty, double dt, double max_v);
  void do_action(int action, Robot & r);
  void intake_nearest(Robot & r);
  void place_pin(Robot & r);
  void place_cup(Robot & r);
  void flip_nearest(Robot & r);
  bool blocked(double x, double y) const;
  void opponent_step(double dt);
};

}  // namespace noros_sim

#endif  // NOROS_SIM_SIM_HPP_
