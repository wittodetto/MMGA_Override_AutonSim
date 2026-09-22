// sim.cpp - zero-dependency Override match simulator core.
#include "noros_sim/sim.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace noros_sim
{

// ---------------------------------------------------------------------------
// built-in autonomous routines
// ---------------------------------------------------------------------------
const std::vector<AutonRoutine> & builtin_autons()
{
  static const std::vector<AutonRoutine> routines = {
    {
      "north_goal",
      "North quadrant: intake the north red/yellow pin, score it on the north "
      "neutral goal, flip the north toggle to red so the yellow half counts "
      "for red (+15 pts in auton).",
      -1.6, 0.5, 0.0,
      {
        {-1.60, 0.95, 0},   // climb north first (stay clear of goal 2)
        {-0.60, 1.55, 3},   // intake pin_ry_1 (north wall, omnidirectional)
        {-0.45, 1.32, 2},   // place pin beside neutral goal 1 (N)
        {0.00, 1.55, 1},    // flip north toggle (0) -> red
      },
    },
    {
      "south_route",
      "South quadrant: intake the south blue/yellow pin, score it on the "
      "south red alliance goal, flip the south toggle to red.",
      -1.6, -0.5, 0.0,
      {
        {-1.45, -1.00, 0},  // diagonal down (clear of goal 3)
        {-0.60, -1.55, 3},  // intake pin_by_4 (south wall, omnidirectional)
        {-0.45, -1.32, 2},  // place pin beside red alliance goal 4 (S)
        {0.00, -1.55, 1},   // flip south toggle (2) -> red
      },
    },
    {
      "midfield_push",
      "Park a robot inside the Midfield diamond for 8 pts and stay there.",
      -1.6, 0.0, 0.0,
      {
        {0.25, 0.25, 0},    // drive into the midfield diamond (clear of center goal)
      },
    },
    {
      "west_goal",
      "West quadrant: intake the west red/yellow pin, score it on the west "
      "neutral goal, flip the west toggle red.",
      -1.6, 0.5, 0.0,
      {
        {-1.55, 0.75, 3},   // intake pin_ry_2 (west wall, omnidirectional)
        {-1.00, 0.95, 2},   // place pin north of neutral goal 2 (W)
        {-1.30, 0.10, 1},   // flip west toggle (3) -> red
      },
    },
    {
      "center_goal",
      "Center: intake the center yellow pin, score it on the tall center goal, "
      "then park in the Midfield so red owns the yellow terminals.",
      -1.6, 0.0, 0.0,
      {
        {0.10, 0.65, 3},    // intake pin_yy_1 only (cup_3 is out of range)
        {0.30, 0.30, 2},    // place pin on center goal 0, robot sits in midfield
      },
    },
    {
      "cup_cover_test",
      "Coverage demo: score pin_ry_1 on the north neutral goal, flip the north "
      "toggle red (15 pts), then cover the pin with cup_1 whose opaque half "
      "hides the yellow bottom terminal (5 pts) — a data-driven coverage test.",
      -1.6, 0.5, 0.0,
      {
        {-1.60, 1.00, 0},   // climb north first (clear of goal 2)
        {-0.60, 1.55, 3},   // intake pin_ry_1 (north wall)
        {-0.45, 1.32, 2},   // place pin beside neutral goal 1 (N)
        {0.00, 1.55, 1},    // flip north toggle (0) -> red  (15 pts)
        {-1.10, 1.25, 3},   // intake cup_1 (north-west corner)
        {-0.60, 1.25, 4},   // place cup on goal 1 (yaw 0 hides yellow bottom)
      },
    },
  };
  return routines;
}

const AutonRoutine * find_auton(const std::string & name)
{
  for (const auto & r : builtin_autons()) {
    if (r.name == name) {
      return &r;
    }
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// construction / reset
// ---------------------------------------------------------------------------
Sim::Sim(const MapData * map, bool with_elements, bool autonomous,
  bool with_opponent, const std::string & opponent_mode)
: with_elements(with_elements), autonomous(autonomous), map_(map),
  with_opponent_(with_opponent), opponent_mode_(opponent_mode)
{
  reset(builtin_autons().front(), opponent_mode);
}

void Sim::reset(const AutonRoutine & auton, const std::string & opponent_mode)
{
  t = 0.0;
  phase = 0;
  auton_active = autonomous;
  toggles = {0, 0, 0, 0};
  elements_left = kElementsLeftStart;
  auton_red_pins = 0;
  auton_blue_pins = 0;
  bonus_red = 0;
  bonus_blue = 0;
  for (auto & s : goal_stacks_) s.clear();
  goal_count_.clear();
  elements.clear();
  auton_step_ = 0;
  step_done_ = false;
  opponent_mode_ = opponent_mode;

  robot.x = auton.start_x;
  robot.y = auton.start_y;
  robot.yaw = auton.start_yaw;
  robot.intake.clear();
  active_routine_ = &auton;

  opponent.x = 1.5;
  opponent.y = 0.3;
  opponent.yaw = M_PI;
  opponent.intake.clear();

  place_elements_start();
}

void Sim::place_elements_start()
{
  if (!with_elements) {
    return;
  }
  for (const auto & s : kStartElements) {
    Element e;
    e.id = next_element_id_++;
    e.type = s.type;
    e.top = s.top;
    e.bottom = s.bottom;
    e.x = s.x;
    e.y = s.y;
    e.z = 0.01;
    e.yaw = 0.0;
    e.goal_id = -1;
    e.name = s.name;
    elements.push_back(e);
  }
}

// ---------------------------------------------------------------------------
// collision
// ---------------------------------------------------------------------------
bool Sim::blocked(double x, double y) const
{
  if (map_ != nullptr && !map_->empty()) {
    // probe a few points around the robot disc
    constexpr int kProbes = 8;
    for (int i = 0; i < kProbes; ++i) {
      const double a = 2.0 * M_PI * i / kProbes;
      const double px = x + kRobotRadius * 0.8 * std::cos(a);
      const double py = y + kRobotRadius * 0.8 * std::sin(a);
      if (map_->occupied(px, py) == 1) {
        return true;
      }
    }
  }
  return !InsideField(x, y);
}

// ---------------------------------------------------------------------------
// low-level motion
// ---------------------------------------------------------------------------
void Sim::drive_toward(Robot & r, double tx, double ty, double dt, double max_v)
{
  const double dx = tx - r.x;
  const double dy = ty - r.y;
  const double dist = std::hypot(dx, dy);
  if (dist < 1e-3) {
    return;
  }
  const double target_yaw = std::atan2(dy, dx);
  double d_yaw = target_yaw - r.yaw;
  while (d_yaw > M_PI) d_yaw -= 2.0 * M_PI;
  while (d_yaw < -M_PI) d_yaw += 2.0 * M_PI;

  constexpr double kMaxOmega = 3.0;   // rad/s
  const double omega = std::max(-kMaxOmega, std::min(kMaxOmega, 3.0 * d_yaw));
  r.yaw += omega * dt;

  double v = max_v;
  if (std::abs(d_yaw) > 0.35) {
    v = 0.0;   // turn in place first
  } else {
    v = std::min(max_v, 2.0 * dist);
  }
  const double nx = r.x + v * std::cos(r.yaw) * dt;
  const double ny = r.y + v * std::sin(r.yaw) * dt;
  if (!blocked(nx, ny)) {
    r.x = nx;
    r.y = ny;
    return;
  }
  // obstacle on the heading: slide along the tangent until clear
  const double slide = 0.04;
  for (const double sign : {1.0, -1.0}) {
    const double tx = r.x + sign * slide * std::cos(r.yaw + M_PI / 2.0);
    const double ty = r.y + sign * slide * std::sin(r.yaw + M_PI / 2.0);
    if (!blocked(tx, ty)) {
      r.x = tx;
      r.y = ty;
      return;
    }
  }
}

// ---------------------------------------------------------------------------
// actions
// ---------------------------------------------------------------------------
void Sim::intake_nearest(Robot & r)
{
  if (static_cast<int>(r.intake.size()) >= kIntakeCapacity) {
    return;
  }
  // omnidirectional intake zone: nearest field element within reach
  int best = -1;
  double best_d = 0.35;
  for (size_t i = 0; i < elements.size(); ++i) {
    const Element & e = elements[i];
    if (e.z > 0.12) {
      continue;
    }
    const double d = std::hypot(e.x - r.x, e.y - r.y);
    if (d < best_d) {
      best_d = d;
      best = static_cast<int>(i);
    }
  }
  if (best == -1) {
    return;
  }
  const Element e = elements[best];
  elements.erase(elements.begin() + best);
  r.intake.push_back({e.type, e.top, e.bottom});
  std::printf("[auton] %s intaked %s (type %d)\n", auton_active ? "red" : "?",
    e.name.c_str(), e.type);
}

void Sim::place_pin(Robot & r)
{
  if (r.intake.empty()) {
    return;
  }
  const int goal_id = NearestGoal(r.x, r.y, 0.9);
  if (goal_id == -1) {
    return;
  }
  const auto it = r.intake.begin();
  if ((*it)[0] != 1) {
    return;   // only pins are placed by this action
  }
  const std::array<int, 3> el = *it;
  r.intake.pop_front();

  const GoalInfo & g = kGoals.at(goal_id);
  const double z = g.height + goal_count_[goal_id] * kStackStepZ + 0.02;
  Element e;
  e.id = next_element_id_++;
  e.type = el[0];
  e.top = el[1];
  e.bottom = el[2];
  e.x = g.x;
  e.y = g.y;
  e.z = z;
  e.yaw = 0.0;
  e.goal_id = goal_id;
  e.name = "placed_pin_" + std::to_string(e.id);
  goal_stacks_[goal_id].push_back(e);
  goal_count_[goal_id] += 1;
  std::printf("[auton] placed pin (top %d / bottom %d) on goal %d (%s)\n",
    el[1], el[2], goal_id, g.quadrant.c_str());
}

void Sim::place_cup(Robot & r)
{
  if (r.intake.empty()) {
    return;
  }
  const int goal_id = NearestGoal(r.x, r.y, 0.9);
  if (goal_id == -1) {
    return;
  }
  const auto it = r.intake.begin();
  if ((*it)[0] != 2) {
    return;   // only cups are placed by this action
  }
  const std::array<int, 3> el = *it;
  r.intake.pop_front();

  const GoalInfo & g = kGoals.at(goal_id);
  const double z = g.height + goal_count_[goal_id] * kStackStepZ + 0.02;
  Element e;
  e.id = next_element_id_++;
  e.type = el[0];
  e.top = 0;
  e.bottom = 0;
  e.x = g.x;
  e.y = g.y;
  e.z = z;
  e.yaw = 0.0;   // opaque half faces the bottom terminal (hides it)
  e.goal_id = goal_id;
  e.name = "placed_cup_" + std::to_string(e.id);
  goal_stacks_[goal_id].push_back(e);
  goal_count_[goal_id] += 1;
  std::printf("[auton] placed cup on goal %d (%s), yaw 0 -> covers bottom half\n",
    goal_id, g.quadrant.c_str());
}

void Sim::flip_nearest(Robot & r)
{
  const int tid = NearestToggle(r.x, r.y, 1.2);
  if (tid == -1) {
    return;
  }
  toggles[tid] = (toggles[tid] + 1) % 3;
  std::printf("[auton] flipped toggle %d (%s) -> state %d\n",
    tid, kQuadrantByToggle[tid].c_str(), toggles[tid]);
}

void Sim::do_action(int action, Robot & r)
{
  switch (action) {
    case 1: flip_nearest(r); break;
    case 2: place_pin(r); break;
    case 3: intake_nearest(r); break;
    case 4: place_cup(r); break;
    default: break;
  }
}

// ---------------------------------------------------------------------------
// controllers
// ---------------------------------------------------------------------------
void Sim::controller(double dt, Robot & r, const AutonRoutine & auton)
{
  if (auton.steps.empty()) {
    return;
  }
  if (auton_step_ >= auton.steps.size()) {
    return;
  }
  const AutonStep & st = auton.steps[auton_step_];
  const double dist = std::hypot(st.x - r.x, st.y - r.y);
  if (dist < 0.15) {
    do_action(st.action, r);
    auton_step_++;
  } else {
    drive_toward(r, st.x, st.y, dt, 0.85);
  }
}

void Sim::opponent_step(double dt)
{
  if (!with_opponent_) {
    return;
  }
  if (opponent_mode_ == "midfield") {
    drive_toward(opponent, 0.3, 0.3, dt, 0.4);
  }
  // "none" / "park": blue opponent stays at its start
}

// ---------------------------------------------------------------------------
// main step
// ---------------------------------------------------------------------------
void Sim::step(double dt)
{
  t += dt;

  // phase transitions: auton 0-15 s, driver 15-105 s
  if (autonomous && phase == 0 && t >= 15.0) {
    phase = 1;
    auton_active = false;
    settle_auton_bonus(auton_red_pins, auton_blue_pins, bonus_red, bonus_blue);
    std::printf("[sim] autonomous ended: red %d, blue %d -> bonus red %d, blue %d\n",
      auton_red_pins, auton_blue_pins, bonus_red, bonus_blue);
  }
  if (phase == 1 && t >= 105.0) {
    // match over; controller will idle
  }

  if (auton_active && active_routine_ != nullptr) {
    // red robot follows its auton routine during the auton period
    controller(dt, robot, *active_routine_);
  }
  opponent_step(dt);

  // score snapshot during auton (SC7: pin points only)
  if (auton_active) {
    const ScoreDetail d = current_score();
    auton_red_pins = d.red_pins + d.red_yellow;
    auton_blue_pins = d.blue_pins + d.blue_yellow;
  }
}

// ---------------------------------------------------------------------------
// queries
// ---------------------------------------------------------------------------
std::vector<GoalStack> Sim::build_goal_stacks() const
{
  std::vector<GoalStack> out;
  for (const auto & kv : kGoals) {
    const int gid = kv.first;
    GoalStack gs;
    gs.goal_id = gid;
    gs.type = kv.second.type;
    gs.quadrant = kv.second.quadrant;
    const int tid = QuadrantToToggle(gs.quadrant);
    gs.toggle_state = tid >= 0 ? toggles[tid] : 0;
    for (const auto & e : goal_stacks_[gid]) {
      if (e.type == 1) {
        Pin p;
        p.z = e.z;
        p.top = static_cast<int8_t>(e.top);
        p.bottom = static_cast<int8_t>(e.bottom);
        gs.pins.push_back(p);
      } else {
        Cup c;
        c.z = e.z;
        c.yaw = e.yaw;
        gs.cups.push_back(c);
      }
    }
    std::sort(gs.pins.begin(), gs.pins.end(),
      [](const Pin & a, const Pin & b) { return a.z < b.z; });
    std::sort(gs.cups.begin(), gs.cups.end(),
      [](const Cup & a, const Cup & b) { return a.z < b.z; });
    out.push_back(gs);
  }
  return out;
}

ScoreDetail Sim::current_score() const
{
  return compute_score(build_goal_stacks(),
    robot.x, robot.y, opponent.x, opponent.y, bonus_red, bonus_blue);
}

int Sim::stack_count(int goal_id) const
{
  const auto it = goal_count_.find(goal_id);
  return it == goal_count_.end() ? 0 : it->second;
}

}  // namespace noros_sim
