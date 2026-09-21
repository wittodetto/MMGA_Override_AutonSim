// scoring.cpp - VEX V5RC Override scoring node.
//   - Each placed pin has two terminals (top / bottom color halves).
//   - A terminal is worth 5 pts if red/blue, 10 pts if yellow.
//   - A terminal is hidden if the cup stacked on that pin has its opaque half
//     facing it (simplified: cup yaw in [0, pi) hides the bottom terminal,
//     [pi, 2pi) hides the top terminal).
//   - Yellow pins in a quadrant are owned by the alliance whose color matches
//     the quadrant Toggle; yellow pins on the center (midfield) goal are owned
//     by the alliance with more robots in the Midfield (tie = no one).
//   - A robot with any part inside the Midfield scores 8 pts.
//   - Autonomous bonus: +12 to the alliance with more autonomous points
//     (pins only), +6 each on a tie.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

#include "override_sim/msg/cup_element.hpp"
#include "override_sim/msg/goal_array.hpp"
#include "override_sim/msg/goal_state.hpp"
#include "override_sim/msg/pin_element.hpp"

namespace override_sim
{

constexpr int kPinTerminalPoints = 5;
constexpr int kYellowPinPoints = 10;
constexpr int kRobotInMidfieldPoints = 8;
constexpr int kAutonBonus = 12;
constexpr int kAutonBonusTie = 6;

constexpr double kMidfieldHalf = 0.6;   // midfield diamond |x|+|y| <= 0.6 m
constexpr double kRobotRadius = 0.15;   // any part of the robot counts

// one entry of a goal's element stack, sorted bottom-up
struct StackEntry
{
  double z;
  int kind;          // 0 = pin, 1 = cup
  PinElement pin;
  CupElement cup;
};

class Scoring : public rclcpp::Node
{
public:
  Scoring() : Node("scoring")
  {
    goals_sub_ = this->create_subscription<GoalArray>(
      "/goals", 10,
      [this](const GoalArray::SharedPtr msg) { score_calculator(msg); });
    robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/otto_pose", 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (!msg->poses.empty()) {
          robot_x_ = msg->poses.back().position.x;
          robot_y_ = msg->poses.back().position.y;
        }
      });
    opponent_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/opponent/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        opp_x_ = msg->pose.position.x;
        opp_y_ = msg->pose.position.y;
      });
    phase_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
      "/game_phase", 10,
      [this](const std_msgs::msg::Int64MultiArray::SharedPtr msg) {
        phase_cb(msg);
      });

    score_pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>(
      "/game_score", 10);
    detail_pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>(
      "/score_detail", 10);
  }

private:
  void phase_cb(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
  {
    if (msg->data.empty()) {
      return;
    }
    const int64_t phase = msg->data[0];
    if (phase == 0) {
      if (!auton_started_) {
        auton_started_ = true;
        auton_active_ = true;
        auton_red_ = 0;
        auton_blue_ = 0;
        auton_bonus_red_ = 0;
        auton_bonus_blue_ = 0;
        RCLCPP_INFO(this->get_logger(), "autonomous period started");
      }
    } else if (phase == 1 && auton_active_) {
      auton_active_ = false;
      if (auton_red_ > auton_blue_) {
        auton_bonus_red_ = kAutonBonus;
        auton_bonus_blue_ = 0;
      } else if (auton_blue_ > auton_red_) {
        auton_bonus_red_ = 0;
        auton_bonus_blue_ = kAutonBonus;
      } else {
        auton_bonus_red_ = kAutonBonusTie;
        auton_bonus_blue_ = kAutonBonusTie;
      }
      RCLCPP_INFO(this->get_logger(),
        "autonomous bonus: red %d, blue %d", auton_bonus_red_,
        auton_bonus_blue_);
    }
  }

  static bool InMidfield(double x, double y)
  {
    return std::abs(x) + std::abs(y) <= kMidfieldHalf + kRobotRadius;
  }

  std::pair<int, int> midfield_counts() const
  {
    const int red = InMidfield(robot_x_, robot_y_) ? 1 : 0;
    const int blue = InMidfield(opp_x_, opp_y_) ? 1 : 0;
    return {red, blue};
  }

  // 1 (red), 2 (blue) or 0 (no one) for a yellow terminal on this goal
  int yellow_owner(const GoalState & gs, int red_mid, int blue_mid) const
  {
    if (gs.quadrant == "C") {
      if (red_mid > blue_mid) return 1;
      if (blue_mid > red_mid) return 2;
      return 0;
    }
    return gs.toggle_state;
  }

  // 0 = none, 1 = bottom terminal hidden, 2 = top terminal hidden
  static int cup_covering_half(const PinElement & pin,
    const std::vector<StackEntry> & stack)
  {
    double best_d = 0.0;
    bool found = false;
    const CupElement * covering = nullptr;
    for (const auto & e : stack) {
      if (e.kind != 1) {
        continue;
      }
      const double dz = e.z - pin.location.z;
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

  void score_calculator(const GoalArray::SharedPtr goals)
  {
    int red = 0;
    int blue = 0;
    int red_pins = 0;
    int blue_pins = 0;
    int red_yellow = 0;
    int blue_yellow = 0;

    const auto mids = midfield_counts();
    const int red_mid = mids.first;
    const int blue_mid = mids.second;

    for (const auto & gs : goals->goals) {
      // combine pins and cups into one stack, sorted bottom-up
      std::vector<StackEntry> stack;
      for (const auto & pin : gs.pins) {
        StackEntry e;
        e.z = pin.location.z;
        e.kind = 0;
        e.pin = pin;
        stack.push_back(e);
      }
      for (const auto & cup : gs.cups) {
        StackEntry e;
        e.z = cup.location.z;
        e.kind = 1;
        e.cup = cup;
        stack.push_back(e);
      }
      std::sort(stack.begin(), stack.end(),
        [](const StackEntry & a, const StackEntry & b) { return a.z < b.z; });

      for (const auto & e : stack) {
        if (e.kind != 0) {
          continue;
        }
        const PinElement & pin = e.pin;
        const int covered = cup_covering_half(pin, stack);
        const std::vector<int8_t> terminals = {
          covered == 2 ? 0 : pin.top_color,
          covered == 1 ? 0 : pin.bottom_color,
        };

        for (const int8_t color : terminals) {
          if (color == 1) {
            red += kPinTerminalPoints;
            red_pins += kPinTerminalPoints;
          } else if (color == 2) {
            blue += kPinTerminalPoints;
            blue_pins += kPinTerminalPoints;
          } else if (color == 3) {
            const int owner = yellow_owner(gs, red_mid, blue_mid);
            if (owner == 1) {
              red += kYellowPinPoints;
              red_yellow += kYellowPinPoints;
            } else if (owner == 2) {
              blue += kYellowPinPoints;
              blue_yellow += kYellowPinPoints;
            }
          }
        }
      }
    }

    // robots in the midfield
    red += red_mid * kRobotInMidfieldPoints;
    blue += blue_mid * kRobotInMidfieldPoints;

    // autonomous pin-score snapshot (midfield points excluded per SC7)
    if (auton_active_) {
      auton_red_ = red_pins + red_yellow;
      auton_blue_ = blue_pins + blue_yellow;
    }

    red += auton_bonus_red_;
    blue += auton_bonus_blue_;

    auto score_msg = std::make_unique<std_msgs::msg::Int64MultiArray>();
    score_msg->data.push_back(red);
    score_msg->data.push_back(blue);
    score_pub_->publish(std::move(score_msg));

    auto detail = std::make_unique<std_msgs::msg::Int64MultiArray>();
    detail->data.push_back(red_pins);
    detail->data.push_back(blue_pins);
    detail->data.push_back(red_yellow);
    detail->data.push_back(blue_yellow);
    detail->data.push_back(red_mid * kRobotInMidfieldPoints);
    detail->data.push_back(blue_mid * kRobotInMidfieldPoints);
    detail->data.push_back(auton_bonus_red_);
    detail->data.push_back(auton_bonus_blue_);
    detail_pub_->publish(std::move(detail));
  }

  // state
  bool auton_active_ = false;
  bool auton_started_ = false;
  int auton_bonus_red_ = 0;
  int auton_bonus_blue_ = 0;
  int auton_red_ = 0;
  int auton_blue_ = 0;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  double opp_x_ = 0.0;
  double opp_y_ = 0.0;

  rclcpp::Subscription<GoalArray>::SharedPtr goals_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr robot_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    opponent_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr phase_sub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr score_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr detail_pub_;
};

}  // namespace override_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<override_sim::Scoring>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
