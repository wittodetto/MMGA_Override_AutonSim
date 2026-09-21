// field_location.cpp - classifies every scoring object into goals / loaders /
// the open field and publishes the GoalArray consumed by the scorer. Also
// turns controller buttons into intake / placement / toggle-flip calls.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

#include "override_sim/field_constants.h"
#include "override_sim/msg/cup_element.hpp"
#include "override_sim/msg/field_element.hpp"
#include "override_sim/msg/field_element_array.hpp"
#include "override_sim/msg/goal_array.hpp"
#include "override_sim/msg/goal_state.hpp"
#include "override_sim/msg/pin_element.hpp"
#include "override_sim/msg/toggle_array.hpp"
#include "override_sim/srv/flip_toggle.hpp"
#include "override_sim/srv/intake_element.hpp"
#include "override_sim/srv/score_element.hpp"

namespace override_sim
{

constexpr double kGoalXyTol = 0.13;       // horizontal distance that counts
constexpr double kLoaderXyTol = 0.14;
constexpr double kGoalEngageTol = 0.02;   // base may sink this much below rim
constexpr double kGoalMaxHeight = 0.35;   // base may be this high above rim

class FieldLocation : public rclcpp::Node
{
public:
  FieldLocation() : Node("field_location")
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) { controller_cb(msg); });
    elements_sub_ = this->create_subscription<FieldElementArray>(
      "/_object_locations", 10,
      [this](const FieldElementArray::SharedPtr msg) { object_location_cb(msg); });
    robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/otto_pose", 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) { robot_pose_cb(msg); });
    toggles_sub_ = this->create_subscription<ToggleArray>(
      "/toggles", 10,
      [this](const ToggleArray::SharedPtr msg) { toggle_cb(msg); });

    goals_pub_ = this->create_publisher<GoalArray>("/goals", 10);
    loaders_pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>(
      "/loaders", 10);
    field_objects_pub_ = this->create_publisher<FieldElementArray>(
      "/field_objects", 10);

    intake_client_ = this->create_client<IntakeElement>("/robot_intake");
    score_client_ = this->create_client<ScoreElement>("/score_element");
    flip_client_ = this->create_client<FlipToggle>("/flip_toggle");

    prev_buttons_ = {0, 0, 0, 0};
    toggle_states_ = {{0, 0}, {1, 0}, {2, 0}, {3, 0}};
  }

private:
  // ------------------------------------------------------------------
  // callbacks
  // ------------------------------------------------------------------
  void controller_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::vector<int32_t> buttons;
    for (size_t i = 0; i < std::min<size_t>(msg->buttons.size(), 4); ++i) {
      buttons.push_back(msg->buttons[i]);
    }
    while (buttons.size() < 4) {
      buttons.push_back(0);
    }
    if (buttons[0] == 1 && prev_buttons_[0] == 0) {
      check_collision();                       // intake
    } else if (buttons[1] == 1 && prev_buttons_[1] == 0) {
      scoring_cb(1);                           // place pin
    } else if (buttons[2] == 1 && prev_buttons_[2] == 0) {
      scoring_cb(2);                           // place cup
    } else if (buttons[3] == 1 && prev_buttons_[3] == 0) {
      flip_nearest_toggle();                   // flip toggle
    }
    prev_buttons_ = buttons;
  }

  void robot_pose_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      return;
    }
    const auto & pose = msg->poses.back();
    robot_x_ = pose.position.x;
    robot_y_ = pose.position.y;
    robot_r_ = YawFromQuat(
      pose.orientation.x, pose.orientation.y, pose.orientation.z,
      pose.orientation.w);
  }

  void toggle_cb(const ToggleArray::SharedPtr msg)
  {
    for (const auto & t : msg->toggles) {
      toggle_states_[t.toggle_id] = t.state;
    }
  }

  void object_location_cb(const FieldElementArray::SharedPtr msg)
  {
    elements_ = msg;

    std::map<int, std::vector<FieldElement>> goal_stacks;
    std::map<int, std::vector<FieldElement>> loader_contents;
    for (const auto & kv : kGoals) {
      goal_stacks[kv.first] = {};
    }
    for (const auto & kv : kLoaders) {
      loader_contents[kv.first] = {};
    }
    auto field_elems = std::make_unique<FieldElementArray>();

    for (const auto & el : msg->elements) {
      const double base_z = el.location.z;
      const double x = el.location.x;
      const double y = el.location.y;
      bool placed = false;

      for (const auto & kv : kGoals) {
        const int gid = kv.first;
        const GoalInfo & g = kv.second;
        if (std::hypot(x - g.x, y - g.y) < kGoalXyTol &&
          g.height - kGoalEngageTol <= base_z &&
          base_z <= g.height + kGoalMaxHeight)
        {
          goal_stacks[gid].push_back(el);
          placed = true;
          break;
        }
      }
      if (placed) {
        continue;
      }

      for (const auto & kv : kLoaders) {
        const int lid = kv.first;
        const auto & p = kv.second;
        if (std::abs(x - p.first) < kLoaderXyTol &&
          std::abs(y - p.second) < kLoaderXyTol && base_z < 0.4)
        {
          loader_contents[lid].push_back(el);
          placed = true;
          break;
        }
      }

      if (!placed) {
        field_elems->elements.push_back(el);
      }
    }

    // build the GoalArray (kGoals is ordered, so ids come out sorted)
    auto goal_array = std::make_unique<GoalArray>();
    for (const auto & kv : kGoals) {
      const int gid = kv.first;
      const GoalInfo & g = kv.second;
      std::vector<FieldElement> stack = goal_stacks[gid];
      std::sort(stack.begin(), stack.end(),
        [](const FieldElement & a, const FieldElement & b) {
          return a.location.z < b.location.z;
        });

      GoalState gs;
      gs.goal_id = gid;
      gs.goal_type = static_cast<int8_t>(g.type);
      gs.quadrant = g.quadrant;
      gs.toggle_state = static_cast<int8_t>(
        toggle_states_.at(QuadrantToToggle(g.quadrant)));

      for (const auto & el : stack) {
        if (el.element_type == 1) {
          PinElement pin;
          pin.object_name = el.object_name;
          pin.id = el.id;
          pin.top_color = el.top_color;
          pin.bottom_color = el.bottom_color;
          pin.location = el.location;
          pin.orientation = el.orientation;
          gs.pins.push_back(pin);
        } else {
          CupElement cup;
          cup.object_name = el.object_name;
          cup.id = el.id;
          cup.location = el.location;
          cup.orientation = el.orientation;
          cup.yaw = YawFromQuat(
            el.orientation.x, el.orientation.y, el.orientation.z,
            el.orientation.w);
          gs.cups.push_back(cup);
        }
      }
      goal_array->goals.push_back(gs);
    }

    goals_pub_->publish(std::move(goal_array));

    auto loader_msg = std::make_unique<std_msgs::msg::Int64MultiArray>();
    for (const auto & kv : kLoaders) {
      loader_msg->data.push_back(
        static_cast<int64_t>(loader_contents[kv.first].size()));
    }
    loaders_pub_->publish(std::move(loader_msg));

    field_objects_pub_->publish(std::move(field_elems));
  }

  // ------------------------------------------------------------------
  // helpers
  // ------------------------------------------------------------------
  void check_collision()
  {
    // is there an element in the robot's intake zone?
    const double h = robot_x_;
    const double k = robot_y_;
    const double th = robot_r_;
    constexpr double offset = 0.15;
    const double ref_x = h + offset * std::cos(th);
    const double ref_y = k + offset * std::sin(th);

    for (const auto & el : elements_->elements) {
      const double dx = el.location.x - ref_x;
      const double dy = el.location.y - ref_y;
      const double dx_r = std::cos(th) * dx + std::sin(th) * dy;
      const double dy_r = -std::sin(th) * dx + std::cos(th) * dy;
      if (std::abs(dx_r) < 0.10 && std::abs(dy_r) < 0.10 &&
        el.location.z < 0.12)
      {
        auto req = std::make_shared<IntakeElement::Request>();
        req->entity_id = el.id;
        req->entity_type = el.element_type;
        req->top_color = el.top_color;
        req->bottom_color = el.bottom_color;
        intake_client_->async_send_request(req);
        RCLCPP_INFO(this->get_logger(),
          "intaking %s (type %d)", el.object_name.c_str(), el.element_type);
        return;
      }
    }
    RCLCPP_INFO(this->get_logger(), "no element in intake zone");
  }

  int check_location()
  {
    int best = 0;
    double best_d = 0.7;
    for (const auto & kv : kGoals) {
      const double d = std::hypot(robot_x_ - kv.second.x, robot_y_ - kv.second.y);
      if (d < best_d) {
        best = kv.first;
        best_d = d;
      }
    }
    return best;
  }

  void scoring_cb(int8_t element_type)
  {
    const int goal_id = check_location();
    RCLCPP_INFO(this->get_logger(), "placing element type %d at goal %d",
      element_type, goal_id);
    auto req = std::make_shared<ScoreElement::Request>();
    req->element_type = element_type;
    req->top_color = 0;
    req->bottom_color = 0;
    req->goal_id = goal_id;
    score_client_->async_send_request(req);
  }

  void flip_nearest_toggle()
  {
    int best = -1;
    double best_d = 1.0;
    for (const auto & kv : kToggles) {
      const auto & t = kv.second;
      const double d = std::hypot(robot_x_ - t[0], robot_y_ - t[1]);
      if (d < best_d) {
        best = kv.first;
        best_d = d;
      }
    }
    if (best == -1) {
      RCLCPP_INFO(this->get_logger(), "no toggle within range");
      return;
    }
    auto req = std::make_shared<FlipToggle::Request>();
    req->toggle_id = best;
    req->state = -1;
    flip_client_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "flipping toggle %d", best);
  }

  // state
  FieldElementArray::SharedPtr elements_;
  std::vector<int32_t> prev_buttons_;
  std::map<int, int8_t> toggle_states_;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  double robot_r_ = 0.0;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<FieldElementArray>::SharedPtr elements_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr robot_pose_sub_;
  rclcpp::Subscription<ToggleArray>::SharedPtr toggles_sub_;
  rclcpp::Publisher<GoalArray>::SharedPtr goals_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr loaders_pub_;
  rclcpp::Publisher<FieldElementArray>::SharedPtr field_objects_pub_;
  rclcpp::Client<IntakeElement>::SharedPtr intake_client_;
  rclcpp::Client<ScoreElement>::SharedPtr score_client_;
  rclcpp::Client<FlipToggle>::SharedPtr flip_client_;
};

}  // namespace override_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<override_sim::FieldLocation>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
