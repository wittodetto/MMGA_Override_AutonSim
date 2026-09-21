// strategy_ai_bridge.cpp - subscribes to every simulation topic and packs one
// override_sim/WorldState message for the Strategy AI node on /sai_input.
#include <chrono>
#include <memory>
#include <utility>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

#include "override_sim/field_constants.h"
#include "override_sim/msg/field_element_array.hpp"
#include "override_sim/msg/goal_array.hpp"
#include "override_sim/msg/toggle_array.hpp"
#include "override_sim/msg/world_state.hpp"

namespace override_sim
{

class StrategyAIBridge : public rclcpp::Node
{
public:
  StrategyAIBridge() : Node("sai_bridge")
  {
    score_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
      "/game_score", 10,
      [this](const std_msgs::msg::Int64MultiArray::SharedPtr msg) {
        world_state_.score = *msg;
      });
    goals_sub_ = this->create_subscription<GoalArray>(
      "/goals", 10,
      [this](const GoalArray::SharedPtr msg) { world_state_.goals = *msg; });
    toggles_sub_ = this->create_subscription<ToggleArray>(
      "/toggles", 10,
      [this](const ToggleArray::SharedPtr msg) { world_state_.toggles = *msg; });
    robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/otto_pose", 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        world_state_.robot_pose = pose2d_from_pose_array(msg);
      });
    intake_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
      "/robot_elements", 10,
      [this](const std_msgs::msg::Int64MultiArray::SharedPtr msg) {
        world_state_.robot_intake = *msg;
      });
    field_sub_ = this->create_subscription<FieldElementArray>(
      "/field_objects", 10,
      [this](const FieldElementArray::SharedPtr msg) {
        world_state_.field_elements = *msg;
      });
    loaders_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
      "/loaders", 10,
      [this](const std_msgs::msg::Int64MultiArray::SharedPtr msg) {
        world_state_.loaders = *msg;
      });
    elements_left_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
      "/elements_remaining", 10,
      [this](const std_msgs::msg::Int64MultiArray::SharedPtr msg) {
        world_state_.elements_left = *msg;
      });
    opponent_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/opponent/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        world_state_.opponent_pose.x = msg->pose.position.x;
        world_state_.opponent_pose.y = msg->pose.position.y;
        world_state_.opponent_pose.theta = YawFromQuat(
          msg->pose.orientation.x, msg->pose.orientation.y,
          msg->pose.orientation.z, msg->pose.orientation.w);
      });

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), [this]() { update_sai_world_state(); });
    sai_pub_ = this->create_publisher<WorldState>("/sai_input", 10);
  }

private:
  static geometry_msgs::msg::Pose2D pose2d_from_pose_array(
    const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    geometry_msgs::msg::Pose2D pose;
    if (msg->poses.empty()) {
      return pose;
    }
    const auto & p = msg->poses.back();
    pose.x = p.position.x;
    pose.y = p.position.y;
    pose.theta = YawFromQuat(
      p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    return pose;
  }

  void update_sai_world_state()
  {
    world_state_.header.stamp = this->now();
    world_state_.header.frame_id = "map";
    sai_pub_->publish(world_state_);
  }

  WorldState world_state_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<WorldState>::SharedPtr sai_pub_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr score_sub_;
  rclcpp::Subscription<GoalArray>::SharedPtr goals_sub_;
  rclcpp::Subscription<ToggleArray>::SharedPtr toggles_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr robot_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr intake_sub_;
  rclcpp::Subscription<FieldElementArray>::SharedPtr field_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr loaders_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr
    elements_left_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr opponent_sub_;
};

}  // namespace override_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<override_sim::StrategyAIBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
