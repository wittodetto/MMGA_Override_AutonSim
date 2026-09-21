// ai_driver.cpp - executes the Strategy AI output: drives to the requested
// pose via Nav2 and forwards intake / placement / toggle actions to the world
// services.
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "override_sim/msg/action_state.hpp"
#include "override_sim/srv/intake_element.hpp"
#include "override_sim/srv/score_element.hpp"

namespace override_sim
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class AIDriver : public rclcpp::Node
{
public:
  AIDriver() : Node("ai_driver")
  {
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/ai_goal_marker", 10);

    sai_sub_ = this->create_subscription<ActionState>(
      "/sai_output", 10,
      [this](const ActionState::SharedPtr msg) { interpret_action(msg); });
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        avg_vel_ = (msg->twist.linear.x + msg->twist.linear.y) / 2.0;
      });

    intake_client_ = this->create_client<IntakeElement>("/robot_intake");
    score_client_ = this->create_client<ScoreElement>("/score_element");

    marker_id_counter_ = 0;
  }

private:
  void publish_goal_marker(double x, double y, bool red)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.id = marker_id_counter_++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.1;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.lifetime = rclcpp::Duration::from_seconds(99999);
    if (red) {
      marker.color.r = 1.0;
    } else {
      marker.color.b = 1.0;
    }
    marker.color.a = 1.0;
    marker_pub_->publish(marker);
  }

  void drive_to(double x, double y, double theta)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "nav2 action server not available");
      return;
    }
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = std::sin(theta / 2.0);
    goal.pose.pose.orientation.w = std::cos(theta / 2.0);
    nav_client_->async_send_goal(goal);
  }

  void interpret_action(const ActionState::SharedPtr msg)
  {
    publish_goal_marker(msg->red_robot.x, msg->red_robot.y, true);
    publish_goal_marker(msg->blue_robot.x, msg->blue_robot.y, false);

    if (msg->red_robot_action == 0) {
      drive_to(msg->red_robot.x, msg->red_robot.y, msg->red_robot.theta);
    }
    // action codes 1..4 (intake / place pin / place cup / flip toggle) are
    // handled by the robot's own controller / future manipulator stack.
  }

  double avg_vel_ = 0.0;
  int marker_id_counter_ = 0;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<ActionState>::SharedPtr sai_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Client<IntakeElement>::SharedPtr intake_client_;
  rclcpp::Client<ScoreElement>::SharedPtr score_client_;
};

}  // namespace override_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<override_sim::AIDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
