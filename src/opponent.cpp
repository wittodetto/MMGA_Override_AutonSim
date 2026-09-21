// opponent.cpp - a simple teleporting opponent robot that wanders the blue
// side of the Override field via the gz set_pose service.
#include <array>
#include <chrono>
#include <memory>
#include <random>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>

namespace override_sim
{

using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;

// blue-side waypoints (meters)
const std::vector<std::array<double, 2>> kMoveSet = {
  {0.50, 1.50}, {1.00, 1.10}, {1.55, 0.50}, {1.55, -0.50},
  {1.00, -1.10}, {0.50, -1.50}, {1.30, 0.00}, {0.90, 1.55},
  {0.90, -1.55},
};

const std::vector<std::vector<int>> kMoveGraph = {
  {1, 7}, {0, 2, 7}, {1, 3, 6}, {2, 4}, {3, 5, 8}, {4, 8},
  {2}, {0, 1}, {4, 5},
};

class Opponent : public rclcpp::Node
{
public:
  Opponent() : Node("opponent")
  {
    this->declare_parameter<std::string>("world_name", "override");
    this->get_parameter("world_name", world_name_);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/opponent/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        opp_x_ = msg->pose.position.x;
        opp_y_ = msg->pose.position.y;
      });
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2), [this]() { dumb_behavior(); });

    set_pose_ = this->create_client<SetEntityPose>(
      "/world/" + world_name_ + "/set_pose");
    height_ = 0.2;
  }

private:
  void dumb_behavior()
  {
    // wander the blue-side pose graph (70% chance each tick)
    if (!bernoulli_(rng_)) {
      return;
    }
    const auto & neighbors = kMoveGraph[current_pose_key_];
    std::uniform_int_distribution<size_t> pick(0, neighbors.size() - 1);
    const int next = neighbors[pick(rng_)];
    teleport_to_pose(kMoveSet[next]);
    current_pose_key_ = next;
  }

  void teleport_to_pose(const std::array<double, 2> & pose)
  {
    if (!set_pose_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "set_pose service not ready");
      return;
    }
    auto req = std::make_shared<SetEntityPose::Request>();
    req->entity.name = "opponent";
    req->pose.position.x = pose[0];
    req->pose.position.y = pose[1];
    req->pose.position.z = height_;
    req->pose.orientation.w = 1.0;
    set_pose_->async_send_request(req);
  }

  std::string world_name_;
  double opp_x_ = 0.0;
  double opp_y_ = 0.0;
  double height_ = 0.2;
  int current_pose_key_ = 0;
  std::mt19937 rng_{std::random_device{}()};
  std::bernoulli_distribution bernoulli_{0.7};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<SetEntityPose>::SharedPtr set_pose_;
};

}  // namespace override_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<override_sim::Opponent>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
