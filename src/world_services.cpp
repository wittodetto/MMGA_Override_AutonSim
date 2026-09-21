// world_services.cpp - owns world-side game actions: robot intake, element
// placement on goals, match loading onto loaders, and Toggle flipping.
// Also publishes toggle state and the match phase.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/msg/entity_factory.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

#include "override_sim/field_constants.h"
#include "override_sim/msg/toggle_array.hpp"
#include "override_sim/msg/toggle_state.hpp"
#include "override_sim/srv/flip_toggle.hpp"
#include "override_sim/srv/intake_element.hpp"
#include "override_sim/srv/load_element.hpp"
#include "override_sim/srv/score_element.hpp"

namespace override_sim
{

using DeleteEntity = ros_gz_interfaces::srv::DeleteEntity;
using SpawnEntity = ros_gz_interfaces::srv::SpawnEntity;
using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;

class WorldServices : public rclcpp::Node
{
public:
  WorldServices() : Node("world_services")
  {
    this->declare_parameter<std::string>("world_name", "override");
    this->declare_parameter<int>("intake_capacity", 10);
    this->declare_parameter<bool>("autonomous", false);
    this->get_parameter("world_name", world_name_);
    this->get_parameter("intake_capacity", intake_capacity_);
    this->get_parameter("autonomous", autonomous_);

    pkg_path_ = ament_index_cpp::get_package_share_directory("override_sim");

    robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/otto_pose", 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (!msg->poses.empty()) {
          robot_x_ = msg->poses.back().position.x;
          robot_y_ = msg->poses.back().position.y;
        }
      });

    robot_elements_pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>(
      "/robot_elements", 10);
    elements_remaining_pub_ =
      this->create_publisher<std_msgs::msg::Int64MultiArray>(
      "/elements_remaining", 10);
    toggles_pub_ = this->create_publisher<ToggleArray>("/toggles", 10);
    phase_pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>(
      "/game_phase", 10);

    intake_srv_ = this->create_service<IntakeElement>("/robot_intake",
      [this](const std::shared_ptr<IntakeElement::Request> req,
        std::shared_ptr<IntakeElement::Response> res) { handle_intake(req, res); });
    score_srv_ = this->create_service<ScoreElement>("/score_element",
      [this](const std::shared_ptr<ScoreElement::Request> req,
        std::shared_ptr<ScoreElement::Response> res) { handle_score(req, res); });
    loader_srv_ = this->create_service<LoadElement>("/loader",
      [this](const std::shared_ptr<LoadElement::Request> req,
        std::shared_ptr<LoadElement::Response> res) { handle_load(req, res); });
    toggle_srv_ = this->create_service<FlipToggle>("/flip_toggle",
      [this](const std::shared_ptr<FlipToggle::Request> req,
        std::shared_ptr<FlipToggle::Response> res) { handle_flip(req, res); });

    remove_entity_ = this->create_client<DeleteEntity>(
      "/world/" + world_name_ + "/remove");
    spawn_entity_ = this->create_client<SpawnEntity>(
      "/world/" + world_name_ + "/create");
    set_pose_ = this->create_client<SetEntityPose>(
      "/world/" + world_name_ + "/set_pose");

    for (const auto & kv : kGoals) {
      goal_stack_count_[kv.first] = 0;
    }
    for (const auto & kv : kToggles) {
      toggle_states_[kv.first] = 0;
    }
    elements_left_ = {20, 20, 19, 4, 56};

    publish_elements_remaining();
    publish_toggles();

    phase_ = 1;
    if (autonomous_) {
      phase_ = 0;
      auton_timer_ = this->create_wall_timer(
        std::chrono::seconds(15), [this]() { end_autonomous(); });
    }
    phase_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), [this]() { publish_phase(); });
  }

private:
  // ------------------------------------------------------------------
  // phase
  // ------------------------------------------------------------------
  void end_autonomous()
  {
    if (phase_ == 0) {
      RCLCPP_INFO(this->get_logger(), "autonomous period ended");
      phase_ = 1;
    }
  }

  void publish_phase()
  {
    auto msg = std::make_unique<std_msgs::msg::Int64MultiArray>();
    msg->data.push_back(phase_);
    phase_pub_->publish(std::move(msg));
  }

  // ------------------------------------------------------------------
  // services
  // ------------------------------------------------------------------
  void handle_intake(
    const std::shared_ptr<IntakeElement::Request> req,
    std::shared_ptr<IntakeElement::Response> res)
  {
    if (static_cast<int>(robot_intake_.size()) >= intake_capacity_) {
      RCLCPP_INFO(this->get_logger(), "robot intake full");
      res->success = false;
      return;
    }
    delete_entity(req->entity_id);
    robot_intake_.push_back(
      {req->entity_type, req->top_color, req->bottom_color});
    RCLCPP_INFO(this->get_logger(), "robot intake: size %zu", robot_intake_.size());
    publish_robot_elements();
    res->success = true;
  }

  void handle_score(
    const std::shared_ptr<ScoreElement::Request> req,
    std::shared_ptr<ScoreElement::Response> res)
  {
    int8_t el_type = req->element_type;
    int8_t top = req->top_color;
    int8_t bottom = req->bottom_color;

    if (top == 0 && bottom == 0) {
      if (robot_intake_.empty()) {
        RCLCPP_INFO(this->get_logger(), "nothing to place");
        res->success = false;
        return;
      }
      auto e = robot_intake_.front();
      robot_intake_.pop_front();
      el_type = e[0];
      top = e[1];
      bottom = e[2];
    }

    const auto goal_it = kGoals.find(req->goal_id);
    if (goal_it == kGoals.end()) {
      // not at a goal: drop near the robot
      std::uniform_real_distribution<double> dist(-0.3, 0.3);
      const double x = robot_x_ + dist(rng_);
      const double y = robot_y_ + dist(rng_);
      spawn_element(el_type, top, bottom, x, y, 0.3);
      RCLCPP_INFO(this->get_logger(), "dropped element near robot");
    } else {
      const GoalInfo & g = goal_it->second;
      const double z = g.height +
        goal_stack_count_[req->goal_id] * 0.17 + 0.02;
      spawn_element(el_type, top, bottom, g.x, g.y, z);
      goal_stack_count_[req->goal_id] += 1;
      RCLCPP_INFO(this->get_logger(), "placed element type %d on goal %d",
        el_type, req->goal_id);
    }

    publish_robot_elements();
    res->success = true;
  }

  void handle_load(
    const std::shared_ptr<LoadElement::Request> req,
    std::shared_ptr<LoadElement::Response> res)
  {
    const auto loader_it = kLoaders.find(req->loader_id);
    if (loader_it == kLoaders.end()) {
      res->success = false;
      return;
    }
    const std::string model = element_model(
      req->element_type, req->top_color, req->bottom_color);
    if (model.empty() || !take_one_element(
        req->element_type, req->top_color, req->bottom_color))
    {
      RCLCPP_INFO(this->get_logger(), "no matching elements left");
      res->success = false;
      return;
    }
    spawn_element(req->element_type, req->top_color, req->bottom_color,
      loader_it->second.first, loader_it->second.second, 0.45);
    RCLCPP_INFO(this->get_logger(), "loaded element onto loader %d",
      req->loader_id);
    publish_elements_remaining();
    res->success = true;
  }

  void handle_flip(
    const std::shared_ptr<FlipToggle::Request> req,
    std::shared_ptr<FlipToggle::Response> res)
  {
    const auto toggle_it = kToggles.find(req->toggle_id);
    if (toggle_it == kToggles.end()) {
      res->success = false;
      return;
    }
    int new_state;
    if (req->state < 0) {
      new_state = (toggle_states_[req->toggle_id] + 1) % 3;
    } else {
      new_state = req->state % 3;
    }
    toggle_states_[req->toggle_id] = static_cast<int8_t>(new_state);
    rotate_toggle(req->toggle_id, new_state);
    publish_toggles();
    RCLCPP_INFO(this->get_logger(), "toggle %d -> state %d",
      req->toggle_id, new_state);
    res->success = true;
    res->new_state = static_cast<int8_t>(new_state);
  }

  // ------------------------------------------------------------------
  // gazebo helpers
  // ------------------------------------------------------------------
  static std::string element_model(int8_t el_type, int8_t top, int8_t bottom)
  {
    if (el_type == 1) {  // pin
      switch ((bottom << 8) | top) {
        case (1 << 8) | 3: return "pin-ry";
        case (2 << 8) | 3: return "pin-by";
        case (3 << 8) | 3: return "pin-yy";
        case (1 << 8) | 2: return "pin-rb";
        default: return "";
      }
    }
    if (el_type == 2) {  // cup
      return "cup";
    }
    return "";
  }

  void spawn_element(
    int8_t el_type, int8_t top, int8_t bottom,
    double x, double y, double z, double yaw = 0.0)
  {
    const std::string model = element_model(el_type, top, bottom);
    if (model.empty()) {
      RCLCPP_WARN(this->get_logger(), "unknown element, not spawning");
      return;
    }

    int counter = 100;
    const auto it = name_counters_.find(model);
    if (it != name_counters_.end()) {
      counter = it->second;
    }
    name_counters_[model] = counter + 1;
    std::string model_flat = model;
    std::replace(model_flat.begin(), model_flat.end(), '-', '_');
    const std::string name = model_flat + "_" + std::to_string(counter);

    auto factory = std::make_shared<ros_gz_interfaces::msg::EntityFactory>();
    factory->name = name;
    factory->allow_renaming = true;
    factory->sdf_filename = pkg_path_ + "/models/" + model + "/model.sdf";
    factory->pose.position.x = x;
    factory->pose.position.y = y;
    factory->pose.position.z = z;
    double qx, qy, qz, qw;
    QuatFromYaw(yaw, &qx, &qy, &qz, &qw);
    factory->pose.orientation.x = qx;
    factory->pose.orientation.y = qy;
    factory->pose.orientation.z = qz;
    factory->pose.orientation.w = qw;

    auto req = std::make_shared<SpawnEntity::Request>();
    req->entity_factory = *factory;
    if (!spawn_entity_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "spawn service not ready");
      return;
    }
    spawn_entity_->async_send_request(req);
  }

  void delete_entity(int32_t entity_id)
  {
    auto req = std::make_shared<DeleteEntity::Request>();
    req->entity.id = static_cast<uint64_t>(entity_id);
    if (!remove_entity_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "remove service not ready");
      return;
    }
    remove_entity_->async_send_request(req);
  }

  void rotate_toggle(int toggle_id, int state)
  {
    const auto & t = kToggles.at(toggle_id);
    const double roll = state * (2.0 * M_PI / 3.0);
    double qz_x, qz_y, qz_z, qz_w;
    QuatFromYaw(t[2], &qz_x, &qz_y, &qz_z, &qz_w);
    const double qx_x = std::sin(roll / 2.0);
    const double qx_w = std::cos(roll / 2.0);
    double qx0, qy0, qz0, qw0;
    QuatMul(qz_x, qz_y, qz_z, qz_w, qx_x, 0.0, 0.0, qx_w,
      &qx0, &qy0, &qz0, &qw0);

    auto req = std::make_shared<SetEntityPose::Request>();
    req->entity.name = "toggle_" + std::to_string(toggle_id + 1);
    req->pose.position.x = t[0];
    req->pose.position.y = t[1];
    req->pose.position.z = 0.0;
    req->pose.orientation.x = qx0;
    req->pose.orientation.y = qy0;
    req->pose.orientation.z = qz0;
    req->pose.orientation.w = qw0;
    if (!set_pose_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "set_pose service not ready");
      return;
    }
    set_pose_->async_send_request(req);
  }

  // ------------------------------------------------------------------
  // bookkeeping publishers
  // ------------------------------------------------------------------
  bool take_one_element(int8_t el_type, int8_t top, int8_t bottom)
  {
    int idx;
    if (el_type == 2) {
      idx = 4;
    } else {
      const std::string model = element_model(el_type, top, bottom);
      if (model == "pin-ry") idx = 0;
      else if (model == "pin-by") idx = 1;
      else if (model == "pin-yy") idx = 2;
      else if (model == "pin-rb") idx = 3;
      else return false;
    }
    if (elements_left_[idx] <= 0) {
      return false;
    }
    elements_left_[idx] -= 1;
    return true;
  }

  void publish_robot_elements()
  {
    auto msg = std::make_unique<std_msgs::msg::Int64MultiArray>();
    for (const auto & e : robot_intake_) {
      msg->data.push_back(e[0]);
      msg->data.push_back(e[1]);
      msg->data.push_back(e[2]);
    }
    robot_elements_pub_->publish(std::move(msg));
  }

  void publish_elements_remaining()
  {
    auto msg = std::make_unique<std_msgs::msg::Int64MultiArray>();
    for (int v : elements_left_) {
      msg->data.push_back(v);
    }
    elements_remaining_pub_->publish(std::move(msg));
  }

  void publish_toggles()
  {
    auto msg = std::make_unique<ToggleArray>();
    for (const auto & kv : kToggles) {
      ToggleState t;
      t.toggle_id = kv.first;
      t.state = toggle_states_.at(kv.first);
      t.quadrant = kQuadrantByToggle[kv.first];
      msg->toggles.push_back(t);
    }
    toggles_pub_->publish(std::move(msg));
  }

  // state
  std::string world_name_;
  int intake_capacity_ = 10;
  bool autonomous_ = false;
  std::string pkg_path_;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  std::deque<std::array<int8_t, 3>> robot_intake_;
  std::map<int, int> goal_stack_count_;
  std::map<int, int8_t> toggle_states_;
  std::array<int, 5> elements_left_;
  std::map<std::string, int> name_counters_;
  std::mt19937 rng_{std::random_device{}()};
  int phase_ = 1;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr robot_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr
    robot_elements_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr
    elements_remaining_pub_;
  rclcpp::Publisher<ToggleArray>::SharedPtr toggles_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr phase_pub_;
  rclcpp::Service<IntakeElement>::SharedPtr intake_srv_;
  rclcpp::Service<ScoreElement>::SharedPtr score_srv_;
  rclcpp::Service<LoadElement>::SharedPtr loader_srv_;
  rclcpp::Service<FlipToggle>::SharedPtr toggle_srv_;
  rclcpp::Client<DeleteEntity>::SharedPtr remove_entity_;
  rclcpp::Client<SpawnEntity>::SharedPtr spawn_entity_;
  rclcpp::Client<SetEntityPose>::SharedPtr set_pose_;
  rclcpp::TimerBase::SharedPtr auton_timer_;
  rclcpp::TimerBase::SharedPtr phase_timer_;
};

}  // namespace override_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<override_sim::WorldServices>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
