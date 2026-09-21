// pose_bridge.cpp - republishes the Gazebo dynamic-pose stream (pins and cups)
// as an override_sim/FieldElementArray on /_object_locations.
//
// The Gazebo topic /world/<world>/dynamic_pose/info is bridged to ROS by
// ros_gz_bridge (see sim_backend.launch.py) as ros_gz_interfaces/msg/Pose_V,
// which keeps each entity's name and id.
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/pose_v.hpp>

#include "override_sim/msg/field_element.hpp"
#include "override_sim/msg/field_element_array.hpp"

namespace override_sim
{

class PoseBridge : public rclcpp::Node
{
public:
  PoseBridge() : Node("pose_bridge")
  {
    this->declare_parameter<std::string>("world_name", "override");
    this->get_parameter("world_name", world_name_);

    pub_ = this->create_publisher<FieldElementArray>("/_object_locations", 10);
    sub_ = this->create_subscription<ros_gz_interfaces::msg::Pose_V>(
      "/world/" + world_name_ + "/dynamic_pose/info", rclcpp::SensorDataQoS(),
      [this](const ros_gz_interfaces::msg::Pose_V::SharedPtr msg) { latest_ = msg; });
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), [this]() { update_locations(); });
  }

private:
  // color codes: 1 = red, 2 = blue, 3 = yellow
  static std::pair<int, int> ParsePinColors(const std::string & name)
  {
    // pin_ry_3 -> (bottom=1, top=3)
    const std::vector<std::pair<std::string, std::pair<int, int>>> combos = {
      {"ry", {1, 3}}, {"by", {2, 3}}, {"yy", {3, 3}}, {"rb", {1, 2}}};
    for (const auto & c : combos) {
      const std::string token = "pin_" + c.first + "_";
      if (name.find(token) != std::string::npos ||
        name.rfind("pin_" + c.first, 0) == 0)
      {
        return c.second;
      }
    }
    return {0, 0};
  }

  static bool IsNonElement(const std::string & name)
  {
    static const std::vector<std::string> tokens = {
      "link", "Otto", "opponent", "wheel", "field", "goal", "toggle",
      "loader", "ground"};
    for (const auto & t : tokens) {
      if (name.find(t) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  void update_locations()
  {
    auto objects = std::make_unique<FieldElementArray>();
    if (latest_) {
      for (const auto & p : latest_->data) {
        if (IsNonElement(p.name)) {
          continue;
        }
        FieldElement el;
        el.object_name = p.name;
        el.id = static_cast<int32_t>(p.id);
        el.location = p.position;
        el.orientation = p.orientation;
        if (p.name.find("cup") != std::string::npos) {
          el.element_type = 2;
        } else if (p.name.find("pin") != std::string::npos) {
          el.element_type = 1;
          const auto colors = ParsePinColors(p.name);
          el.bottom_color = static_cast<int8_t>(colors.first);
          el.top_color = static_cast<int8_t>(colors.second);
        } else {
          continue;
        }
        objects->elements.push_back(el);
      }
    }
    pub_->publish(std::move(objects));
  }

  std::string world_name_;
  rclcpp::Publisher<FieldElementArray>::SharedPtr pub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Pose_V>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  ros_gz_interfaces::msg::Pose_V::SharedPtr latest_;
};

}  // namespace override_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<override_sim::PoseBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
