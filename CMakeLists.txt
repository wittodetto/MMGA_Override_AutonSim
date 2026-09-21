cmake_minimum_required(VERSION 3.8)
project(override_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# custom interfaces for the V5RC Override game
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PinElement.msg"
  "msg/CupElement.msg"
  "msg/GoalState.msg"
  "msg/GoalArray.msg"
  "msg/ToggleState.msg"
  "msg/ToggleArray.msg"
  "msg/FieldElement.msg"
  "msg/FieldElementArray.msg"
  "msg/ActionState.msg"
  "msg/WorldState.msg"
  "srv/ScoreElement.srv"
  "srv/IntakeElement.srv"
  "srv/LoadElement.srv"
  "srv/FlipToggle.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# C++ game nodes
set(OVERRIDE_NODE_DEPS
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  sensor_msgs
  nav2_msgs
  visualization_msgs
  ros_gz_interfaces
  ament_index_cpp
)

function(override_sim_node target)
  add_executable(${target} src/${target}.cpp)
  target_include_directories(${target} PRIVATE include)
  ament_target_dependencies(${target} ${OVERRIDE_NODE_DEPS})
  rosidl_target_interfaces(${target} ${PROJECT_NAME} "rosidl_typesupport_cpp")
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

override_sim_node(pose_bridge)
override_sim_node(field_location)
override_sim_node(world_services)
override_sim_node(scoring)
override_sim_node(opponent)
override_sim_node(strategy_ai_bridge)
override_sim_node(ai_driver)

install(
  DIRECTORY launch worlds models maps config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
