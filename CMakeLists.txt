cmake_minimum_required(VERSION 3.8)
project(override_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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

install(
  DIRECTORY launch worlds models maps config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/field_location.py
  src/pose_bridge.py
  src/world_services.py
  src/scoring.py
  src/strategy_ai_bridge.py
  src/opponent.py
  src/ai_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
