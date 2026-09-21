# Full world state packed for the Strategy AI node.
std_msgs/Header header
std_msgs/Int64MultiArray score
GoalArray goals
ToggleArray toggles
FieldElementArray field_elements
geometry_msgs/Pose2D robot_pose
geometry_msgs/Pose2D opponent_pose
std_msgs/Int64MultiArray robot_intake
std_msgs/Int64MultiArray loaders
std_msgs/Int64MultiArray elements_left
