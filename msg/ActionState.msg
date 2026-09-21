# Strategy AI output: target poses + requested actions for both robots.
# action codes: 0 = drive to pose, 1 = intake, 2 = place pin, 3 = place cup,
#               4 = flip toggle
geometry_msgs/Pose2D red_robot
geometry_msgs/Pose2D blue_robot
int8 red_robot_action
int8 blue_robot_action
