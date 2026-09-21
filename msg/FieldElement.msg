# Any scoring object (pin or cup) on the field, as tracked by pose_bridge.
# element_type: 1 = pin, 2 = cup
string object_name
int32 id
int8 element_type
int8 top_color
int8 bottom_color
geometry_msgs/Point location
geometry_msgs/Quaternion orientation
