# A pin placed on the field or in a goal.
# Color codes: 1 = red, 2 = blue, 3 = yellow, 0 = none
string object_name
int32 id
int8 top_color      # color of the top half of the pin
int8 bottom_color   # color of the bottom half of the pin
geometry_msgs/Point location
geometry_msgs/Quaternion orientation
