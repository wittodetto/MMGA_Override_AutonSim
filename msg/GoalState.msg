# State of one of the nine Override goals.
# goal_type: 0 = tall neutral center goal, 1 = short neutral goal,
#            2 = red alliance goal, 3 = blue alliance goal
# toggle_state: 0 = neutral (yellow), 1 = red, 2 = blue
# quadrant: 'N' 'E' 'S' 'W' (diagonal quadrants), 'C' for the center goal
int32 goal_id
int8 goal_type
int8 toggle_state
string quadrant
PinElement[] pins
CupElement[] cups
