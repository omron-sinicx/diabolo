#These enums define the possible states of the diabolo relative to the string_length
#See diabolo gazebo plugin README for more details
uint8 ON_STR=1
uint8 OFF_STR_LOOSE=2
uint8 FLY=3
uint8 OUTSIDE_STR=4

std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Point trans_velocity
float32 rot_velocity
float32 mass
float32 string_length #Ignored if 0, or less than initial distance between strings

# The state of the diabolo relative to the string 
# It is one of the values defined in the above enums
int32 string_state