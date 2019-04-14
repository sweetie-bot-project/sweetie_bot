# sweetie_bot_kinematics_msgs/EndEffectorGoal
# Goal state of end effector.
# 
# * name --- end effector name.
# * frame_type --- frame of the end effector pose.
# * contact --- should be end effector be in contact at the end of motion.
# * pose --- desired pose. Note that towr ignores end effector orientation.
#
string name
uint32 frame_type
geometry_msgs/Point position
bool contact

int32 IGNORE_POSE = 0
int32 NOMINAL_POSE = 1
int32 BASE_INITIAL = 2
int32 BASE_FINAL = 3
int32 PATH_INITIAL = 4
int32 PATH_FINAL = 5
int32 WORLD = 6	# unmovable coordinate system
