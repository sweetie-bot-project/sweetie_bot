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
geometry_msgs/Pose pose
bool contact

int32 WORLD = 0	# unmovable coordinate system
int32 BASE_INTIAL = 1
int32 BASE_FINAL = 2
int32 PATH_INITIAL = 3
int32 PATH_FINAL = 4
int32 IGNORE_POSE = 5
