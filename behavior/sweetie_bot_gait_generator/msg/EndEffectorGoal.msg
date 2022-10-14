# sweetie_bot_kinematics_msgs/EndEffectorGoal
# Goal state of end effector.
# 
# * name --- end effector name.
# * frame_type --- frame of the end effector pose.
# * contact --- should be end effector be in contact at the end of motion.
# * position --- desired pose.
# * position_bounds --- which coordinates of final pose are fixed or free.
#     Zero bits indicate that coresponding dimension of final pose is fixed. The order of coordinates is (X,Y,Z) from low bit to high.
#
string name
uint32 frame_type
geometry_msgs/Point position
uint32 position_bounds
bool contact

int32 NOMINAL_POSE = 0
int32 BASE_INITIAL = 1
int32 BASE_FINAL = 2
int32 PATH_INITIAL = 3
int32 PATH_FINAL = 4
int32 WORLD = 5	# unmovable coordinate system

uint32 POSITION_FIXED = 0
uint32 POSITION_FREE_XY = 3
uint32 POSITION_FREE_Z = 4
uint32 POSITION_FREE = 7
