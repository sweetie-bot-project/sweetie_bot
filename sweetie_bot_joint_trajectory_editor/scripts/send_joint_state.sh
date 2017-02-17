#!/bin/sh
VARIABLE="
{
 header: auto,
 name: ['joint11', 'joint12', 'joint13', 'joint14', 'joint15',
        'joint21', 'joint22', 'joint23', 'joint24', 'joint25',
        'joint31', 'joint32', 'joint33', 'joint34', 'joint35',
        'joint41', 'joint42', 'joint43', 'joint44', 'joint45',
        'joint51', 'joint52', 'joint53', 'joint54'],
 position: [0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0],
 velocity: [],
 effort:   []
}"
echo "<<<" "$VARIABLE" "\n<<<"
echo ">>>"
rostopic pub /trajectory_editor/joints_real sensor_msgs/JointState --once "$VARIABLE"
echo ">>>"
