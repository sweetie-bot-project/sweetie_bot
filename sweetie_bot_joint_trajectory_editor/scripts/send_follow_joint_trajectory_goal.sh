#!/bin/sh

VARIABLE="
trajectory:
  header: auto
  joint_names:
  - 'joint11'
  - 'joint12'
  points:
  - positions: [0]
    velocities: [0]
    accelerations: [0]
    effort: [0]
    time_from_start: {secs: 0, nsecs: 0}
path_tolerance:
- {name: '', position: 0.0, velocity: 0.0, acceleration: 0.0}
goal_tolerance:
- {name: '', position: 0.0, velocity: 0.0, acceleration: 0.0}
goal_time_tolerance: {secs: 0, nsecs: 0}
"

echo "<<<" "$VARIABLE" "\n<<<"
echo ">>>"
rostopic pub /trajectory_editor/follow_joint_trajectory_goal control_msgs/FollowJointTrajectoryGoal --once "$VARIABLE"
echo ">>>"
