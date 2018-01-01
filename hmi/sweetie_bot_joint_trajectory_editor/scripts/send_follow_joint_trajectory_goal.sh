#!/bin/sh

VARIABLE="
trajectory:
  header: auto
  joint_names:
  - 'joint11'
  - 'joint12'
  - 'joint13'
  - 'joint14'
  - 'joint15'
  points:
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0]
    velocities: [0.0, 0.0, 0.0, 0.0, 0.0]
    accelerations: [0.0, 0.0, 0.0, 0.0, 0.0]
    effort: [0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0]
    velocities: [0.0, 0.0, 0.0, 0.0, 0.0]
    accelerations: [0.0, 0.0, 0.0, 0.0, 0.0]
    effort: [0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0]
    velocities: [0.0, 0.0, 0.0, 0.0, 0.0]
    accelerations: [0.0, 0.0, 0.0, 0.0, 0.0]
    effort: [0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0]
    velocities: [0.0, 0.0, 0.0, 0.0, 0.0]
    accelerations: [0.0, 0.0, 0.0, 0.0, 0.0]
    effort: [0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {secs: 0, nsecs: 0}
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0]
    velocities: [0.0, 0.0, 0.0, 0.0, 0.0]
    accelerations: [0.0, 0.0, 0.0, 0.0, 0.0]
    effort: [0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {secs: 0, nsecs: 0}
path_tolerance:
- {name: 'joint11', position: 0.1, velocity: 0.1, acceleration: 0.1}
- {name: 'joint12', position: 0.1, velocity: 0.1, acceleration: 0.1}
- {name: 'joint13', position: 0.1, velocity: 0.1, acceleration: 0.1}
- {name: 'joint14', position: 0.1, velocity: 0.1, acceleration: 0.1}
- {name: 'joint15', position: 0.1, velocity: 0.1, acceleration: 0.1}
goal_tolerance:
- {name: 'joint11', position: 0.1, velocity: 0.1, acceleration: 0.1}
- {name: 'joint12', position: 0.1, velocity: 0.1, acceleration: 0.1}
- {name: 'joint13', position: 0.1, velocity: 0.1, acceleration: 0.1}
- {name: 'joint14', position: 0.1, velocity: 0.1, acceleration: 0.1}
- {name: 'joint15', position: 0.1, velocity: 0.1, acceleration: 0.1}
goal_time_tolerance: {secs: 0, nsecs: 0}
"

echo "<<<" "$VARIABLE" "\n<<<"
echo ">>>"
rostopic pub /trajectory_editor/follow_joint_trajectory_goal control_msgs/FollowJointTrajectoryGoal --once "$VARIABLE"
echo ">>>"
