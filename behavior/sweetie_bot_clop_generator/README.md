Sweetie Bot Gait Generator
==========================

Gait generator utilizes optimization based method. This package contains ROS node implementing 
actionlib interface for TOWR library. Original TOWR code developed by A. Winkler is pretty slow
and do not have many features (planar gaits, end-effector position control and so on) so 
this package depends on following fork: https://github.com/sweetie-bot-project/sweetie_bot


`clop_generator` node
---------------------

This node receives motion requests via actionlib, performs motion generation and then moves robot by passing 
resulting trajectory to execution system.

Motion request specifies desired base position and optionally positions of legs. Resulting trajectory 
specifies motions of leg in cartesian coordinates relative to robot base link.

Gait generator configuration is preformed via ROS parameters. Configuration contains kinematic and dynamic models of the robot, 
optimization problem parametrization parameters and solver properties. Reconfiguration can be performed only by restarting node.

### Terrain surface

Terrain is assumed flat.

### Robot model

TOWR uses simplified robot model. Robot consists of base link and four end effectors (legs). All contacts are assumed to be point contacts so
with each end effector is associated contact point. There are following frames: 
* **Base link frame** can have arbitrary orientation if 6D planner is used. In case of planar planner have the same orientation as path frame.
* **Path frame** is projection of base link frame on support surface. Z-axis always points upward.
* **End effector (leg) frames** always have the same orientation as path frame.

**Dynamic model**.  Base link is presented by single rigid bode. It has mass and associated inertia tensor. End effectors are point masses. 
Its have only masses which for simplicity can be set to zero. In this case corresponding link lists should be empty. 

**Kinematic model** limits end effector movements. There are there three types of constraints:
* **bounding box** is parallelepiped with faces parallel to base link axes.
* **bounding spheres** limit distance between contact points and "shoulders".
* **collision ellipsoid** limit distance between end effectors.

### Initial pose

Current robot pose received via `tf` subsystem is used as initial pose. Legs that are close enough to support surface is assumed in contact with it.

### Target pose 

Desired pose is specified with `sweetie_bot_clop_generator/MoveBaseGoal` message. For more information see message specification.

The most important fields^
* `gait_type` is one of supported gait: `free`, `stand`, `walk[_right]`, `walk_overlap[_right]`, `trot[_right]`, `pace[_right]`, `pronk[_right]`.  Gait type specifies leg movement sequence:
   * `free` is special type of gait which is designed to perform pose change. Legs are moved one after another.
   * `walk` is classical walk gait. At each moment at least three legs are on support surface.
   * `walk_overlap` is walk gait which contains phase when robot balance on two legs. This gait is faster then walk.
   * `trot` is fastest gait. Robot moves two legs at one time. 
   * `*_right` are right versions of gaits. By default movement is started from front left leg. 
* `duration` is movement duration in seconds.
* `n_steps` is number of full steps.
* `base_goal` and  `base_goal_bounds` specifies desired base position (position and orientation).
* `ee_goal` are desired end effector position. 
* `visualize_only` do not execute movement.
* `execute_only` executes previously planned trajectory. 

### Nonlinear optimization problem (NLP)

Movement trajectory is solution of optimization problem. Initial pose is specified by current robot pose, target pose is defined by MoveBaseGoal message. 

Gait generator supports different NLP formulations:
| **NLP name**   | **Movement type** | **Optimization variables** | **Constraints with time-based discretization** | **Constraints with phase-based discretization** | **EE motion constraints**              |
| 6D             | 6D                | base, EEs, forces          | stability, EE range of motion                  | swing                                           | box, sphere                            |
| 6D_phase       | 6D                | base, EEs, forces          | EE range of motion                             | stability, swing                                | box, sphere                            |
| 6D_phase2      | 6D                | base, EEs, forces          |                                                | stability, EE range of motion, swing            | box, sphere                            |
| 6D_free        | 6D                | base, EEs, forces          | stability, EE range of motion                  | swing                                           | sphere, distance between end effectors |
| 6D_free_phase  | 6D                | base, EEs, forces          | EE range of motion                             | stability, swing                                | sphere, distance between end effectors |
| 6D_free_phase2 | 6D                | base, EEs, forces          |                                                | stability, EE range of motion, swing            | sphere, distance between end effectors |
| jump           | 6D                | base, EEs, forces          | stability, EE range of motion                  |                                                 | sphere, box                            |
| planar         | planar            | base, EEs, forces          | stability, EE range of motion                  | swing                                           | sphere, box                            |
| zmp            | planar            | base, EEs                  | stability, EE range of motion                  | swing                                           | box, sphere                            |
| zmp_phase      | planar            | base, EEs                  | EE range of motion                             | stability, swing                                | box, sphere                            |
| zmp_phase2     | planar            | base, EEs                  |                                                | stability, EE range of motion, swing            | box, sphere                            |

Recommended formulations: "6D" (general 6D gait), "6D_free_phase" (pose changes with "free" gait), "zmp_phase" (planar motion).
Recommended gait: "walk_overlap" or "trot".

### Resulting trajectory

Resulting trajectory is stored in `FollowStepSequenceGoal` trajectory message. See message file for more details.
It contains time series which defines base frame position in world frame and end effector frame positions relative to base frame.

The most important fileds:

* `time_from_start` defines time discretization (uniform).
* `base_motion` contains base orientation and position.
* `ee_motion[]` contains end effectors orientation and position relaative to base frame.
* `append` is bool flag which indicates that two trajectories should be concatenated.

### ROS interface

#### Subscribe 

* `tf` --- current robot pose in cartesian coordinates.

### Services

**Provides**:

* `save_trajectory` (`sweetie_bot_clop_generator/SaveTrajectory`) --- save the last generated trajectory in ROS parameter with given name in namespace specified by  `storage_namespace` parameter.

#### Actionlib

**Provides**:

* `.` (`sweetie_bot_clop_generator/MoveBase`, actionlib server) --- receive move requests, perform motion planning and execute resulting trajectory.

**Requires**:

* `step_sequrnce` (`sweetie_bot_control_msgs/FollowStepSequence`, actionlib client) --- send resulting trajectory to execution.

### Parameters

* `robot_description` (string) --- robot model in URDF format.
* `period` (float) --- descretization period of resulting trajectory (seconds, 0.0056)
* `contact_height_tolerance` (float, 0.003) --- if during the initilaization phase distance between terrain surface and end-effector is less then this value end-effector is assumed to be in contact with terrain.
* `world_frame` (string, "odom_combined") --- unmoving frame linked with terrain 
* `planning_frame` (string, "base_link_path") --- unmoving frame in which motion planning is performed.
* `storage_namespace` (string, "saved_msgs") --- parameter namespace where planned trajectories are saved by `save_trajectory` service. Trajectory is represented by `FollowStepSequenceGoal` message, which is saved in serialized form.
* `towr_model` (parameters subtree) --- robot model for motion planning
  * `nlp_type` (string, "zmp_phase") --- NLP formulation name. See coresponding section for detials.
  * `base` (parameters subtree) --- information robot robot base
    * `frame_id` (string, "base_link") --- robot base link frame
    * `links`: (string[], ["base_link", "base_link_inertia", ...]) --- these links are considered to be part of robot base in simplified robot model. Their masses and inertias are summed.
  * `LF` (parameters subtree) --- left front end-effector description
	`name` (string, "leg1") --- end-effector name (see `sweetie_bot_robot_model` package description)
	`frame_id` (string) --- end-effector frame. During motion planning it is assumed that z-axis points upward and x-axis points forwards.
	`contact_point` (float[3], [ 0.0, 0.0, 0.0 ]) --- position of actual contact point in end-effector frame. Actually TOWR calculate contact point movement the in translated to end-effector trajectory.
	`nominal_stance` (float[3], [x, y, z]) --- contact point nominal position relative to base link frame.
	`bounding_box` (float[6], [ x_min, y_min, z_min, x_max, y_max, z_max ]) --- bounding box for end effector movements in base link frame.
	`bounding_sphere` (float[4], [ x, y, z, r_min, r_max ]) --- bounding spheres for end effector movements in base link frame. The distance between (x,y,z) and end effector should be between r_min and r_max.
	`links` (string[], ["link13", "link14", ...]) --- these links masses are used to calculate end-effector mass and inertia.
  * `RF`, `LH`, `RH` (parameters subtree) --- right front, left hind and right hind end-effectors descriptions.
  * `ee_collision_ellipsoid` (float[3], [ rx, ry, rz ]) --- some kinematic models assumes that  end-effector are collisions if ellipsoids with centers in EE contact points are intersects (in path frame).
* `towr_parameters` (parameters subtree) --- TOWR NLP parameters (see towr::Parameters structure description for more information) 
  * `scale_per_step` (bool, false) --- scale time-based discretization periods with movement duration (not recommended). In this case `dt_*` periods are represented in fractions of full step duration.
  * `base_phase_durations` (bool, true) --- use phase-based discritization for base movements.
  * `duration_base_polynomial` (float, 0.25) --- discretization period for base movements, it is ignored if phase-based parametrization is on.
  * `dt_constraint_dynamic` (float, 0.1) --- period of dynamic constraints checks
  * `dt_constraint_range_of_motion` (float, 0.1) --- period of kinematic constraints checks
  * `dt_constraint_base_motion`: 0.1
  * `ee_polynomials_per_swing_phase` (float, 2) --- number of polynomials for end-effector swing parametrization.
  * `optimize_phase_durations` (bool, false) --- not supported. Should be always false.
  * `min_swing_height` (float, 0.015) --- distance between end-effector contact point and terrain surface during swing.
  * `stability_margin` (float, 0.02) --- minimal stability margin for ZMP-based stability constraint.
  * `costs` (parameters subtree) cost's names and weights
    * `base_lin_acc` (float, 0.1)
    * `base_ang_acc` (float, 0.01)
    * `ee_acc` (float, 0.0025)
* `ipopt_solver_parameters` (parameters subtree) --- IPOPT solver parameters (see solver documentation)
  * `linear_solver` (string, "mumps")
  * `derivative_test` (string, "none" or "first-order") ---  Note that "first-order" test breaks optimization process, so use it for debug purpose only.
  * `max_cpu_time` (float, 20.0) --- maximal optimization duration.
  * `max_iter` (int, 30) --- maximal number of iterations.
  * `print_level` (int, 5) --- print level.

Helper library
--------------

`clopper` is python helper library which simplifies `MoveBaseGoal` message construction and handling. 
See source code for documentation in docstring format and `scripts` directory for examples.

Typical usage pattern:

    from sweetie_bot_clop_generator.clopper import Clopper, MoveBaseGoal
    # create message (clopper.MoveBaseGoal is subclass of sweetie_bot_clop_generator.MoveBaseGoal)
	# set gait type, number of steps and movement duration
    msg = MoveBaseGoal(gait_type = "walk_overlap", n_steps = 4, duration = 3.4)
	# set realtive shift of base link
    msg.setTargetBaseShift(x = 0.4, y = 0.0, angle = 0.0) 
	# create default end-effector tragets: nominal poses
    msg.addEndEffectorsTargets(["leg1","leg2","leg3","leg4"]) 

    # send message to server
    clop = Clopper("clop_generator") # "clop_generator" is gait generator node name
    clop.invokeClopGenerator(msg) 

