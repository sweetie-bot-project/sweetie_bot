Trajectory Editor 
=================

TrajectoryPinkitor will make you live fun! Because it is fun to teach SweetieBot new movements.
This package provide GUI for creating movement trajectories by specifying a set of way points.
This package is the part of [Sweetie Bot project](sweetiebot.net).  Full documentation 
is available in Russian [here](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-animation-stored-move).


### General concept

* To specify trajectory you should select active joints, set joint tolerances and specify a set of waypoints. Only selected joints' states are stored and published.

* Trajectories are stored on Parameter Server in serialized in binary form `control_msgs::FollowJointTrajectoryGoal` messages.
     Parameter `~trajectory_storage` specifies namespace for serialized messages. Use `store` script from `sweetie_bot_deploy` to save and load them from .json files.
	 TrajectoryPinkiptor doe not save trajectories to files, it only modifies parameteres value on Parameter Server.

* If you are working with real robot `Set troque on/off` buttons can be used to switch servos on or off. Trajectory editor `set_torque_off` service (`std_srvs::SetBool`) 
    to control servo state. (See `MainTorqueSwitch` controller from `sweetie_bot_controllers_joint_space` package).

* New way point can be added in two ways. You can duplicate existing point or you can add current robot pose from `joint_state` (`sensor_msgs::JointState`) topic.
    To place robot in desired pose you can utilize `joint_state_publiser` GUI or MoviIt! rviz plugin. If you are working with real robot it can be placed in 
	desired pose manually if the servos are turned off.

* When you press `Set robot pose` current way point is published on topic `joint_state_set` (`sensor_msgs::JointState`). If there is controller which monitors this topic it can place 
    robot in specific pose (e.g. `FollowJointState` controller from `sweetie_bot_controllers_joint_space` package).

* When you select way point it is published on topic `joint_marker_set` (`sensor_msgs::JointState`). 

* When your press `Execute trajectory` button `FollowJointTrajectoryGoal` messages is formed from current way point list and editor requests `FollowJointTrajectory` 
    action server with name `joint_trajectory`  to execute trajectory. (See `ExecuteJointTrajectory` controller from `sweetie_bot_controllers_joint_space` package)

* `FollowJointTrajectoryGoal` do not store information about contacts. As walkaround special joints `support/<kinematic_chain_name>` was introduced. Sweetie Bot action server 
    assumes that corresponding kinemtic chain is in contact but oes not test movement consistently. Those joints are supported in TrajectoryPinkiptor interface and displayed as 
    checkboxes. If contact is set it is active is period from the current to the next waypoint,
    

### ROS interface

#### Publish topics

* `joints_marker_set` (`JointState`) --- pose clicked by user in waypoints' list.
* `joints_state_set` (`JointState`) --- publish pose if `Set robot pose` button is clicked.

#### Subscribe topics

* `joints_state` (`JointState`) --- current robot pose.

#### Services

* Requires: `set_torque_off` (`std_srvs::SetBool`) --- turn robot servos off.

#### Actionlib

* Client: `joint_trajectory` (`FollowJointTrajectory`) --- execute trajectory if `Execute trajectory` button is pressed.

#### Parameters

* `~trajectory_storage` (`string`, default `joint_trajectory`) --- trajectory storage namespace.

### Usage

Run SweetieBot simulation ans start trajectory editor.

    roslaunch sweetie_bot_deploy joint_space_control.launch
    roslaunch sweetie_bot_deploy joint_trajectory_editor.launch
