HMI nodes for rviz
==================

`robot_pose_marker` node
------------------

This node adds 6-DOF composite `InteractiveMarker` publishes to rviz and publishes pose as `geometry_msgs::PoseStamped` message.
Main marker allows you to controll robot stance by using `SetOperational` action interface for corresponding controller.
Selecting resouces in context menu activates corresponding limb markers which can be dragged to set position of individual limbs.
User can also controll markers positions by normalizing them and moving them to predescribed positions.
`robot_pose_marker` is fully customizable by the ROS parameters, where you can add/remove limb markers with their `PoseStamped` and `SetOperational` topic names.

### ROS interface

#### Published topics

* `pose` (`PoseStamped`) --- publish pose on marker position change (if it is enabled in context menu).
* `update`, `update_full` (`visualization_msgs::InteractiveMarkers`) 

#### Subscribed topics

* `tf`, `tf_static`
* `feedback` (`visualization_msgs::InteractiveMarkersFeedback`) 

#### Actionlib

* Client: `stance_set_operational_action` (`SetOperational`) --- activate or deactivate corresponding controller with resource set selected in context menu.

#### Parameters

* `~name` (`string`) --- displayed stance marker name. Default: `''`
* `~world_frame` (`string`) --- world frame string id. Default: `'odom_combined'`
* `~scale` (`double`) --- stance marker scale. Default: `1.0`. Original marker is a 16cm x 8cm x 3cm size box.
* `~normalized_z_level` (`double`) --- if user select `Normalize pose` in context menu, marker will be oriented parallel to the XY plane and placed at `normalized_z_level` along Z axis. Default: `0.0`.
* `~frame` (`string`) --- "home" frame where marker should be automatically placed before activation. Empty string means no automatic placement. Default: `''`.

These parameters are describing main marker. But the same pattern applies to the limb markers as well.
Both limbs and legs markers described in `~inner_markers/` namespace:

* `~inner_markers/legs/list` --- describes dictionary of leg parameters . Must contain all 4 entries.
* `~inner_markers/legs/list/legX/resource` (`string`) --- name of resource (kinematic chain) associated with `legX`. No default value.
* `~inner_markers/legs/list/legX/frame` (`string`) --- name of "home" frame associated with `legX`. Default: `''`.
* `~inner_markers/legs/normalized_z_level` (`double`) --- same as above for main marker but shared between all legs markers.
   Default: `0.0`
* `~inner_markers/legs/scale` (`double`) --- same as above for main marker but shared between all legs markers.
   Default: `1.0`

`~inner_markers/limbs/` contains list of parameter namespaces related to corresponding configurable limb markers. Each of these namespaces has all parameters that the main marker have (`name`, `scale`, `normalized_z_level`, `frame`), but also contains additional ones:

* `~inner_markers/limbs/limb_name/is_sphere` (`bool`) --- flag that indicates what shape the marker should be: sphere or parallelepiped.
   Default: `true`
* `~inner_markers/limbs/limb_name/frame` (`string`) --- "home" frame, associated with `limb_name`. Default: `''`.
* `~inner_markers/limbs/limb_name/resource` (`string`) --- controllable resource (kinematic chain), associated with `limb_name`.
   Default: `''`.
* `~inner_markers/limbs/limb_name/pose_topic` (`string`) --- name of topic where `PoseStamped` message for this marker should be published.
   Default: `limb_pose`
* `~inner_markers/limbs/limb_name/activation_action` (`string`) --- name of the action server topic where `SetOperational` message for this marker sending out.
   Default: `limb_set_operational_action`

`generic_pose_marker` node
------------------

This node implements generic pose marker architecture, which intended for debugging purposes.

### ROS interface

#### Publish topics

* `pose` (`PoseStamped`) --- publish pose on marker position change (if it is enabled in context menu).
* `update`, `update_full` (`visualization_msgs::InteractiveMarkers`) 


#### Subscribe topics

* `tf`, `tf_static`
* `feedback` (`visualization_msgs::InteractiveMarkersFeedback`) 

#### Actionlib

* Client: `set_operational_action` (`SetOperational`) --- activate or deactivate corresponding controller with resource set selected in context menu.

#### Parameters

* `~name` (`string`) --- displayed marker name. Default: `''`.
* `~world_frame` (`string`) --- world frame string id. Default: `'odom_combined'`
* `~scale` (`double`) --- marker scale. Default: `1.0`. Original marker is a 16cm x 8cm x 3cm size box.
* `~resources` (`string[]`) --- the set of resources (kinematic chains) to display in context menu. 
    Default: `['leg1', 'leg2', 'leg3', 'leg4', 'nose']`
* `~frames` (`string[]`) --- the list of frames to be displayed in "Move to frame" context submenu. User can easily move marker to any of it. 
    Default: `['bone15', 'bone25', 'bone35', 'bone45', 'bone55', 'base_link']`
* `~resources_select_only_one` (`bool`) --- only one resource can be active at time. Default: `false`.
* `~normalized_z_level` (`double`) --- if user select `Normalize pose` in context menu marker is oriented parallel XY plane and placed at `normalized_z_level` along Z axis. Default: 0.0.
* `~select_only_one_resource` (`bool`) --- flag that indicates whether user allowed to select only one resource or multiple. Default: `false`.


`destination_marker` node
------------------

This node contains `InteractiveMarker` which invoke `MoveBase` action allowing you to control the gait generator (`clop_generator`) by publishing `sweetie_bot_clop_generator::MoveBaseGoal` message.
Marker itself can be placed in desired gait target position. Arrow above the marker points in the direction of the robot gaze after it reaches the goal.
Every time when gait gets executed, resulting trajectories recorded into `saved_msgs/step_sequence/recorded_trajectory` and `saved_msgs/move_base/recorded_trajectory` ROS parameters. It happens by calling `clop_generator/save_trajectory` service with `sweetie_bot_clop_generator::SaveTrajectory` message.
Context menu contains `clop_generator` settings (type of gait, number of steps, gait duration), allows to change name of recorded trajectory, as well as invoke `clop_generator`.

### ROS interface

#### Published topics

* `update`, `update_full` (`visualization_msgs::InteractiveMarkers`) 

#### Subscribed topics

* `tf`, `tf_static`
* `feedback` (`visualization_msgs::InteractiveMarkersFeedback`) 

#### Called services

* Client: `/clop_generator/save_trajectory` (`SaveTrajectory`) --- service which saves last executed gait as trajectories in ROS parameters.

#### Actionlib

* Client: `move_base_action` (`MoveBase`) --- invokes `clop_generator` with end effectors described in parameters.

#### Parameters

* `~name` (`string`) --- displayed marker name. Default: `''`.
* `~world_frame` (`string`) --- world frame string id. Default: `'odom_combined'`
* `~scale` (`double`) --- marker scale. Default: `1.0`.
* `~gait_type_options` (`string[]`) --- list of the gait type options contained in corresponding submenu. No default value.
* `~gait_type_default_idx` (`int`) --- gait type list index selectable by default. Default: `0`.
* `~n_steps_options` (`int[]`) --- list of the gait steps number options contained in corresponding submenu. No default value.
* `~n_steps_default_idx` (`int`) --- number of steps list index selectable by default. Default: `0`.
* `~duration` (`double`) --- duration of the gait in seconds. User can change its value in context menu. Default: `4.0`.
* `~nominal_height` (`double`) --- target height of the robot base after reaching the position. Default: `0.1825`.
* `~recorded_trajectory_name` (`string`) --- default recorded trajectory name. Default: `'recorded_trajectory'`
* `~ee_names` (`string[]`) --- the set of `clop_generator` end effector names.
    Default: `[]`

