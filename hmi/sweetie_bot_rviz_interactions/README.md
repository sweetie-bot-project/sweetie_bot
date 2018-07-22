HMI nodes for rviz
==================

`pose_marker` node
------------------

This node adds 6-DOF `InteractiveMarker` publishes to rviz and publishes pose as `geometry_msg::PoseStamped` message.
Also the context menu of the marker allows to manage corresponding motion controllers which implement `SetOperational` action interface.
User can activate or deactivate controller ("OPERATIONAL" menu entry), change controlled resources set, move marker to predefined frames.

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

* `~scale` (`double`) --- marker scale. Default: `1`. Original marker is a 16cm x 8cm x 3cm size box.
* `~resources` (`string[]`) --- the set of resources (kinematic chains) to display in context menu. 
    Default: `['leg1', 'leg2', 'leg3', 'leg4']`
* `~frames` (`string[]`) --- the list of frames to be displayed in "Move to" context submenu. User can easily move marker to any of it. 
    Default: `['bone15', 'bone25', 'bone35', 'bone45', 'bone54', 'base_link']`
* `~resources_select_only_one` (`bool`) --- only one resource can be active at time. Default: `false`.
* `~marker_home_frame` (`string`) --- frame where marker should be automatically placed before activation. Empty string means no automatic placement. Default: `''`.
* `~normalized_z_level` (`double`) --- if user select `Normalize pose` in context menu marker is oriented parallel XY plane and placed at `normalized_z_level` along Z axis. Default: 0.0.

