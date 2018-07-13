HMI nodes for rviz
==================

`pose_marker` node
------------------

This node adds 6-DOF `InteractiveMarker` publishes to rviz and publishes pose as `geometry_msg::PoseStamped` message.
Also it able to manage corresponding motion controller using context menu. Internally it `SetOperational` action to 
activate and deactivate controller.

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

* `scale` (`double`) --- marker scale.
* `resources` (`string[]`) --- set of resources (kinematic chains) to display in context menu.
* `resources_select_only_one` (`bool`) --- only one resource can be active at time.
* `marker_home_frame` (`string`) --- frame where marker should be placed before activation.
* `normalized_z_level` (`double`) --- if user select `Normalize pose` in context menu marker is oriented parallel XY plane and placed at `normalized_z_level` along Z axis.

