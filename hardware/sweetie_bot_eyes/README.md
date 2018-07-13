Sweetie Bot Eyes (displays)
==================

`eye` node
------------------

QT node which draws eye in window or any other provided backend. It reads eye position from `joint_states` (`eyes_pitch` and `eyes_yaw` joint) 
and can perform eyes-related `TextCommand` on `control` topic.

Supported `TextCommand` types:
* `eyes/action` --- perform scripted action and return to previous state. ("blink", "slow\_blink")
* `eyes/emotion` --- change eyes state ("normal", "red\_eyes", "sad\_look", "evil\_look")

Also component supports deprecated keyboard control interface.

### ROS interface

#### Publish topics

* `eye_image_left` or `eye_image_right` (`sensor_msg::Image`) --- publish eye image for `rviz_texture_quad` plugin if `publish_pixmap` parameter is set.

#### Subscribe topics

* `control` (`TextCommand`) --- text command to perform.
* `joint_states` (`JointState`)  --- eyes positions (`eyes_pitch` and `eyes_yaw` joints).

#### Actionlib

#### Parameters

* `~is_left` (`bool`, false) --- select left/right eye.
* `~publish_pixmap` (`bool`, false) --- publish image on `eye_image_*` topic.

