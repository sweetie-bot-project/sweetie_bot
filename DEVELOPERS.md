## Installation from sources

Your may compile Sweetie Bot manually in ROS workspace but this method is not trivial and not recommended.
This method does not conflicts with installation from binary packages due to ROS overlay mechanism.

#### Project dependencies

Internal dependencies:

* [`sweetie_bot_proto2_movements`](https://gitlab.com/sweetie-bot/sweetie_bot_proto2_movements) --- stored movements for Proto2,
* [`sweetie_bot_sounds`](https://gitlab.com/sweetie-bot/sweetie_bot_sounds) --- sound package.

External dependencies:

* [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation) or later. We need following packages:
    * `ros_base` --- basic ROS installation.
	* MoveIt! packages.
    * `eigen-conversions`, `tf-conversions`, `interactive-markers`.
	* `orocos_kdl` and `track_ik` kinematics packages.
	* [`rospy_message_converter`](https://github.com/baalexander/rospy_message_converter)
	* [`rviz_textured_quads`](https://github.com/lucasw/rviz_textured_quads)
* [OROCOS 2.9](https://github.com/orocos-toolchain/orocos_toolchain), it is recommended slightly modified version from [here](https://github.com/disRecord) with improved lua completion. Also ROS package may be used but it may have some limitation. 
* Additional ROS packages
    * [`rtt-ros-integration` 2.9](https://github.com/orocos/rtt_ros_integration) (You may use ROS packages).
    * [`kdl_msgs`](https://github.com/orocos/kdl_msgs), [rtt_kdl_msgs](https://github.com/orocos/rtt_kdl_msgs), [`rtt_geometry`](https://github.com/orocos/rtt_geometry).
    * [`rttlua_completion`](https://github.com/orocos-toolchain/rttlua_completion), [`patched version recommended`](https://github.com/disRecord)
    * `rtt_tf2_msgs`,`rtt_control_msgs` typekit packages can be generated with [`rtt_roscom`](https://github.com/orocos/rtt_ros_integration/tree/toolchain-2.9/rtt_roscomm)
* [Rigid Body Bynamics Library 2.6](https://rbdl.bitbucket.io/) `next` branch.
* [ALGLIB library](http://www.alglib.net) you may use package `libalglib-dev`)
* [FlexBe](http://philserver.bplaced.net/fbe/) behavior framework.
	* [`flexbe_behavior_engine`](https://github.com/team-vigir/flexbe_behavior_engine/tree/feature/flexbe_app), specifically `feature/flexbe_app` branch for `felxbe_app` support.
	* [`flexbe_app`](https://github.com/FlexBE/flexbe_app).
    * [`felexbe_general_states`](https://github.com/FlexBE/generic_flexbe_states).
* QT5 development packages (`libqt5-dev`).

Let's assume that all build requirements are satisfied. Or you can install them from binary package `ros-melodic-sweetie-bot-base`.

Create ROS workspace:
```
mkdir -p ~/ros/sweetie_bot/src 
```

Clone dependencies if necessary and generate typekit packages if they not installed. 
If you are using `ros-melodic-sweetie-bot-base` package only FlexBe and `rviz_textured_quads` are needed.
```
cd ~/ros/sweetie_bot/src; mkdir inc; cd inc
git clone https://github.com/lucasw/rviz_textured_quads.git
git clone -b feature/flexbe_app https://github.com/team-vigir/flexbe_behavior_engine.git
git clone https://github.com/FlexBE/flexbe_app.git
```
Due to the bug (quads are always black) it is recommended to install `rviz_textured_quads` in Sweetie Bot workspace.

Clone SweetieBot sources:
```
cd ~/ros/sweetie_bot/src
git clone -b devel --recursive git@gitlab.com:sweetie-bot/sweetie_bot.git
git clone git@gitlab.com:sweetie-bot/sweetie_bot_sounds.git
git clone git@gitlab.com:sweetie-bot/sweetie_bot_proto2_movements.git
git clone git@gitlab.com:sweetie-bot/sweetie_bot_flexbe_behaviors.git
mkdir msgs; cd msgs;
git clone https://github.com/orocos/rtt_kdl_msgs
git clone https://github.com/orocos/kdl_msgs.git
git clone https://github.com/orocos/rtt_geometry.git
rosrun rtt_roscomm create_rtt_msgs control_msgs
rosrun rtt_roscomm create_rtt_msgs tf2_msgs
```
Compile:
```
source /opt/ros/melodic/setup.bash
cd ~/ros/sweetie_bot
catkin_make
``` 