
=======
Sweetie Bot software repository 
===================================

This repository contains software framework for the [Sweetie Bot Proto2 robot](http://sweetie.bot).

![](doc/figures/control-system.png)

## Overview

Sweetie Bot software is based on [Robot Operating System (ROS)](http://wiki.ros.org/ROS/Introduction). Also it uses [OROCOS](http://www.orocos.org/wiki/orocos/toolchain/getting-started) middleware to implement real-time motion control subsystem. See [`sweetie_bot_deploy`](config/sweetie_bot_deploy) and  [`rt_control`](rt_control) for more details (namespaces, nodes, configuration parameters).

Sweetie Bot software support two modes: real and virtual. 
1. In a virtual mode it runs completely on the host computer. The real robot is not needed. You can program robot behaviors in a virtual enviroment (3d model). 
2. In a real mode it controls the real robot. One part of software runs the host computer and another part works the on-board computer. 

Current version of Sweetie Bot software provides following functionality:
1. Joint level control using [`joint_state_publisher`](http://wiki.ros.org/joint_state_publisher).
2. The control in Cartesian space (for the limbs and the body) using [pose markers](hmi/sweetie_bot_rviz_interactions) and [MoveIt!](https://moveit.ros.org/) integration.
3. Movements creation (See [TrjectoryEditor](hmi/sweetie_bot_trajectory_editor))
4. High-level control based on [FlexBe](http://philserver.bplaced.net/fbe/) hybrid finite state machines via external repository [`sweetie_bot_flexbe_behaviors`](https://gitlab.com/sweetie-bot/sweetie_bot_flexbe_behaviors).

<!-- Watch the Software demonstration video [here](https://www.youtube.com/watch?v=FTKn_fK0Puo): -->

## Repository content

This repository contains all necessary software components to run the Sweetie Bot robots except for movements, sounds and actual behaviors which are stored in separate repositories
([`sweetie_bot_proto2_movements`](https://gitlab.com/sweetie-bot/sweetie_bot_proto2_movements), [`sweetie_bot_sounds`](https://gitlab.com/sweetie-bot/sweetie_bot_sounds) and 
[`sweetie_bot_flexbe_behaviors`](https://gitlab.com/sweetie-bot/sweetie_bot_flexbe_behaviors).

* [`config`](config) directory contains robot configuration and deployment scripts
    * `sweetie_bot_deploy` --- robot-independent OROCOS deployment scripts and ROS launch files.
	* `sweetie_bot_proto2_deploy` --- robot-specific parameters and launch files for Proto2 Sweetie Bot robot.
	* `sweetie_bot_proto2_description` --- URDF model of Proto2 (git submodule).
	* `sweetie_bot_proto2_moveit_config` --- MoveIt! configuration for Proto2.
* [`rt_control`](rt_control) --- OROCOS-based motion control subsystem (git submodule).
* [`hardware`](hardware) --- hardware-depended components.
    * `sweetie_bot_eyes` --- eyes visualization.
* [`behavior`](behavior) --- high-level control subsystem (not implemented yet).
* [`hmi`](hmi) --- operator interface components, intended to be running on the host computer.

## Usage

To start basic control framework use 

    roslaunch sweetie_bot_deploy joint_space_control.launch

Following command starts MoveIt! `move_group` and FlexBe subsystem (your need `sweetie_bot_flexbe_behaviors` package which provides behaviors and states).

    roslaunch sweetie_bot_deploy flexbe_control.launch run_flexbe:=true

For more details see [`sweetie_bot_deploy` package](config/sweetie_bot_deploy).

## Installation

### Project dependencies

Internal dependencies:
* [`sweetie_bot_proto2_movements`](https://gitlab.com/sweetie-bot/sweetie_bot_proto2_movements), 
* [`sweetie_bot_sounds`](https://gitlab.com/sweetie-bot/sweetie_bot_sounds)

External dependencies:

* [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation) or later. We need following packages:
    * `ros_base` --- basic ROS installation.
	* MoveIt! packages.
	* `orocos_kdl` and `track_ik` kinematics.
	* [`rospy_message_converter`](https://github.com/baalexander/rospy_message_converter)
	* [`rviz_textured_quads`](https://github.com/lucasw/rviz_textured_quads)
* [OROCOS 2.9](https://github.com/orocos-toolchain/orocos_toolchain), it is recommended slightly modified version from [here](https://github.com/disRecord) with improved lua completion. Also ROS package may be used but it may have some limitation. 
* Additional ROS packages
    * [`rtt-ros-integration` 2.9](https://github.com/orocos/rtt_ros_integration) (You may use ROS packages).
    * [`kdl_msgs`](https://github.com/orocos/kdl_msgs), [rtt_kdl_msgs](https://github.com/orocos/rtt_kdl_msgs), [`rtt_geometry`](https://github.com/orocos/rtt_geometry).
    * [`rttlua_completion`](https://github.com/orocos-toolchain/rttlua_completion), рекомендуется модифицированная версия [отсюда](https://github.com/disRecord)
    * `rtt_tf2_msgs`,`rtt_control_msgs` typekit packages can be generated with [`rtt_roscom`](https://github.com/orocos/rtt_ros_integration/tree/toolchain-2.9/rtt_roscomm)
* [Rigid Body Bynamics Library 2.5](https://rbdl.bitbucket.io/). Note that 2.6 version is not supported.
* [FlexBe](http://philserver.bplaced.net/fbe/) behavior framework and [`felexbe_general_states`](https://github.com/FlexBE/generic_flexbe_states).

### Installation from binary packages

We have repository with binary packages for Ubuntu 16.04, Debian 9 Stretch and Raspbian. 

Add apt keys
```
$ sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 5523BAEEB01FA116
$ wget -O - https://raw.githubusercontent.com/slavanap/ros-build/master/slavanap.key | sudo apt-key add -
```

Add ROS and Sweetie Bot repositories
```
$ sudo -i
# echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
# echo "deb http://sweetie.bot/apt $(lsb_release -sc) main" > /etc/apt/sources.list.d/sweetie-bot.list
```

Install binary packages
```
$ sudo apt-get update
$ sudo apt-get install ros-lunar-sweetie-bot ros-lunar-sweetie-bot-base
```
Note this `ros-lunar-sweetie-bot-base` conflicts with installed ROS packages with OROCOS toolchain.
Sweetie Bot specific software is installed in `/opt/ros/sweetie_bot` directory. Launch corresponding `setup.bash` script to setup ROS environment.

After installation you may want to install `sweetie_bot_sounds`, `sweetie_bot_proto2_movements` and `sweetie_bot_flexbe_behaviors` in ROS overlay in your home directory to modify them and create new behaviors.

### Installation from sources

Your may compile Sweetie Bot manually in ROS overlay. This method does not conflicts with installation from binary packages due to ROS overlay mechanism.
Assume all build requirements are satisfied (you may install `ros-lunar-sweetie-base` package on build them manually).

```
mkdir -p ~/ros/sweetie_bot/src
cd ~/ros/sweetie_bot/src
git clone git@gitlab.com:sweetie-bot/sweetie_bot_sounds.git
git clone git@gitlab.com:sweetie-bot/sweetie_bot_proto2_movements.git
git clone git@gitlab.com:sweetie-bot/sweetie_bot_flexbe_behaviors.git
git clone https://github.com/lucasw/rviz_textured_quads.git
cd ~/ros/sweetie_bot
source /opt/ros/sweetie_bot/setup.bash
catkin_make
```

### Installing from source

Use this [repository](https://github.com/slavanap/ros-build) and this Docker images [here](https://hub.docker.com/r/slavanap/ros-build/tags/).

### Build status

#### Base package

Platform        | Status
----------------|--------------
Desktop         | [![Build Status](https://travis-ci.org/slavanap/ros-build.svg?branch=master)](https://travis-ci.org/slavanap/ros-build)
Raspberry Pi 3  | [![Build Status](https://travis-ci.org/slavanap/ros-build.svg?branch=rpi3)](https://travis-ci.org/slavanap/ros-build/branches)

#### Main package

[![Build Status](https://gitlab.com/sweetie-bot/sweetie_bot/badges/devel/build.svg)](https://gitlab.com/sweetie-bot/sweetie_bot/pipelines)

