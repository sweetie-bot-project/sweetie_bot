SWEETIE BOT DEPLOYMENT PACKAGE
======================================

This package contains ROS launch files and OROCOS lua deployment scripts necessary to start SweetieBot robot.  
It does not contain robot-specific configuration parameters. They are loaded from separate configuration package.
It is the part of [Sweetie Bot project](sweetiebot.net). Full documentation is available in Russian [here](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/deployment).

ROS nodes are started in standard way by roslaunch files.

[OROCOS](http://orocos.org) components are started by hierarchy of `lua` scripts. Components' properties are loaded from ROS Parameter Server.
Mapping from ROS parameters to component properties is quite direct: parameters' namespace has the same name as OROCOS component.  
Part of configuration is stored in OROCOS `.cpf` files but they also may be loaded via ROS parameter server.

### Scripts

* `store` and `store-joint-trajectories` (moved to robot-specific package) are used to load and save movements from/to Parameter Server to/from json files.
* `sweetie-bot-core` [rttlua](http://www.orocos.org/wiki/orocos/toolchain/luacookbook) envelop to launch SweetieBot OROCOS components.

### Namespace structure

Sweetie Bot software is using following namespace structure. Namespaces are marked by caps, the topics and parameters have `topic:` and `param:` prefixes.

    /
    |- param: robot_description --- URDF model
    |- param: robot_description_semantic --- MoveIt configuration
    |
    |- CONF_FILE --- OROCOS configuration files parameters (.cpf, .log4cpp). 
    |- JOINT_TRAJECTORY --- parameters with joint trajectories.
    |
    |- MOTION --- OROCOS rt control node and configuration. Note that ROS see all OROCOS component as a one node with 'motion' name.
    |    |             This node is lauched on the robot on-board computer.
    |    |
    |    |- ROBOT_MODEL --- OROCOS robot semantic model (kinematics chains, contacts). It is loaded from `kinematic_chains.cpf`.
    |    |
    |    |- CONTROLLER --- motion controllers
    |    |    |- <controller_name> --- controller parameters and its topics.
    |    |    ...
    |    |- HERKULEX --- servo control groups
    |    |    |- LEG12 
    |    |    |    |- array --- provides configuration and debug interface for servos.
    |    |    |    |- driver
    |    |    |    \- sched 
    |    |    ...
    |    |- aggregator_ref --- provides robot reference pose in joint space 
    |    |- kinematics_fwd --- provides robot limbs positons in cartesian space
    |    |- odometry_ref --- base_link odometry
    |    |- dynamics_inv --- robot balace and servo efforts calculation
    |    |- aggregator_real --- provides robot real pose in joint space 
    |    ...
    |
    |- topic: joint_states, tf
    |- topic: control --- high level TextCommands for eyes emotions change and voice control.
    |- topic: move_group, trajectory_execution, pick, place --- MoveIt! topics.
    |
    |- robot_state_publisher --- /tf publication
    |- move_group --- MoveIt! control node, coresponding topics and parameters are loaded in / namespace.
    |- behavior --- FlexBe behavior engine
    |- eye_left, eye_right --- hardware dependent eyes node. They are lauched on the robot on-board computer.
    |
    |- VOICE 
    |   |- voice_node
    |   \- sound_play
    |
    |- FLEXBE --- flexbe operator interface nodes.
    |
    \- HMI --- visualization and opertor interfce
        |- rviz 
        \- pose_markers

#### OROCOS and ROS integration

[`rtt_ros_integration` package](https://github.com/orocos/rtt_ros_integration) allows to map ROS topics on OROCOS ports transparently. Also it gives OROCOS components access to ROS parameter server. 
ROS sees all OROCOS subsystem as one node with name `motion`. OROCOS components advertise their interface in corresponding namespace. For example, `controller/stance` component provides 
actionlib server under `motion/controller/stance` to activate it and listen to the `motion/controller/in_base_ref` topic. The controller get properties from `motion/controller/stance` namespace.

Some parameters of OROCOS components has special meaning: 
 * `period` is set to the main timer period (control cycle duration) which defined in `timer` component configuration.
 * `services` is list of OROCOS subservices to be loaded into component, 
 * `priority` is linux RT priority.

You can access and view ROS nodes, topics and parameters using standard ROS tools. OROCOS components interface can be access via [`rttlua` console](http://www.orocos.org/wiki/orocos/toolchain/luacookbook).
It is started with OROCOS control node. Note that OROCOS component interface is self-documented you can browse `controller/stance` interface using command `= controller.stance`. 

### Launch files

High-level launch files takes following parameters:

* `run_real` (boolean) --- Run on real robot. Try to launch robot components on `pi@sweetiebot`. Starts hardware interface components. (Default: false).
* `host` (boolean) --- Launch components which meant to be run on host side (default: true).
* `robot` (boolean) --- Launch components which meant to be run on robot side (default: false).
* `robot_name` (string) --- Robot-specific configuration packages prefix (e.g. set `sweetie_bot_proto2` for `sweetie_bot_proto2_description`, `sweetie_bot_proto2_moveit_config`, `sweetie_bot_proto2_deploy` packages). (Default: `sweetie_bot_proto2`).
* `robot_profile` (string) --- Use configuration profile (`load_param.launch`, `robot_module.launch`, `*.cpf`, `*.log4cpp` files) located in `package:<robot_name>_deploy/<robot_profile>`. (Default: `default`).
* `robot_ns` (string) --- launch all nodes in given namespace.

`robot_name` and `robot_profile` parameters allows to run different robots with different configuration.


The most important launch files:

* `load_param.launch` --- load parameters to ROS Parameter Server.
* `joint_space_control.launch` --- robot deployment script. It starts basic motion control configuration. 
* `flexbe_control.launch` --- robot deployment script. It starts basic motion control configuration and high level control nodes (eyes and voice).
* `flexbe.launch` --- FlexBe core and user interface module. Starts GUI to control robot high-level behavior.
* `joint_trajectory_editor.launch` --- start GUI tool to create new movements.
For more information see description and parameter documentation inside files.

`joint_space_control.launch` and `flexbe_control.launch` starts OROCOS node which provides `rttlua` console.

### Configuration profile

Typical robot profile directory contains following files:

* `load_param.launch` --- load parameters to ROS Parameter Server. It must set `robot_description` and `robot_description_dynamics` parameters. (The last is used by `dynamics_inv` component).
* `robot_module.launch` --- launch hardware-specific eyes control.
* `logger.log4cpp` --- logger reporting level configuration.
* `controller.yaml` --- controller parameters (OROCOS).
* `kinematic_chains.cpf` --- kinematic chains description.
* `herkulex_servos_*.cpf` --- servos group definitions and hardware configuration and ID mapping.
* `kinematics_inv_joint_limits.cpf` --- joint limits for `kinematics_inv_trac_ik` component.
* `herkulex_feedback.yaml` --- hardware interface configuration.
* `motion.yaml` --- motion core parameters.
* `hmi.yaml` --- visualization configuration.

See `sweetie_bot_proto2_deploy` for example.

### Host and robot on-board computer relations 

* All parameters (including .cpf files, and trajectories) can be loaded into ROS Parameter Server. So any changes performed on host affects on-board subsystem.
    Note that the actual list of  configuration files loaded into pareamter server is defined in `load_param.launch`
* `.launch` scripts support `machine` tag so if configuration is right host is able to start all necessary nodes on on-board computer via ssh.
    But note that even remote deplyment procedure uses local `.lua` scritpts. If deployment is not remote then local launch file is used.
    So launch files must be consistent beetwen the robot and the host.
* `.lua` scripts are always local. So they must be consistent between host and robot.


### Usage example

All usage examples assumes that `sweetie_bot_proto2_deploy` package is present on ROS path.

Start simulated robot, rviz and `joint_trajectory_editor`:

    roslaunch sweetie_bot_deploy joint_space_control.launch
	roslaunch sweetie_bot_deploy joint_trajectory_editor.launch

Save trajectories to .json files.

    rosrun sweetie_bot_proto2_deploy store-joint-trajectories save

Start real robot, rviz. It is assumed that roscore can be accessed form SweetieBot.

	host $ roslaunch sweetie_bot_deploy joint_space_param.launch
	sweetie $ roslaunch sweetie_bot_deploy joint_space_control.launch run_real:=true host:=false robot:=true 
	host $ roslaunch sweetie_bot_deploy joint_space_control.launch run_real:=true host:=true robot:=false

Start real robot with high-level control subsystem. 

	host $ roslaunch sweetie_bot_deploy joint_space_param.launch
	sweetie $ roslaunch sweetie_bot_deploy flexbe_control.launch run_real:=true host:=false robot:=true 
	host $ roslaunch sweetie_bot_deploy flexbe_control.launch run_real:=true run_moveit:=true host:=true robot:=false
	host $ roslaunch sweetie_bot_deploy flexbe.launch


