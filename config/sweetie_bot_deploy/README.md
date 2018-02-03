SWEETIE BOT DEPLOYMENT PACKAGE
======================================

This package contains ROS launch files and OROCOS lua deployment scripts necessary to start SweetieBot robot.  
It does not contain robot-specific configuration parameters. They are loaded from separate configuration package.
It is the part of [Sweetie Bot project](sweetiebot.net). Full documentation is available in Russian [here](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/deployment).

OROCOS components are started by hierarchy of lua scripts. Components' properties are loaded from ROS Parameter Server.
Mapping from components names to parameters names is quite direct. Part of configuration is stored in OROCOS .cpf files. 
To start ROS nodes roslaunch files are used.

### Scripts

* `store` and `store-joint-trajectories` (moved to robot-specific package) are used to load and save movements from/to Parameter Server to/from json files.
* `sweetie-bot-core` rttlua envelop to launch SweetieBot OROCOS components.

### Launch files

High-level launch files takes following parameters:

* `run_real` (boolean) --- Run on real robot. Try to launch robot components on `pi@sweetiebot`. Starts hardware interface components. (Default: false).
* `host` (boolean) --- Launch components which meant to be run on host side (default: true).
* `robot` (boolean) --- Launch components which meant to be run on robot side (default: false).
* `robot_name` (string) --- Robot-specific configuration packages prefix (e.g. set `sweetie_bot_proto2` for `sweetie_bot_proto2_description`, `sweetie_bot_proto2_moveit_config`, `sweetie_bot_proto2_deploy` packages). (Default: `sweetie_bot_proto2`).
* `robot_profile` (string) --- Use configuration profile (`load_param.launch`, `*.cpf`, `*.log4cpp` files) located in `package:<robot_name>_deploy/<robot_profile>`. (Default: `default`).

`robot_name` and `robot_profile` parameters allows to run different robots with different configuration.


The most important launch files:

* `load_param.launch` --- load parameters to ROS Parameter Server.
* `joint_space_control.launch` --- robot deployment script. It starts basic motion control configuration.
* `flexbe_control.launch` --- robot deployment script. It starts basic motion control configuration and high level control nodes (eyes and voice).
* `flexbe.launch` --- FlexBe core and user interface module. Starts GUI to control robot high-level behavior.
* `joint_trajectory_editor.launch` --- start GUI tool to create new movements.
For more information see description and parameter documentation inside files.

### Configuration profile

Typical robot profile directory contains following files:

* `load_param.launch` --- load parameters to ROS Parameter Server. It must set `robot_description` and `robot_description_dynamics` parameters. (The last is used by `dynamics_inv` component).
* `logger.log4cpp` --- logger reporting level configuration.
* `controller.yaml` --- controller parameters.
* `kinematic_chains.cpf` --- kinematic chains description.
* `sweetie_bot_servos.cpf` --- servos hardware ID mapping.
* `kinematics_inv_joint_limits.cpf` --- joint limits for `kinematics_inv_trac_ik` component.
* `herkulex_feedback.yaml` --- hardware interface configuration.
* `motion.yaml` --- motion core parameters.
Some parameters has special meaning: `services` is list of OROCOS subservices to be loaded into componet, 
`priority` is linux RT priority.

See `sweetie_bot_proto2_deploy` for example.

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


