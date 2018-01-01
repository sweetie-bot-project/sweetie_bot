SWEETIE BOT DEPLOYMENT PACKAGE
======================================

This package contains ROS launch files and OROCOS lua deployment scripts necessary to start SweetieBot robot.  
It is the part of [Sweetie Bot project](sweetiebot.net). Full documentation is available in Russian [here](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/deployment).

OROCOS components are started by hierarchy of lua scripts. Components' properties are loaded from ROS Parameter Server.
Mapping from components names to parameters names is quite direct. Part of configuration is stored in OROCOS .cpf files. 

To start ROS nodes roslaunch files are used.

Configuration files:
* `controller.yaml` --- controller parameters.
* `kinematic_chains.cpf` --- kinematic chains description.
* `sweetie_bot_servos.cpf` --- servos hardware ID mapping.
* `herkulex_feedback.yaml` --- hardware interface configuration.
* `motion.yaml` --- motion core parameters.
Some parameters has special meaning: `services` is list of OROCOS subservices to be loaded into componet, 
`priority` is linux RT priority.

Scripts:
* `store` and `store_joint_trajectories` are used to load and save movements from/to Parameter Server to/from json files.
* `sweetie-bot-core` rttlua envelop to launch SweetieBot OROCOS components.

The most important launch files ans scripts:
* `joint_space_param.launch` --- load parameters to ROS Parameter Server from .yaml files.
* `joint_space_control.launch` --- robot deployment script. It starts basic motion control configuration.
* `flexbe_control.launch` --- robot deployment script. It starts basic motion control configuration and high level control nodes (eyes and voice).
* `flexbe.launch` --- FlexBe core and user interface module. Starts GUI to control robot high-level behavior.
* `joint_trajectory_editor.launch` --- start GUI tool to create new movements.
For more information see description and parameter documentation inside files.

### Usage example

Start simulated robot, rviz and `joint_trajectory_editor`:

    roslaunch sweetie_bo_deploy joint_space_control.launch
	roslaunch sweetie_bo_deploy joint_trajectory_editor.launch

Start real robot, rviz. It is assumed that roscore can be accessed form SweetieBot.

	host $ roslaunch sweetie_bo_deploy joint_space_param.launch
	sweetie $ roslaunch sweetie_bo_deploy joint_space_control.launch run_real:=true host:=false robot:=true 
	host $ roslaunch sweetie_bo_deploy joint_space_control.launch run_real:=true host:=true robot:=false

Start real robot with high-level control subsystem. 

	host $ roslaunch sweetie_bo_deploy joint_space_param.launch
	sweetie $ roslaunch sweetie_bo_deploy flexbe_control.launch run_real:=true host:=false robot:=true 
	host $ roslaunch sweetie_bo_deploy flexbe_control.launch run_real:=true run_moveit:=true host:=true robot:=false
	host $ roslaunch sweetie_bo_deploy flexbe.launch


