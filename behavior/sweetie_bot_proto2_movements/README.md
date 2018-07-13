Sweetie Bot Proto2 movements
============================

This package is part of [Sweetie Bot project](sweetiebot.net). 

This package contains `control_msgs::FollowJointTrajectoryGoal` messages with movement trajectories 
stored in JSON format (`rospy_message_convertor`). They are meant to be loaded to ROS Parameter Server 
in serialized binary format before use. It is usually done automatically by `sweetie_bot_deploy` package 
deployment scripts.

Use `store` script from `sweetie_bot_deploy` to save message created by `sweetie_bot_trajectory_editor` in file 
form parameter server or to load your own message to parameter server.

Note, that joints with `support/<kinematic_chain>` prefix has special meaning: they are defining contacts for corresponding
kinematics chains.
