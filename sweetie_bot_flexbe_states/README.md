Sweetie Bot FlexBe states
============================

This package contains Sweetie Bot [FlexBe](http://philserver.bplaced.net/fbe/index.php) state.
It is the part of [Sweetie Bot project](sweetiebot.net). 

Use `flexbe_app`/`flexbe_widget` build-in documentation system to browse full API information.

Defined states:

* `ExecuteJointTrajectoryState` (`AnimationStoredJointTrajectoryState`) --- pass stored in ROS parameter `control_msgs::FollowJointTrajectoryGoal` to the action server to
    perform corresponding motion.
* `LeapMotionMonitor` --- listen to Leap Motion topic and detect hand presence.
* `MoveitToPose`, `MoveToPose2` --- move MoveIt! move group to desired pose.
* `PublisherState` --- publish ROS message.
* `SweetieBotRandHeadMovements` --- move head and eyes randomly.
* `SetBoolState` --- call `std_srvs::SetBool`  ROS service.
* `SweetieBotCompoundAction` --- perform a few actions in one (`ExecuteJointTrajectoryState` and `TextCommandState`)
* `SweetieBotFollowHeadPoseSmart` --- Sweetie Bot follows object (`geometry_msgs::PoseStamped` topic) with head.
* `TextCommandState` --- send `TextCommand`.
* `WaitForMessageState` --- wait for message on topic which satisfies given predicate.

