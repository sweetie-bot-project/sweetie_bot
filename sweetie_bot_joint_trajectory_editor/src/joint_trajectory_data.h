#ifndef JOINT_TRAJECTORY_POINT_LIST_H
#define JOINT_TRAJECTORY_POINT_LIST_H

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
//#include <sweetie_bot_orocos_misc/joint_state_check.hpp>
#include <unordered_map>
#include <boost/range/adaptor/reversed.hpp>

namespace sweetie_bot {
namespace interface {

/*
FollowJointTrajectoryGoal:

trajectory:
  header: {secu: 0, stamp: 0.0, frame_id: ""}
  joint_names:
  - 'joint11'
  points:
  - positions: [0.0]
    velocities: [0.0]
    accelerations: [0.0]
    effort: [0.0]
    time_from_start: {secs: 0, nsecs: 0}
path_tolerance:
- {name: 'joint11', position: 0.1, velocity: 0.1, acceleration: 0.1}
goal_tolerance:
- {name: 'joint11', position: 0.1, velocity: 0.1, acceleration: 0.1}
goal_time_tolerance: {secs: 0, nsecs: 0}
*/

class JointTrajectoryData
{

	public:
		typedef sensor_msgs::JointState JointState;
		typedef control_msgs::FollowJointTrajectoryGoal FollowJointTrajectoryGoal;
		typedef control_msgs::JointTolerance JointTolerance;
		typedef trajectory_msgs::JointTrajectory JointTrajectory;
		typedef trajectory_msgs::JointTrajectoryPoint JointTrajectoryPoint;
		
		JointTrajectoryData(const FollowJointTrajectoryGoal& follow_joint_trajectory_goal);

		//std::vector<std::string> joint_names_;
		bool loadFromMsg(const FollowJointTrajectoryGoal& msg);
		void clear();
		FollowJointTrajectoryGoal& getTrajectoryMsg(bool reverse = false);

		bool addJoint(const std::string name, double path_tolerance = 0.0, double goal_tolerance = 0.0);
		bool removeJoint(const std::string name);
		int jointCount();
		std::string getJointName(int index);

		int addPoint(const sensor_msgs::JointState& msg, double time_from_start);
		sensor_msgs::JointState& getPoint(int index);
		double getPointTimeFromStart(int index);
		bool setPointTimeFromStart(int index, double time_from_start);
		bool removePoint(int index);
		int pointCount();

		double getPathTolerance(const std::string name);
		bool setPathTolerance(const std::string name, double path_tolerance);
		double getGoalTolerance(const std::string name);
		bool setGoalTolerance(const std::string name, double goal_tolerance);
		double getGoalTimeTolerance();
		void setGoalTimeTolerance(const double sec);
	protected:
		FollowJointTrajectoryGoal follow_joint_trajectory_goal_;
        sensor_msgs::JointState joint_state_;

		struct Tolerance {
			double position;
			double velocity;
			double acceleration;
		};
/*
		struct GoalPathTolerance {
			Tolerance path_tolerance;
			Tolerance goal_tolerance;
		}; */
		std::vector<std::string> joint_names_;
		std::unordered_map<std::string, Tolerance> path_tolerances_;
		std::unordered_map<std::string, Tolerance> goal_tolerances_;

		struct TrajectoryPoint {
			std::vector<double> position;
			std::vector<double> velocity;
			std::vector<double> acceleration;
			std::vector<double> effort;
			ros::Duration time_from_start;
		};
		std::vector<TrajectoryPoint> trajectory_point_;

		//double goal_time_;
		//std::vector<double> path_tolerance_; 
		//std::vector<double> goal_tolerance_;
		ros::Duration goal_time_tolerance_;
};

} // namespace interface
} // namespace sweetie_bot

#endif // JOINT_TRAJECTORY_POINT_LIST_H
