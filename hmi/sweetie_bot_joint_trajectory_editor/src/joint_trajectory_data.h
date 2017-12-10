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

class JointTrajectoryData
{

	public:
		typedef sensor_msgs::JointState JointState;
		typedef control_msgs::FollowJointTrajectoryGoal FollowJointTrajectoryGoal;
		typedef control_msgs::JointTolerance JointTolerance;
		typedef trajectory_msgs::JointTrajectory JointTrajectory;
		typedef trajectory_msgs::JointTrajectoryPoint JointTrajectoryPoint;

	public:
		struct Joint {
			std::string name;
			double path_tolerance;
			double goal_tolerance;
			unsigned int index;

			bool operator<(const Joint& that) const { return name < that.name; }
			bool operator<(const std::string& that) const { return name < that; }
			bool operator==(const Joint& that) const { return name == that.name; }
			bool operator==(const std::string& that) const { return name == that; }
		};

		struct TrajectoryPoint {
			std::vector<double> positions;
			double time_from_start;

			bool operator<(const TrajectoryPoint& that) const { return time_from_start < that.time_from_start; }
			bool operator<(double that) const { return time_from_start < that; }
		};

	protected:
		// Trajectory Storage
		std::vector<Joint> joints_;
		std::vector<TrajectoryPoint> trajectory_points_;
		double goal_time_tolerance_;

	public:
		
		JointTrajectoryData() : goal_time_tolerance_(0.0) {};

		void loadFromMsg(const FollowJointTrajectoryGoal& msg); /**< Load FollowJointTrajectoryGoal message in the table.  */
		control_msgs::FollowJointTrajectoryGoal getTrajectoryMsg(bool reverse = false, double scale = 1.0); /**< Construct FollowJointTrajectoryGoal message. */
		void clear(); /**< Clear trajectory points and joints tables. */

		unsigned int jointCount() { return joints_.size(); }
		bool addJoint(const std::string& name, double path_tolerance = 0.0, double goal_tolerance = 0.0);
		const Joint& getJoint(unsigned int index) { return joints_.at(index); }
		void removeJoint(unsigned int index);
		int getJointIndex(const std::string& name);

		unsigned int pointCount() { return trajectory_points_.size(); }
		void addPoint(const TrajectoryPoint& point);
		void addPointMsg(const sensor_msgs::JointState& msg, double time_from_start);
		const TrajectoryPoint& getPoint(unsigned int index) { return trajectory_points_.at(index); }
		sensor_msgs::JointState getPointMsg(unsigned int index);
		void setPointTimeFromStart(unsigned int index, double time_from_start);
		void removePoint(unsigned int index);

		void setJointPathTolerance(unsigned int index, double tolerance) { joints_.at(index).path_tolerance = tolerance; }
		void setPathTolerance(double tolerance);
		void setJointGoalTolerance(unsigned int index, double tolerance) { joints_.at(index).goal_tolerance = tolerance; }
		void setGoalTolerance(double tolerance);

		double getGoalTimeTolerance() { return goal_time_tolerance_; }
		void setGoalTimeTolerance(double sec) { goal_time_tolerance_ = sec; }
};

} // namespace interface
} // namespace sweetie_bot

#endif // JOINT_TRAJECTORY_POINT_LIST_H
