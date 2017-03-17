#include "joint_trajectory_data.h"

using namespace std;

namespace sweetie_bot {
namespace interface {

JointTrajectoryData::JointTrajectoryData(const FollowJointTrajectoryGoal& follow_joint_trajectory_goal) :
	follow_joint_trajectory_goal_(follow_joint_trajectory_goal),
	joint_names_(follow_joint_trajectory_goal.trajectory.joint_names)
{
	loadFromMsg(follow_joint_trajectory_goal);
}

bool JointTrajectoryData::loadFromMsg(const FollowJointTrajectoryGoal& follow_joint_trajectory_goal)
{
  follow_joint_trajectory_goal_ = follow_joint_trajectory_goal;
  joint_names_ = follow_joint_trajectory_goal.trajectory.joint_names;
	ROS_INFO("joint_names_.size=%lu", joint_names_.size());
	tolerances_.clear();
	for(int i = 0; i != joint_names_.size(); ++i) {
		tolerances_[joint_names_[i]].path_tolerance.name = joint_names_[i];
		//ROS_INFO("?%lu", follow_joint_trajectory_goal.path_tolerance.size());
		for(int t = 0; t != follow_joint_trajectory_goal.path_tolerance.size(); ++t)
		{
		  if(follow_joint_trajectory_goal.path_tolerance[t].name == joint_names_[i]){
		    tolerances_[joint_names_[i]].path_tolerance = follow_joint_trajectory_goal.path_tolerance[t];
		    //ROS_INFO("!%f %f", follow_joint_trajectory_goal.path_tolerance[t].position, tolerances_[joint_names_[i]].path_tolerance.position);
		    break;
		  }
		}
		ROS_INFO("%f", tolerances_[joint_names_[i]].path_tolerance.position);

		for(int t = 0; t != follow_joint_trajectory_goal.goal_tolerance.size(); ++t)
		{
		  if(follow_joint_trajectory_goal.goal_tolerance[t].name == joint_names_[i]){
		    tolerances_[joint_names_[i]].goal_tolerance = follow_joint_trajectory_goal.goal_tolerance[t];
		    //ROS_INFO("!%f %f", follow_joint_trajectory_goal.goal_tolerance[t].position, tolerances_[joint_names_[i]].goal_tolerance.position);
		    break;
		  }
		}
		ROS_INFO("%f", tolerances_[joint_names_[i]].goal_tolerance.position);

/*
		tolerances_[joint_names_[i]].path_tolerance = follow_joint_trajectory_goal.path_tolerance[i];
		tolerances_[joint_names_[i]].goal_tolerance = follow_joint_trajectory_goal.goal_tolerance[i];
		ROS_INFO("%s", joint_names_[i].c_str());
		//ROS_INFO_STREAM("\n" << tolerances_[joint_names_[i]].path_tolerance.name.c_str());
*/
	}
	ROS_INFO("%lu", tolerances_.size());
}

bool JointTrajectoryData::addPoint(const JointState& msg, double time_from_start)
{
  ROS_INFO_STREAM("\n" << msg);
  trajectory_msgs::JointTrajectoryPoint traj;
  for(auto &name: joint_names_){
    for (int i = 0; i != msg.name.size(); ++i) {
      if(msg.name[i] == name) {
        traj.positions.push_back(msg.position[i]);
        ROS_INFO("%s=%f", name.c_str(), msg.position[i]);
      }
    }
  }
  traj.time_from_start.fromSec(time_from_start);
  follow_joint_trajectory_goal_.trajectory.points.push_back(traj);
  //ROS_INFO_STREAM("/n" << follow_joint_trajectory_goal_);
  return true;
}

bool JointTrajectoryData::addJoint(const string name)
{
  follow_joint_trajectory_goal_.trajectory.joint_names.push_back(name);
  return true;
}

bool JointTrajectoryData::removeJoint(const string name)
{
  auto n = find(follow_joint_trajectory_goal_.trajectory.joint_names.begin(), follow_joint_trajectory_goal_.trajectory.joint_names.end(), name);
  if(n != follow_joint_trajectory_goal_.trajectory.joint_names.end())
    follow_joint_trajectory_goal_.trajectory.joint_names.erase(n);
  ROS_INFO("%lu", follow_joint_trajectory_goal_.trajectory.joint_names.size());
  return true;
}

bool JointTrajectoryData::removeJoint(const int row)
{
  follow_joint_trajectory_goal_.trajectory.joint_names.erase(follow_joint_trajectory_goal_.trajectory.joint_names.begin()+row);
  ROS_INFO("%lu", follow_joint_trajectory_goal_.trajectory.joint_names.size());
  return true;
}



}
}
