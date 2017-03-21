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

control_msgs::FollowJointTrajectoryGoal& JointTrajectoryData::getTrajectoryMsg()
{
  return follow_joint_trajectory_goal_;
}

bool JointTrajectoryData::loadFromMsg(const FollowJointTrajectoryGoal& follow_joint_trajectory_goal)
{
  follow_joint_trajectory_goal_ = follow_joint_trajectory_goal;
  //ROS_INFO_STREAM("\n" << follow_joint_trajectory_goal);

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

int JointTrajectoryData::addPoint(const JointState& msg, double time_from_start)
{
  // prepare trajectory
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
  // return latst index
  return follow_joint_trajectory_goal_.trajectory.points.size()-1;
}

double JointTrajectoryData::getPointTimeFromStart(int index)
{
  if(index < follow_joint_trajectory_goal_.trajectory.points.size())
  {
    return follow_joint_trajectory_goal_.trajectory.points[index].time_from_start.toSec();
  }
  else
    return 0.0;
}
bool JointTrajectoryData::setPointTimeFromStart(int index, double time_from_start)
{
  if(index < follow_joint_trajectory_goal_.trajectory.points.size())
  {
    follow_joint_trajectory_goal_.trajectory.points[index].time_from_start.fromSec(index);
    return true;
  }  
  else
    return false;
}

bool JointTrajectoryData::removePoint(int index)
{
  if(index < follow_joint_trajectory_goal_.trajectory.points.size())
  {
    follow_joint_trajectory_goal_.trajectory.points.erase(follow_joint_trajectory_goal_.trajectory.points.begin()+index);
    return true;
  }
  else
    return false;
}

int JointTrajectoryData::pointCount()
{
  return follow_joint_trajectory_goal_.trajectory.points.size();
}

bool JointTrajectoryData::addJoint(const string name, double path_tolerance /* = 0.0 */, double goal_tolerance /* = 0.0 */)
{
  auto n = find(follow_joint_trajectory_goal_.trajectory.joint_names.begin(), follow_joint_trajectory_goal_.trajectory.joint_names.end(), name);
  if(n == follow_joint_trajectory_goal_.trajectory.joint_names.end()) // if not found
  {
    int p = follow_joint_trajectory_goal_.trajectory.joint_names.size();
    for(auto &point: follow_joint_trajectory_goal_.trajectory.points)
    {
      if(point.positions.size() == p)     point.positions.push_back(0.0);
      if(point.velocities.size() == p)    point.velocities.push_back(0.0);
      if(point.accelerations.size() == p) point.accelerations.push_back(0.0);
      if(point.effort.size() == p)        point.effort.push_back(0.0);
    }

    follow_joint_trajectory_goal_.trajectory.joint_names.push_back(name);

    control_msgs::JointTolerance tol;
    tol.name = name;
    tol.position = path_tolerance;

    follow_joint_trajectory_goal_.path_tolerance.push_back(tol);

    tol.position = goal_tolerance;

    follow_joint_trajectory_goal_.goal_tolerance.push_back(tol);
  }
  return true;
}

bool JointTrajectoryData::removeJoint(const string name)
{
  auto n = find(follow_joint_trajectory_goal_.trajectory.joint_names.begin(), follow_joint_trajectory_goal_.trajectory.joint_names.end(), name);
  if(n != follow_joint_trajectory_goal_.trajectory.joint_names.end())
  {
    follow_joint_trajectory_goal_.trajectory.joint_names.erase(n);

    int p = distance(follow_joint_trajectory_goal_.trajectory.joint_names.begin(), n);

    // delete corresponding trajectory points
    for(auto &point: follow_joint_trajectory_goal_.trajectory.points)
    {
      if(point.positions.size() > p)     point.positions.erase(point.positions.begin() + p);
      if(point.velocities.size() > p)    point.velocities.erase(point.velocities.begin() + p);
      if(point.accelerations.size() > p) point.accelerations.erase(point.accelerations.begin() + p);
      if(point.effort.size() > p)        point.effort.erase(point.effort.begin() + p);
    }

    int t = 0;
    for(auto &tol: follow_joint_trajectory_goal_.path_tolerance)
    {
      if(tol.name == name)
      {
	follow_joint_trajectory_goal_.path_tolerance.erase(follow_joint_trajectory_goal_.path_tolerance.begin() + t);
      }
      t++;
    }
    t = 0;
    for(auto &tol: follow_joint_trajectory_goal_.goal_tolerance)
    {
      if(tol.name == name)
      {
	follow_joint_trajectory_goal_.goal_tolerance.erase(follow_joint_trajectory_goal_.goal_tolerance.begin() + t);
      }
      t++;
    }
 
  }
  ROS_INFO_STREAM("\n" << follow_joint_trajectory_goal_);
  return true;
}

int JointTrajectoryData::jointCount()
{
  return follow_joint_trajectory_goal_.trajectory.joint_names.size();
}

std::string JointTrajectoryData::getJointName(int index)
{
  if(index < follow_joint_trajectory_goal_.trajectory.joint_names.size())
    return follow_joint_trajectory_goal_.trajectory.joint_names.at(index);
  else
    return "";
}

double JointTrajectoryData::getPathTolerance(const std::string name)
{
  auto n = find(follow_joint_trajectory_goal_.trajectory.joint_names.begin(), follow_joint_trajectory_goal_.trajectory.joint_names.end(), name);
  if(n != follow_joint_trajectory_goal_.trajectory.joint_names.end())
  {
    for(int t = 0; t != follow_joint_trajectory_goal_.path_tolerance.size(); ++t)
    {
      if(follow_joint_trajectory_goal_.path_tolerance[t].name == name)
        return follow_joint_trajectory_goal_.path_tolerance[t].position;
    }
  }
  return 0.0;
}

double JointTrajectoryData::getGoalTolerance(const std::string name)
{
  auto n = find(follow_joint_trajectory_goal_.trajectory.joint_names.begin(), follow_joint_trajectory_goal_.trajectory.joint_names.end(), name);
  if(n != follow_joint_trajectory_goal_.trajectory.joint_names.end())
  {
    for(int t = 0; t != follow_joint_trajectory_goal_.goal_tolerance.size(); ++t)
    {
      if(follow_joint_trajectory_goal_.goal_tolerance[t].name == name)
        return follow_joint_trajectory_goal_.goal_tolerance[t].position;
    }
  }
  return 0.0;
}

double JointTrajectoryData::getGoalTimeTolerance()
{
  return follow_joint_trajectory_goal_.goal_time_tolerance.toSec();
}

void JointTrajectoryData::setGoalTimeTolerance(const double sec)
{
  follow_joint_trajectory_goal_.goal_time_tolerance.fromSec(sec);
}

}
}
