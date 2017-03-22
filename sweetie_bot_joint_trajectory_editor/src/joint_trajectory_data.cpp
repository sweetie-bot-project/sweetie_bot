#include "joint_trajectory_data.h"

using namespace std;

namespace sweetie_bot {
namespace interface {

JointTrajectoryData::JointTrajectoryData(const FollowJointTrajectoryGoal& follow_joint_trajectory_goal) :
	//follow_joint_trajectory_goal_(follow_joint_trajectory_goal),
	joint_names_(follow_joint_trajectory_goal.trajectory.joint_names)
{
	if(!loadFromMsg(follow_joint_trajectory_goal)) ROS_WARN("loadFromMsg == false");
}

control_msgs::FollowJointTrajectoryGoal& JointTrajectoryData::getTrajectoryMsg()
{
	FollowJointTrajectoryGoal follow_joint_trajectory_goal;
	follow_joint_trajectory_goal.trajectory.header.stamp = ros::Time::now();
	follow_joint_trajectory_goal.trajectory.joint_names = joint_names_;

	JointTrajectoryPoint traj;
	for(auto &traj: trajectory_point_)
	{
		JointTrajectoryPoint new_point;
		new_point.positions      = traj.position;
		new_point.velocities     = traj.velocity;
		new_point.accelerations  = traj.acceleration;
		new_point.effort         = traj.effort;
		new_point.time_from_start= traj.time_from_start;
		follow_joint_trajectory_goal.trajectory.points.push_back( new_point );
	}
	if(path_tolerances_.size() > 0)
	{
		for(auto &joint: joint_names_)
		{
			JointTolerance new_tol;
			new_tol.name = joint;
			new_tol.position = path_tolerances_[joint].position;
			new_tol.velocity = path_tolerances_[joint].velocity;
			new_tol.acceleration = path_tolerances_[joint].acceleration;
			follow_joint_trajectory_goal.path_tolerance.push_back( new_tol );
		}
	}
	if(goal_tolerances_.size() > 0)
	{
		for(auto &joint: joint_names_)
		{
			JointTolerance new_tol;
			new_tol.name = joint;
			new_tol.position = goal_tolerances_[joint].position;
			new_tol.velocity = goal_tolerances_[joint].velocity;
			new_tol.acceleration = goal_tolerances_[joint].acceleration;
			follow_joint_trajectory_goal.goal_tolerance.push_back( new_tol );
		}
	}
	follow_joint_trajectory_goal.goal_time_tolerance = goal_time_tolerance_;

  follow_joint_trajectory_goal_ = follow_joint_trajectory_goal;
  return follow_joint_trajectory_goal_;
}

bool JointTrajectoryData::loadFromMsg(const FollowJointTrajectoryGoal& msg)
{
	ROS_INFO_STREAM("\n" << msg);
	clear();
	joint_names_ = msg.trajectory.joint_names;

	std::sort(joint_names_.begin(), joint_names_.end());
	if(std::unique(joint_names_.begin(), joint_names_.end()) != joint_names_.end()) {
		ROS_WARN("not unique joints!");
		clear();
		return false;
	}
	joint_names_ = msg.trajectory.joint_names;

	follow_joint_trajectory_goal_ = msg;
	goal_time_tolerance_ = msg.goal_time_tolerance;

	// iterating over joint names
    // itrating over point of trajectory
	for(int p = 0; p < msg.trajectory.points.size(); ++p)
    {
        TrajectoryPoint point;
        point.time_from_start = msg.trajectory.points[p].time_from_start;
        //point.names = joint_names_;

        if(msg.trajectory.points[p].positions.size() > 0){
    	    if (msg.trajectory.points[p].positions.size() != joint_names_.size()){
				ROS_WARN("msg.trajectory.points[p].positions.size(%lu) != joint_names.size(%lu)", msg.trajectory.points[p].positions.size(), joint_names_.size());
				clear();
				return false;
			}
            point.position = msg.trajectory.points[p].positions;
        }
        if(msg.trajectory.points[p].velocities.size() > 0){
            if (msg.trajectory.points[p].velocities.size() != joint_names_.size()){
				ROS_WARN("msg.trajectory.points[p].velocities.size() != joint_names.size()");
				clear();
				return false;
			}
            point.velocity = msg.trajectory.points[p].velocities;
        }
        if(msg.trajectory.points[p].accelerations.size() > 0){
	        if (msg.trajectory.points[p].accelerations.size() != joint_names_.size()){
				ROS_WARN("msg.trajectory.points[p].accelerations.size() != joint_names.size()");
				clear();
				return false;
			}

            point.acceleration = msg.trajectory.points[p].accelerations;
        }
        if(msg.trajectory.points[p].effort.size() > 0){
            if (msg.trajectory.points[p].effort.size() != joint_names_.size()){
				ROS_WARN("msg.trajectory.points[p].effort.size() != joint_names.size()");
				clear();
				return false;
			}
            point.effort = msg.trajectory.points[p].effort;
        }
        trajectory_point_.push_back(point);
    }

    if(msg.path_tolerance.size() > 0)
    {
		for(int j = 0; j < joint_names_.size(); ++j)
		{
	        for(int t=0; t < msg.path_tolerance.size(); ++t)
		    {
				if(msg.path_tolerance[t].name == joint_names_[j])
	            {
		            path_tolerances_.insert({msg.path_tolerance[t].name,	{
			            msg.path_tolerance[t].position,
				        msg.path_tolerance[t].velocity,
					    msg.path_tolerance[t].acceleration
		            }});
					break;
				}
			}
		}
		if(path_tolerances_.size() != joint_names_.size()){
			ROS_WARN("path_tolerances_.size(%lu) != msg.trajectory.joint_names.size(%lu)", path_tolerances_.size(), joint_names_.size());
			clear();
			return false;
		}
    }

    if(msg.goal_tolerance.size() > 0)
    {
		for(int j = 0; j < joint_names_.size(); ++j)
		{
			for(int t=0; t < msg.goal_tolerance.size(); ++t)
			{
				if(msg.goal_tolerance[t].name == msg.trajectory.joint_names[j])
				{
					goal_tolerances_.insert({msg.goal_tolerance[t].name,	{
						msg.goal_tolerance[t].position,
		        		msg.goal_tolerance[t].velocity,
						msg.goal_tolerance[t].acceleration
					}});
					break;
	    	    }
		    }
		}
		if(goal_tolerances_.size() != joint_names_.size()) {
			ROS_WARN("goal_tolerances_.size() != msg.trajectory.joint_names.size()");
			clear();
			return false;
		}
	}
}

void JointTrajectoryData::clear()
{
  joint_names_.clear();
  path_tolerances_.clear();
  goal_tolerances_.clear();
  trajectory_point_.clear();
  goal_time_tolerance_.fromSec( 0.0 );
}

int JointTrajectoryData::addPoint(const JointState& msg, double time_from_start)
{
	//if(!sweetie_bot::isValidJointStatePos(msg)) return -1;
	// prepare trajectory
	TrajectoryPoint new_point;
    for(auto &name: joint_names_){
        for (int i = 0; i < msg.name.size(); ++i) {
            if(msg.name[i] == name) {
                if(msg.position.size() > i)  new_point.position.push_back(msg.position[i]);
                if(msg.velocity.size() > i)  new_point.velocity.push_back(msg.velocity[i]);
                if(msg.effort.size() > i)    new_point.effort.push_back(msg.effort[i]);
                break;
            }
        }
    }

	if(new_point.position.size() != joint_names_.size()) return -1;
	if((new_point.velocity.size() > 0) and (new_point.velocity.size() != joint_names_.size())) return -1;
	if((new_point.effort.size() > 0) and (new_point.effort.size() != joint_names_.size())) return -1;

	new_point.time_from_start.fromSec(time_from_start);

  trajectory_point_.push_back(new_point);
  // return latst index
  return trajectory_point_.size()-1;
}

sensor_msgs::JointState& JointTrajectoryData::getPoint(int index)
{
    joint_state_.header.stamp = ros::Time::now();
	joint_state_.name = joint_names_;
	if(trajectory_point_.size() > index)
	{
		joint_state_.position = trajectory_point_[index].position;
		joint_state_.velocity = trajectory_point_[index].velocity;
		joint_state_.effort   = trajectory_point_[index].effort;
	}
	return joint_state_;
}

double JointTrajectoryData::getPointTimeFromStart(int index)
{
  if(index < trajectory_point_.size())
  {
    return trajectory_point_[index].time_from_start.toSec();
  }
  else
    return 0.0;
}
bool JointTrajectoryData::setPointTimeFromStart(int index, double time_from_start)
{
  if(index < trajectory_point_.size())
  {
    trajectory_point_[index].time_from_start.fromSec(time_from_start);
    return true;
  }  
  else
    return false;
}

bool JointTrajectoryData::removePoint(int index)
{
  if(index < trajectory_point_.size())
  {
    trajectory_point_.erase(trajectory_point_.begin()+index);
    return true;
  }
  else
    return false;
}

int JointTrajectoryData::pointCount()
{
  return trajectory_point_.size();
}

bool JointTrajectoryData::addJoint(const string name, double path_tolerance /* = 0.0 */, double goal_tolerance /* = 0.0 */)
{
	auto n = find(joint_names_.begin(), joint_names_.end(), name);
	if(n != joint_names_.end()) return false; // if found

	int n_joints = joint_names_.size();
	joint_names_.push_back(name);
    for(auto &point: trajectory_point_)
    {
      if(point.position.size() == n_joints)     point.position.push_back(0.0);
      if(point.velocity.size() == n_joints)    point.velocity.push_back(0.0);
      if(point.acceleration.size() == n_joints) point.acceleration.push_back(0.0);
      if(point.effort.size() == n_joints)        point.effort.push_back(0.0);
    }

	if(path_tolerances_.size() == n_joints)
		path_tolerances_.insert({name, {path_tolerance, 0.0, 0.0}});

	if(goal_tolerances_.size() == n_joints)
		goal_tolerances_.insert({name, {goal_tolerance, 0.0, 0.0}});

	return true;
}

bool JointTrajectoryData::removeJoint(const string name)
{
	auto n = find(joint_names_.begin(), joint_names_.end(), name);
	if(n == joint_names_.end()) return false; // if not found

	int j_pos = distance(joint_names_.begin(), n);

	// delete corresponding trajectory points
    for(auto &point: trajectory_point_)
    {
      if(point.position.size() > 0)     point.position.erase(point.position.begin() + j_pos);
      if(point.velocity.size() > 0)     point.velocity.erase(point.velocity.begin() + j_pos);
      if(point.acceleration.size() > 0) point.acceleration.erase(point.acceleration.begin() + j_pos);
      if(point.effort.size() > 0)       point.effort.erase(point.effort.begin() + j_pos);
    }

    joint_names_.erase(n);
	path_tolerances_.erase(name);
	goal_tolerances_.erase(name);
	return true;
}

int JointTrajectoryData::jointCount()
{
  return joint_names_.size();
}

std::string JointTrajectoryData::getJointName(int index)
{
  if(index < joint_names_.size())
    return joint_names_.at(index);
  else
    return "";
}

double JointTrajectoryData::getPathTolerance(const std::string name)
{
  auto n = find(joint_names_.begin(), joint_names_.end(), name);
  if(n == joint_names_.end()) return 0.0;
  return path_tolerances_[ name ].position;
}

bool JointTrajectoryData::setPathTolerance(const std::string name, double path_tolerance)
{
  auto n = find(joint_names_.begin(), joint_names_.end(), name);
  if(n == joint_names_.end()) return false;
  path_tolerances_[ name ].position = path_tolerance;
  return true;
}

double JointTrajectoryData::getGoalTolerance(const std::string name)
{
  auto n = find(joint_names_.begin(), joint_names_.end(), name);
  if(n == joint_names_.end()) return 0.0;
  return goal_tolerances_[ name ].position;
}

bool JointTrajectoryData::setGoalTolerance(const std::string name, double goal_tolerance)
{
  auto n = find(joint_names_.begin(), joint_names_.end(), name);
  if(n == joint_names_.end()) return false;
  goal_tolerances_[ name ].position = goal_tolerance;
  return true;
}

double JointTrajectoryData::getGoalTimeTolerance()
{
  return goal_time_tolerance_.toSec();
}

void JointTrajectoryData::setGoalTimeTolerance(const double sec)
{
  goal_time_tolerance_.fromSec(sec);
}

}
}
