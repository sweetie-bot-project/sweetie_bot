#include "joint_trajectory_data.h"

#include <cmath>
#include <functional>
#include <exception>

using namespace std;

namespace sweetie_bot {
namespace hmi {


unsigned int JointTrajectoryData::crc(const std::vector<double>& positions) 
{	
	size_t crc = 0;
	std::hash<double> hash_fn;
	for(auto it = positions.begin(); it != positions.end(); it++) crc ^= hash_fn(std::floor(*it * 100));
	return crc;
}

void JointTrajectoryData::loadFromMsg(const FollowJointTrajectoryGoal& msg)
{
	clear();
	// get joints names
	for(unsigned int i = 0; i < msg.trajectory.joint_names.size(); i++) {
		const std::string& name = msg.trajectory.joint_names[i];
		if (name.compare(0,8,"support/") == 0) {
			Support support;
			support.name = name.substr(8);
			support.index = i;
			// push it to supports list
			supports_.push_back(support);
		}
		else {	
			// add joint
			Joint joint;
			joint.name = msg.trajectory.joint_names[i];
			joint.path_tolerance = 0.0;
			joint.goal_tolerance = 0.0;
			joint.index = i;
			// push it to joint list
			joints_.push_back(joint);
		}
	}
	sort(joints_.begin(), joints_.end());
	sort(supports_.begin(), supports_.end());
	// check if all joints are unique
	if(std::unique(joints_.begin(), joints_.end()) != joints_.end()) {
		clear();
		throw std::invalid_argument("Joints are not unique.");
	}
	if(std::unique(supports_.begin(), supports_.end()) != supports_.end()) {
		clear();
		throw std::invalid_argument("Supports are not unique.");
	}

	// load trajectory points
	// because joints are rearanged by sort() we have to 
	trajectory_points_.resize(msg.trajectory.points.size());
	for(unsigned int k = 0; k < msg.trajectory.points.size(); ++k) {
		// get time
		trajectory_points_[k].time_from_start = msg.trajectory.points[k].time_from_start.toSec();
		// check number of positions 
		if (msg.trajectory.points[k].positions.size() != msg.trajectory.joint_names.size()) {
			clear(); 
			throw std::invalid_argument("Inconsistent number of joints in FollowJointTrajectoryGoal message: trajectory.joint_names.size() != trajectory.points[k].positions.size(), k = " + std::to_string(k));
		}
		// fill positions in proper order
		trajectory_points_[k].positions.resize(joints_.size());
		for(unsigned int i = 0; i < joints_.size(); i++) {
			unsigned int old_index = joints_[i].index;
			trajectory_points_[k].positions[i] = msg.trajectory.points[k].positions[old_index];
		}
		trajectory_points_[k].crc = crc(trajectory_points_[k].positions);
		// fill supports in proper oder
		trajectory_points_[k].supports.resize(supports_.size());
		for(unsigned int i = 0; i < supports_.size(); i++) {
			unsigned int old_index = supports_[i].index;
			trajectory_points_[k].supports[i] = msg.trajectory.points[k].positions[old_index];
		}
		// ignore velocities, accelerations and efforts
	}
	// fix joint induces
	for(unsigned int i = 0; i < joints_.size(); i++) joints_[i].index = i;
	for(unsigned int i = 0; i < supports_.size(); i++) supports_[i].index = i;

	// load path and goal tolerance (if present) 
	// default values (zeros) are already present in joints_ structures.
	for(auto it = msg.path_tolerance.begin(); it != msg.path_tolerance.end(); it++) {
		// find joint name in joints_ using binary search
		auto joint = lower_bound(joints_.begin(), joints_.end(), it->name);
		if (joint != joints_.end() && joint->name == it->name) {
			// assign path tolerance
			joint->path_tolerance = it->position;
		}
	}
	for(auto it = msg.goal_tolerance.begin(); it != msg.goal_tolerance.end(); it++) {
		// find joint name in joints_ using binary search
		auto joint = lower_bound(joints_.begin(), joints_.end(), it->name);
		if (joint != joints_.end() && joint->name == it->name) {
			// assign path tolerance
			joint->goal_tolerance = it->position;
			// ignore velocity, acceleration and efforts
		}
	}

	// load time tolerance
	goal_time_tolerance_ = msg.goal_time_tolerance.toSec();
}

control_msgs::FollowJointTrajectoryGoal JointTrajectoryData::getTrajectoryMsg(bool reverse, double scale)
{
	FollowJointTrajectoryGoal msg;

	// set header
	msg.trajectory.header.stamp = ros::Time::now();

	// set joints' names and tolerances
	for(auto it = joints_.begin(); it != joints_.end(); it++) {
		// joint_names
		msg.trajectory.joint_names.push_back(it->name);
		// tolerances
		msg.path_tolerance.emplace_back();
		msg.path_tolerance.back().name = it->name;
		msg.path_tolerance.back().position = it->path_tolerance;
		msg.goal_tolerance.emplace_back();
		msg.goal_tolerance.back().name = it->name;
		msg.goal_tolerance.back().position = it->goal_tolerance;
	}
	msg.goal_time_tolerance.fromSec( goal_time_tolerance_ );
	// add supports
	for(auto it = supports_.begin(); it != supports_.end(); it++) {
		msg.trajectory.joint_names.push_back("support/" + it->name);
	}

	// Load trajectory into message. Do not set velocities, accelerations and efforts.
	if (!reverse) {
		for(auto it = trajectory_points_.begin(); it != trajectory_points_.end(); it++) {
			// add new point
			msg.trajectory.points.emplace_back();
			JointTrajectoryPoint& point = msg.trajectory.points.back();
			// set postions, ignore (leave empty) velocities, accelerations and efforts
			point.positions = it->positions;
			// set supports values
			point.positions.insert(point.positions.end(), it->supports.begin(), it->supports.end());
			// set time
			point.time_from_start.fromSec(scale * it->time_from_start);
		}
	}
	else {
		if (!trajectory_points_.empty()) {
			double end_time = trajectory_points_.back().time_from_start;

			for(auto it = trajectory_points_.rbegin(); it != trajectory_points_.rend(); it++) {
				// add new point
				msg.trajectory.points.emplace_back();
				JointTrajectoryPoint& point = msg.trajectory.points.back();
				// set postions, ignore (leave empty) velocities, accelerations and efforts
				point.positions = it->positions;
				// set supports values
				auto next = it + 1; // they must be shifted in time
				if (next != trajectory_points_.rend()) point.positions.insert(point.positions.end(), next->supports.begin(), next->supports.end());
				else point.positions.insert(point.positions.end(), it->supports.begin(), it->supports.end());
				// set time
				point.time_from_start.fromSec( scale*(end_time - it->time_from_start) );
			}
		}
	}
	return msg;
}


void JointTrajectoryData::clear()
{
  joints_.clear();
  supports_.clear();
  trajectory_points_.clear();
  goal_time_tolerance_ = 0.0;
}

bool JointTrajectoryData::addJoint(const string& name, double path_tolerance /* = 0.0 */, double goal_tolerance /* = 0.0 */)
{
	auto it = lower_bound(joints_.begin(), joints_.end(), name);
	if (it != joints_.end() && it->name == name) {
		// joint already exists so only change tolerance
		it->path_tolerance = path_tolerance;
		it->goal_tolerance = goal_tolerance;
		return false; 
	}

	// now add new joint AFTER lower bound
	auto new_joint = joints_.emplace(it); // call default constructor
	new_joint->name = name;
	new_joint->path_tolerance = path_tolerance;
	new_joint->goal_tolerance = goal_tolerance;
	new_joint->index = distance(joints_.begin(), new_joint);
	// modify indexes of last joints
	for(auto it = new_joint + 1; it != joints_.end(); it++) it->index++;

	// now insert position
	for(auto point = trajectory_points_.begin(); point != trajectory_points_.end(); point++) {
		point->positions.insert(point->positions.begin() + new_joint->index, 0.0);
    }

	return true;
}

void JointTrajectoryData::removeJoint(unsigned int index)
{
	// check diapazone
	if (index >= joints_.size()) throw std::out_of_range("joint index");
	// delete joint
    joints_.erase(joints_.begin() + index);
	// delete corresponding trajectory points
	for(auto point = trajectory_points_.begin(); point != trajectory_points_.end(); point++) { 
		point->positions.erase(point->positions.begin() + index);
	}
}

int JointTrajectoryData::getJointIndex(const std::string& name) {
	//TODO exception?
	auto it = lower_bound(joints_.begin(), joints_.end(), name);
	if (it == joints_.end() || it->name != name) return -1; // joint not found
	return distance(joints_.begin(), it);
}

bool JointTrajectoryData::addSupport(const std::string& name)
{
	auto it = lower_bound(supports_.begin(), supports_.end(), name);
	if (it != supports_.end() && it->name == name) {
		// support already exists
		return false; 
	}

	// now add new support AFTER lower bound
	auto new_support = supports_.emplace(it); // call default constructor
	new_support->name = name;
	new_support->index = distance(supports_.begin(), new_support);
	// modify indexes of last supports
	for(auto it = new_support + 1; it != supports_.end(); it++) it->index++;

	// now insert position
	for(auto point = trajectory_points_.begin(); point != trajectory_points_.end(); point++) {
		point->supports.insert(point->supports.begin() + new_support->index, 0.0);
    }

	return true;
}

void JointTrajectoryData::removeSupport(unsigned int index)
{
	// check diapazone
	if (index >= supports_.size()) throw std::out_of_range("support index");
	// delete support
    supports_.erase(supports_.begin() + index);
	// delete corresponding trajectory points
	for(auto point = trajectory_points_.begin(); point != trajectory_points_.end(); point++) { 
		point->supports.erase(point->supports.begin() + index);
	}
}

int JointTrajectoryData::getSupportIndex(const std::string& name)
{
	//TODO exception?
	auto it = lower_bound(supports_.begin(), supports_.end(), name);
	if (it == supports_.end() || it->name != name) return -1; // support not found
	return distance(supports_.begin(), it);
}


void JointTrajectoryData::addPoint(const TrajectoryPoint& point)
{
	if (point.time_from_start < 0.0) throw std::invalid_argument("time_from_start must be nonegative.");
	// find appropriate place to insert new element
	auto it = upper_bound(trajectory_points_.begin(), trajectory_points_.end(), point.time_from_start, [](double t, const TrajectoryPoint& p) { return t < p.time_from_start; } );
	// insert it 
	it = trajectory_points_.insert(it, point);
	it->crc = crc(it->positions);
}
	
void JointTrajectoryData::addPointMsg(const sensor_msgs::JointState& msg, double time_from_start) {
	unsigned int n_joints = joints_.size();
	// convert msg to TrajectoryPoint
	TrajectoryPoint point;
	point.time_from_start = time_from_start;
	point.positions.assign(n_joints, 0.0); // default values

	// now add position values from msg
	if (msg.name.size() != msg.position.size()) throw std::invalid_argument("JointState name and position sizes are not consistent.");
	for (int i = 0; i < n_joints; ++i) {
		auto it = find(msg.name.begin(), msg.name.end(), joints_[i].name);
		if (it != msg.name.end()) {
			int index = distance(msg.name.begin(), it);
            point.positions[i] = msg.position[index];
        }
    }
	// add point
	return addPoint(point);
}

sensor_msgs::JointState JointTrajectoryData::getPointMsg(unsigned int index)
{
	const TrajectoryPoint& point = getPoint(index);

	// construct message
	JointState msg;
	// header 
    msg.header.stamp = ros::Time::now();
	// name
	msg.name.reserve(joints_.size());
	for (auto it = joints_.begin(); it != joints_.end(); it++) msg.name.push_back(it->name);
	// position
	msg.position = getPoint(index).positions;
	// do not set velocities and efforts
	return msg;
}

void JointTrajectoryData::setPointJointPosition(unsigned int index, unsigned int joint_index, double value) 
{ 
	TrajectoryPoint& point = trajectory_points_.at(index);
	point.positions.at(joint_index) = value; 
	point.crc = crc(point.positions);
}

void JointTrajectoryData::setPointSupport(unsigned int index, unsigned int support_index, double value)
{
	TrajectoryPoint& point = trajectory_points_.at(index);
	point.supports.at(support_index) = value; 
}

void JointTrajectoryData::setPointTimeFromStart(unsigned int index, double time_from_start)
{
	TrajectoryPoint& point = trajectory_points_.at(index);
	if (time_from_start < 0.0) return; // TODO exception?
	point.time_from_start = time_from_start;
	sort(trajectory_points_.begin(), trajectory_points_.end());
}

void JointTrajectoryData::scaleTrajectory(double scale) 
{
	if (scale <= 0.0) return;
	for(auto it = trajectory_points_.begin(); it != trajectory_points_.end(); it++) {
		it->time_from_start *= scale;
	}
}

void JointTrajectoryData::removePoint(unsigned int index)
{
	// check diapazone
	if (index >= trajectory_points_.size()) throw std::out_of_range("trajectory point index");
	// delete joint
    trajectory_points_.erase(trajectory_points_.begin() + index);
}

void JointTrajectoryData::setPathTolerance(double path_tolerance)
{
	for (auto it = joints_.begin(); it != joints_.end(); it++) it->path_tolerance = path_tolerance;
}

void JointTrajectoryData::setGoalTolerance(double goal_tolerance)
{
	for (auto it = joints_.begin(); it != joints_.end(); it++) it->goal_tolerance = goal_tolerance;
}

} // namespace hmi
} // namespace sweetie_bot
