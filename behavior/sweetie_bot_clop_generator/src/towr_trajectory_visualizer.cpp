#include "towr_trajectory_visualizer.hpp"

#include <xpp_states/convert.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <towr_ros/towr_xpp_ee_map.h>
#include <towr/variables/euler_converter.h>
#include <towr/terrain/height_map.h>


TowrSolutionVisualizer::TowrSolutionVisualizer(ros::NodeHandle& nh, double period, const std::string& world_frame_id, const std::string& xpp_namespace) :
	period_(period), xpp_namespace_(xpp_namespace), world_frame_id_(world_frame_id)
{
	terrain_pub_ = nh.advertise<visualization_msgs::MarkerArray>(xpp_namespace + "/xpp/terrain");
	robot_parameters_pub_ = nh.advertise<xpp_msgs::RobotParameters>(xpp_namespace + xpp_msgs::robot_parameters);
	robot_state_pub_ = nh.advertise<xpp_msgs::RobotStateCartesian>(xpp_namespace + xpp_msgs::robot_state_desired);
	terrain_info_pub_ = nh.advertise<xpp_msgs::TerrainInfo>(xpp_namespace + xpp_msgs::terrain_info);
}


xpp_msgs::RobotStateCartesian TowrSolutionVisualizer::GetRobotStateCartesianMsg(const towr::SplineHolder& solution, double t) const 
{
	double T = solution.base_linear_->GetTotalTime();
	if (t < 0.0) t = 0.0;
	if (t > T) t = T;
	towr::EulerConverter base_angular(solution.base_angular_);

	return GetRobotStateCartesianMsg_impl(solution, base_angular, t);
}

xpp_msgs::RobotStateCartesian TowrSolutionVisualizer::GetRobotStateCartesianMsg_impl(const towr::SplineHolder& solution, const towr::EulerConverter& base_angular, double t) const 
{ 
	xpp_msgs::RobotStateCartesian msg;
	// time
	msg.time_from_start = t;
	// base linear position
	towr::State lin_state =	solution.base_linear_->GetPoint(t);
	tf::pointEigenToMsg(lin_state.at(kPos), msg.base.pose.position);
	tf::vectorEigenToMsg(lin_state.at(kVel), msg.base.twist.linear);
	tf::vectorEigenToMsg(lin_state.at(kAcc), msg.base.accel.linear);
	// base orientataon
	tf::quaternionEigenToMsg(base_angular.GetQuaternionBaseToWorld(t), msg.base.orientation);
	tf::vectorEigenToMsg(base_angular.GetAngularVelocityInWorld(t), msg.base.twist.angular);
	tf::vectorEigenToMsg(base_angular.GetAngularAccelerationInWorld(t), msg.base.accel.angular);
	// end effector positions, forces and contacts
	int n_ee = solution.ee_motion_.size();
	msg.ee_motion.resize(n_ee);
	msg.ee_force.resize(n_ee);
	msg.ee_contact.resize(n_ee);
	for (int ee=0; ee<n_ee; ++ee) {
		// map to xpp index
		int ee_xpp = towr::ToXppEndeffector(n_ee, ee).first;
		// position, velocity and acceleration
		towr::State ee_state = solution.ee_motion_.at(ee_towr)->GetPoint(t);
		tf::pointEigenToMsg(ee_state.at(kPos), msg.ee_motion[ee_xpp].pos);
		tf::vectorEigenToMsg(ee_state.at(kVel), msg.ee_motion[ee_xpp].vel);
		tf::vectorEigenToMsg(ee_state.at(kAcc), msg.ee_motion[ee_xpp].acc);
		// reaction forces
		tf::vectorEigenToMsg(solution.ee_force_.at(ee_towr)->GetPoint(t).p(), msg.ee_forces[ee_xpp]);
		// contact
		msg.ee_contact[ee_xpp] = solution.phase_durations_.at(ee_towr)->IsContactPhase(t);
	}
}


xpp_msgs::RobotStateCartesianTrajectory TowrSolutionVisualizer::GetRobotStateCartesianTrajectoryMsg(const towr::SplineHolder& solution) const 
{ 
	double t = 0.0;
	double T = solution.base_linear_->GetTotalTime();
	towr::EulerConverter base_angular(solution.base_angular_);

	xpp_msgs::RobotStateCartesianTrajectory msg;
	// fill header
	msg.header.frame_id = world_frame_id_;
	msg.header.stamp = ros::Time::now();
	// fill points array
	msg.points.reserve(std::ceil(T/period_) + 1);
	while (t<=T+1e-5) {
		msg.points.push_back(GetPoint_impl(solution, base_angular_, t));
		t += period;
	}
	return msg;
}

xpp_msgs::RobotParameters TowrSolutionVisualizer::GetRobotParametersMsg(const towr::RobotModel& model) const
{
	xpp_msgs::RobotParameters params_msg;

	// nominal pose	
	auto nominal_B = model.kinematic_model_->GetNominalStanceInBase();
	int n_ee = nominal_B.size();
	for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
		towr::KinematicModel::Vector3d pos = nominal_B.at(ee_towr);
		params_msg.nominal_ee_pos.push_back(xpp::Convert::ToRos<geometry_msgs::Point>(pos));
		params_msg.ee_names.push_back(towr::ToXppEndeffector(n_ee, ee_towr).second);
	}
	// maximal deviations
	auto max_dev_xyz = model.kinematic_model_->GetMaximumDeviationFromNominal();
	params_msg.ee_max_dev = xpp::Convert::ToRos<geometry_msgs::Vector3>(max_dev_xyz);
	// robot mass
	params_msg.base_mass = model.dynamic_model_->m();

	return params_msg;
}

visualization_msgs::MarkerArray TowrSolutionVisualizer::GetTerrainMsg(const towr::HeightMap& terrain) const
{
  // x-y area patch that should be drawn in rviz
  double dxy   =  0.06;
  double x_min = -1.0;
  double x_max =  4.0;
  double y_min = -1.0;
  double y_max =  1.0;

  visualization_msgs::Marker m;
  int id = 0;
  m.type = visualization_msgs::Marker::CUBE;
  m.scale.z = 0.003;
  m.ns = "terrain";
  m.header.frame_id = world_frame_id_;
  m.color.r = 245./355; m.color.g  = 222./355; m.color.b  = 179./355; // wheat
  m.color.a = 0.65;

  visualization_msgs::MarkerArray msg;
  double x =  x_min;
  while (x < x_max) {
    double y = y_min;
    while (y < y_max) {
      // position
      m.pose.position.x = x;
      m.pose.position.y = y;
      m.pose.position.z = terrain.GetHeight(x,y);

      // orientation
      Eigen::Vector3d n = terrain.GetNormalizedBasis(towr::HeightMap::Normal, x, y);
      Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0,0,1), n);
      m.pose.orientation.w = q.w();
      m.pose.orientation.x = q.x();
      m.pose.orientation.y = q.y();
      m.pose.orientation.z = q.z();

      // enlarge surface-path when tilting
      double gain = 1.5;
      m.scale.x = (1+gain*n.cwiseAbs().x())*dxy;
      m.scale.y = (1+gain*n.cwiseAbs().y())*dxy;

      m.id = id++;
      msg.markers.push_back(m);

      y += dxy;
    }
    x += dxy;
  }
  return msg;
}
		 
void TowrSolutionVisualizer::SaveOptimizationAsRosbag(const std::string& bag_name, const towr::NlpFormulation& formulation, const towr::SplineHolder& solution, ros::Time t0) const 
{
	// t=0.0 throws ROS exception
	rosbag::Bag bag;
	bag.open(bag_name, rosbag::bagmode::Write);

	// save the a-priori fixed optimization variables
	bag.write(xpp_namespace_ + xpp_msgs::robot_parameters, t0, GetRobotParametersMsg(formulation.model_));
	bag.write(xpp_namespace_ + "/xpp/terrain", t0, GetTerrainMsg(*formulation.terrain_));

	AddTrajectoryToRosbag(bag, formulation, solution, t0);

	bag.close();
}

xpp_msgs::TerrainInfo TowrSolutionVisualizer::GetTerrainInfoMsg(const towr::Terrain& terrain, const std::vector<xpp_msgs::LinState3D>& ee_states) const
{
	xpp_msgs::TerrainInfo terrain_msg;
	for (auto ee_state : ee_states) {
		Eigen::Vector3d n = terrain.GetNormalizedBasis(towr::HeightMap::Normal, ee_state.pos.x, ee_state.pos.y); // get normal to surface at contact point
		terrain_msg.surface_normals.emplace_back();
		tf::vectorEigenToMsg(n, terrain_msg.surface_normals.back());
	}
	terrain_msg.friction_coeff = formulation.terrain_->GetFrictionCoeff();
	return terrain_msg;
}


void TowrSolutionVisualizer::AddTrajectoryToRosbag (rosbag::Bag& bag, const towr::NlpFormulation& formulation, const towr::SplineHolder& solution, ros::Time) const
{
	// t=0.0 throws ROS exception
	std::string terrain_info_topic = xpp_namespace_ + xpp_msgs::terrain_info;
	std::string robot_state_topic = xpp_namespace_ + xpp_msgs::robot_state_desired;

	towr::EulerConverter base_angular(solution.base_angular_);

	// add trajectory points to bag
	double t = 0.0;
	double T = solution.base_linear_->GetTotalTime();
	int n_samples = std::ceil(T/period_) + 1;
	while(n_samples--) {
		t = std::min(t, T);

		// robot state message
		xpp_msg::RobotStateCartesian state = GetRobotStateCartesianMsg_impl(solution, base_angular_, t);
		bag.write(robot_state_topic, timestamp, state);

		// terrain info message
		bag.write(terrain_info_topic, timestamp, GetTerrainInfoMsg(*formulation.terrain_, state.ee_motion));

		t += period_;
		timestamp += ros::Duration(period);
	}
}

void TowrSolutionVisualizer::PublishPersistentTopics(const towr::NlpFormulation& formulation)
{
	robot_parameters_pub_.publish(GetRobotParametersMsg(formulation.model_));
	terrain_pub_.publish(GetTerrainMsg(*formulation.terrain_));
}

void TowrSolutionVisualizer::PublishSolution(const towr::NlpFormulation& formulation, const towr::SplineHolder& solution, double t)
{
	xpp_msg::RobotStateCartesian state = GetRobotStateCartesianMsg(solution, t);
	robot_state_pub_.publish(state);
	terrain_info_pub_.publish(GetTerrainInfoMsg(*formulation.terrain_, state.ee_motion));
}

void TowrSolutionVisualizer::PublishState(const BaseState& base_state, const towr::NlpFormulation::EEPos& ee_state)
{
	xpp_msgs::RobotStateCartesian msg;
	// time
	msg.time_from_start = 0.0;
	// base linear position
	tf::pointEigenToMsg(base_state.lin.at(kPos), msg.base.pose.position);
	tf::vectorEigenToMsg(Eigen::Vector3d::Zero(), msg.base.twist.linear);
	tf::vectorEigenToMsg(Eigen::Vector3d::Zero(), msg.base.accel.linear);
	// base orientataon
	Vector3d rpy = base_state.ang.at(towr::kPos);
	KDL::Rotation rot_kdl( KDL::Rotation::RPY(rpy.x(), rpy.y(), rpy.z()) );
	tf::quaternionKDLToMsg(rot_kdl msg.base.orientation);
	tf::vectorEigenToMsg(Eigen::Vector3d::Zero(), msg.base.twist.angular);
	tf::vectorEigenToMsg(Eigen::Vector3d::Zero(), msg.base.accel.angular); 
	// end effector positions, forces and contacts
	int n_ee = ee_state.size();
	msg.ee_motion.resize(n_ee);
	msg.ee_force.resize(n_ee);
	msg.ee_contact.resize(n_ee);
	for (int ee=0; ee<n_ee; ++ee) {
		// map to xpp index
		int ee_xpp = towr::ToXppEndeffector(n_ee, ee).first;
		// position, velocity and acceleration
		tf::pointEigenToMsg(ee_state[ee], msg.ee_motion[ee_xpp].pos);
		tf::vectorEigenToMsg(Eigen::Vector3d::Zero(), msg.ee_motion[ee_xpp].vel);
		tf::vectorEigenToMsg(Eigen::Vector3d::Zero(), msg.ee_motion[ee_xpp].acc);
		// reaction forces
		tf::vectorEigenToMsg(Eigen::Vector3d::Zero(), msg.ee_forces[ee_xpp]);
		// contact
		msg.ee_contact[ee_xpp] = false;
	}
	// publish message
	robot_state_pub_.publish(msg);
}

