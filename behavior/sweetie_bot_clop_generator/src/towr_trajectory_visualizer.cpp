#include "towr_trajectory_visualizer.hpp"

#include <xpp_states/convert.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <towr_ros/towr_xpp_ee_map.h>
#include <towr/variables/euler_converter.h>
#include <towr/terrain/height_map.h>

TowrSolutionVisualizer::XppVec TowrSolutionVisualizer::GetTrajectory(const towr::SplineHolder& solution) const
{
	XppVec trajectory;
	double t = 0.0;
	double T = solution.base_linear_->GetTotalTime();
	Eigen::VectorXd p;

	towr::EulerConverter base_angular(solution.base_angular_);

	while (t<=T+1e-5) {
		int n_ee = solution.ee_motion_.size();
		xpp::RobotStateCartesian state(n_ee);

		state.base_.lin = towr::ToXpp(solution.base_linear_->GetPoint(t));

		state.base_.ang.q  = base_angular.GetQuaternionBaseToWorld(t);
		state.base_.ang.w  = base_angular.GetAngularVelocityInWorld(t);
		state.base_.ang.wd = base_angular.GetAngularAccelerationInWorld(t);

		for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
			int ee_xpp = towr::ToXppEndeffector(n_ee, ee_towr).first;

			state.ee_contact_.at(ee_xpp) = solution.phase_durations_.at(ee_towr)->IsContactPhase(t);
			state.ee_motion_.at(ee_xpp)  = towr::ToXpp(solution.ee_motion_.at(ee_towr)->GetPoint(t));
			p = solution.ee_force_.at(ee_towr)->GetPoint(t).p();
			if (p.size() == 1) {
				auto& f = state.ee_forces_.at(ee_xpp);
				f.x() = 0.0;
				f.y() = 0.0;
			  	f.z() = p.x();
		    }
			else {
				state.ee_forces_.at(ee_xpp) = p;
			}
		}
		

		state.t_global_ = t;
		trajectory.push_back(state);
		t += period;
	}

	return trajectory;
}

xpp_msgs::RobotStateCartesianTrajectory TowrSolutionVisualizer::GetRobotCartesianTrajectoryMsg(const towr::SplineHolder& solution) const 
{ 
	return xpp::Convert::ToRos(GetTrajectory(solution));
}

xpp_msgs::RobotParameters TowrSolutionVisualizer::GetRobotParametersMsg(const towr::RobotModel& model) const
{
	xpp_msgs::RobotParameters params_msg;
	auto max_dev_xyz = Eigen::Vector3d::Zero();
	params_msg.ee_max_dev = xpp::Convert::ToRos<geometry_msgs::Vector3>(max_dev_xyz);

	int n_ee = model.kinematic_model_->GetNumberOfEndeffectors();
	for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
		towr::KinematicModel::Vector3d pos = model.kinematic_model_->GetNominalStanceInBase(ee_towr);
		params_msg.nominal_ee_pos.push_back(xpp::Convert::ToRos<geometry_msgs::Point>(pos));
		params_msg.ee_names.push_back(towr::ToXppEndeffector(n_ee, ee_towr).second);
	}

	params_msg.base_mass = model.dynamic_model_->m();

	return params_msg;
}
		 
void TowrSolutionVisualizer::SaveOptimizationAsRosbag(const std::string& bag_name, const towr::NlpFormulationBase& formulation, const towr::SplineHolder& solution) const 
{
  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Write);
  ::ros::Time t0(1e-6); // t=0.0 throws ROS exception

  // save the a-priori fixed optimization variables
  bag.write(xpp_msgs::robot_parameters, t0, GetRobotParametersMsg(formulation.model_));
  bag.write("/xpp/terrain", t0, GetTerrainMsg(*formulation.terrain_));

  AddTrajectoryToRosbag(bag, formulation, solution , xpp_msgs::robot_state_desired);

  bag.close();
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
  m.header.frame_id = "odom_combined";
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

void TowrSolutionVisualizer::AddTrajectoryToRosbag (rosbag::Bag& bag, const towr::NlpFormulationBase& formulation, const towr::SplineHolder& solution, const std::string& topic) const
{
	XppVec traj = GetTrajectory(solution);
	for (const auto state : traj) {
		auto timestamp = ::ros::Time(state.t_global_ + 1e-6); // t=0.0 throws ROS exception

		xpp_msgs::RobotStateCartesian msg;
		msg = xpp::Convert::ToRos(state);
		bag.write(topic, timestamp, msg);

		xpp_msgs::TerrainInfo terrain_msg;
		for (auto ee : state.ee_motion_.ToImpl()) {
			Eigen::Vector3d n = formulation.terrain_->GetNormalizedBasis(towr::HeightMap::Normal, ee.p_.x(), ee.p_.y());
			terrain_msg.surface_normals.push_back(xpp::Convert::ToRos<geometry_msgs::Vector3>(n));
			terrain_msg.friction_coeff = formulation.terrain_->GetFrictionCoeff();
		}

		bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);
	}
}



void TowrSolutionVisualizer::PlayTrajectory(const towr::NlpFormulationBase& formulation, const towr::SplineHolder& solution, double replay_speed) const
{
	const std::string rosbag_file = "/tmp/towr_optimization.bag";
	SaveOptimizationAsRosbag(rosbag_file, formulation, solution);
	system(("rosbag play --topics " + xpp_msgs::robot_state_desired + " " + xpp_msgs::terrain_info + " -r " + std::to_string(replay_speed) + " --quiet " + rosbag_file).c_str());
}

