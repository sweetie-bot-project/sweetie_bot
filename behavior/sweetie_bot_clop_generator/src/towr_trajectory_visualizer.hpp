#ifndef  TOWR_TRAJECTORY_VISUALIZER_HPP
#define  TOWR_TRAJECTORY_VISUALIZER_HPP

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <xpp_states/robot_state_cartesian.h>
#include <xpp_msgs/RobotParameters.h>
#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/RobotStateCartesianTrajectory.h>
#include <visualization_msgs/MarkerArray.h>

#include <towr/nlp_formulation.h>


class TowrSolutionVisualizer
{
	typedef std::vector<xpp::RobotStateCartesian> XppVec;
	typedef towr::SplineHolder SplineHolder;
	typedef towr::RobotModel RobotModel;
	typedef towr::NlpFormulation NlpFormulation;

	protected:
		double period;
		std::string world_frame_id_;
		std::string xpp_namespace_;

		ros::Publisher terrain_pub_;
		ros::Publisher robot_parameters_pub_;
		ros::Publisher robot_state_pub_;
		ros::Publisher terrain_info_pub_;

	public:
		TowrSolutionVisualizer(ros::NodeHandle& nh, double period, const std::string& world_frame_id, const std::string& xpp_namespace);

		xpp_msgs::RobotStateCartesian GetRobotStateCartesianMsg(const towr::SplineHolder& solution, double t) const;
		xpp_msgs::RobotStateCartesianTrajectory GetRobotStateCartesianTrajectoryMsg(const towr::SplineHolder& solution) const;
		xpp_msgs::RobotParameters GetRobotParametersMsg(const towr::RobotModel& model) const;
		xpp_msgs::TerrainInfo TowrSolutionVisualizer::GetTerrainInfoMsg(const towr::Terrain& terrain, const std::vector<xpp_msgs::LinState3D>& ee_states) const;
		visualization_msgs::MarkerArray GetTerrainMsg(const towr::HeightMap& terrain) const

		void SaveOptimizationAsRosbag(const std::string& bag_name, const towr::NlpFormulation& formulation, const towr::SplineHolder& solution, ros::Time t0 = ros::Time(1e-6)) const;  // zero causes ROS exception

		void PublishPersistentTopics(const towr::NlpFormulation& formulation);
		void PublishSolution(const towr::NlpFormulation& formulation, const towr::SplineHolder& solution, double t);
		void PublishState(const BaseState& base_state, const towr::NlpFormulation::EEPos& ee_state);

	protected:
		xpp_msgs::RobotStateCartesian GetRobotStateCartesianMsg_impl(const towr::SplineHolder& solution, const towr::EulerConverter& base_angular, double t) const;
		void AddTrajectoryToRosbag (rosbag::Bag& bag, const towr::NlpFormulation& formulation, const towr::SplineHolder& solution) const;

};


#endif  /*TOWR_TRAJECTORY_VISUALIZER_HPP*/

