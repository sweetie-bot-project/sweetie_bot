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

	protected:
		XppVec GetTrajectory(const towr::SplineHolder& solution) const;
		void AddTrajectoryToRosbag(rosbag::Bag& bag, const towr::NlpFormulation& formulation, const towr::SplineHolder& solution, const std::string& topic) const;

	public:

		TowrSolutionVisualizer(double _period) : period(_period) {}

		xpp_msgs::RobotParameters GetRobotParametersMsg(const towr::RobotModel& model) const;
		xpp_msgs::RobotStateCartesianTrajectory GetRobotCartesianTrajectoryMsg(const towr::SplineHolder& solution) const;
		visualization_msgs::MarkerArray GetTerrainMsg(const towr::HeightMap& terrain) const;

		void SaveOptimizationAsRosbag(const std::string& bag_name, const towr::NlpFormulation& formulation, const towr::SplineHolder& solution/*, bool include_iterations*/) const;

		void PlayTrajectory(const towr::NlpFormulation& formulation, const towr::SplineHolder& solution, double replay_speed) const;
};


#endif  /*TOWR_TRAJECTORY_VISUALIZER_HPP*/

