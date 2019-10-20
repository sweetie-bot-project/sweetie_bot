#include "destination_marker.hpp"
#include <QApplication>

using namespace interactive_markers;
using namespace sweetie_bot::hmi;

// Interactive marker server
std::shared_ptr<InteractiveMarkerServer> server;

int main(int argc, char **argv) {
  ros::init(argc, argv, "destination_marker");

  QApplication a(argc, argv);

  double scale;
  std::vector<std::string> gait_type_options;
  std::vector<int> _n_steps_options;
  double duration;
  double nominal_height;
  std::vector<std::string> ee_names;

	ros::param::get("~scale", scale);
	ros::param::get("~gait_type_options", gait_type_options);
	ros::param::get("~n_steps_options", _n_steps_options);
	ros::param::get("~duration", duration);
	ros::param::get("~nominal_height", nominal_height);
	ros::param::get("~ee_names", ee_names);

  // marker server
	server.reset( new InteractiveMarkerServer(ros::this_node::getNamespace(),"",false) );
	ros::Duration(0.1).sleep();

  std::vector<unsigned> n_steps_options(_n_steps_options.begin(), _n_steps_options.end());

  DestinationMarker destMarker(server, "destination_marker", scale, gait_type_options, n_steps_options, duration, nominal_height);
  destMarker.setEndEffectorTargets(ee_names);

  ROS_INFO("destination_marker has been started!");

  ros::spin();

  server.reset();

  ROS_INFO("destination_marker has been shutdown!");
}
