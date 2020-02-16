#include "destination_marker.hpp"
#include <QApplication>

using namespace interactive_markers;
using namespace sweetie_bot::hmi;

// Interactive marker server
std::shared_ptr<InteractiveMarkerServer> server;

int main(int argc, char **argv) {
  ros::init(argc, argv, "destination_marker");

  QApplication a(argc, argv);

  ros::NodeHandle dest_marker_nh("~");

  // marker server
  server.reset( new InteractiveMarkerServer(ros::this_node::getNamespace(),"",false) );
  ros::Duration(0.1).sleep();

  DestinationMarker destMarker(server, dest_marker_nh);

  ROS_INFO("destination_marker has been started!");

  ros::spin();

  server.reset();

  ROS_INFO("destination_marker has been shutdown!");
}
