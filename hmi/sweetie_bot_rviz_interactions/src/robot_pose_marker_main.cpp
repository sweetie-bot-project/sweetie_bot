#include "stance_pose_marker.hpp"

using namespace interactive_markers;
using namespace sweetie_bot::hmi;

// Interactive marker server
std::shared_ptr<InteractiveMarkerServer> server;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_pose_marker");

  ros::NodeHandle stance_nh("~");

  // marker server
  server.reset( new InteractiveMarkerServer(ros::this_node::getNamespace(),"",false) );
  ros::Duration(0.1).sleep();

  // create markers
  StancePoseMarker stanceMarker(server, stance_nh);

  ROS_INFO("robot_pose_marker is started!");

  // main loop()
  ros::spin();

  // shutdown
  server.reset();

  ROS_INFO("robot_pose_marker is shutdown!");
}
