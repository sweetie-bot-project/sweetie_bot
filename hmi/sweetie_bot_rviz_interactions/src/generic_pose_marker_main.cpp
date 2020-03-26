#include "generic_pose_marker.hpp"

using namespace visualization_msgs;
using namespace interactive_markers;
using namespace sweetie_bot::hmi;

// Interactive marker server
std::shared_ptr<InteractiveMarkerServer> server;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generic_pose_marker");

  ros::NodeHandle nh("~");

  // initialize marker server
  server.reset( new InteractiveMarkerServer(ros::this_node::getNamespace(),"",false) );
  ros::Duration(0.1).sleep();

  // create marker
  GenericPoseMarker poseMarker(server, nh);


  ROS_INFO("generic_pose_marker is started!");

  // main loop()
  ros::spin();

  // shutdown
  server.reset();

  ROS_INFO("generic_pose_marker is shutdown!");
}
