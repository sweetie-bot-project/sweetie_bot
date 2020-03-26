#include "limb_pose_marker.hpp"
#include "stance_pose_marker.hpp"

using namespace visualization_msgs;
using namespace interactive_markers;
using namespace sweetie_bot::hmi;

// Interactive marker server
std::shared_ptr<InteractiveMarkerServer> server;

// Display platform shape as controlled body
Marker makeCubeBody(double scale)
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = 0.16*scale;
  marker.scale.y = 0.08*scale;
  marker.scale.z = 0.02*scale;
  marker.color.r = 0.8;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.7;

  return marker;
}

// Display sphere as controlled body
Marker makeSphereBody(double scale)
{
  Marker marker;

  marker.type = Marker::SPHERE;
  marker.scale.x = 0.08*scale;
  marker.scale.y = 0.08*scale;
  marker.scale.z = 0.08*scale;
  marker.color.r = 0.8;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.7;

  return marker;
}

typedef visualization_msgs::Marker (*makeMarkerBody)(const double scale);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_pose_marker");

  ros::NodeHandle stance_nh("~");

  // marker server
  server.reset( new InteractiveMarkerServer(ros::this_node::getNamespace(),"",false) );
  ros::Duration(0.1).sleep();

  // create markers
  StancePoseMarker stanceMarker(server, &makeCubeBody, stance_nh);

  // add legs markers
  std::vector<std::unique_ptr<LimbPoseMarker>> leg_markers;
  if (stance_nh.hasParam("inner_markers/legs/")) {
    ros::NodeHandle legs_common_nh("~inner_markers/legs/");

    XmlRpc::XmlRpcValue legs;
    stance_nh.getParam("inner_markers/legs/list/", legs);

    for (auto& leg: legs) {
      ros::NodeHandle leg_nh("~inner_markers/legs/list/" + leg.first);

      leg_markers.push_back(std::unique_ptr<LimbPoseMarker>(new LimbPoseMarker(server, &makeCubeBody, legs_common_nh, leg_nh)));
    }
  } else {
    ROS_WARN("RobotPoseMarker: Less than 4 legs has defined in yaml file. Marker may not function properly");
  }

  // add limbs markers
  std::vector<std::unique_ptr<LimbPoseMarker>> limb_markers;
  if (stance_nh.hasParam("inner_markers/limbs/")) {
    XmlRpc::XmlRpcValue limbs;
    stance_nh.getParam("inner_markers/limbs/", limbs);

    for (auto& limb: limbs) {
      ros::NodeHandle limb_nh("~inner_markers/limbs/" + limb.first);

      bool is_sphere;
      limb_nh.param<bool>("is_sphere", is_sphere, true);
      const makeMarkerBody& makeBodyRef = is_sphere ? &makeSphereBody : &makeCubeBody;

      limb_markers.push_back(std::unique_ptr<LimbPoseMarker>(new LimbPoseMarker(server, makeBodyRef, limb_nh)));
    }
  }

  stanceMarker.setLegMarkers(leg_markers);
  stanceMarker.setLimbMarkers(limb_markers);

  ROS_INFO("pose_marker is started!");

  // main loop()
  ros::spin();

  // shutdown
  server.reset();

  ROS_INFO("pose_marker is shutdown!");
}
