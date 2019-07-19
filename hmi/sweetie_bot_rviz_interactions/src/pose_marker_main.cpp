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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pose_marker");

	// get node parameters
  double scale;
  std::vector<std::string> resources;
  std::vector<std::string> frames;
  std::string stance_home_frame;
  double stance_normalized_z_level;
  double limbs_normalized_z_level;
  double head_normalized_z_level;

	ros::param::get("~scale", scale);
	ros::param::get("~resources", resources);
	ros::param::get("~frames", frames);
	ros::param::get("~stance_home_frame", stance_home_frame);
	ros::param::get("~stance_normalized_z_level", stance_normalized_z_level);
	ros::param::get("~limbs_normalized_z_level", limbs_normalized_z_level);
	ros::param::get("~head_normalized_z_level", head_normalized_z_level);

  // marker server
	server.reset( new InteractiveMarkerServer(ros::this_node::getNamespace(),"",false) );
	ros::Duration(0.1).sleep();

  // create markers
  StancePoseMarker stanceMarker(server, &makeCubeBody, "stance", scale, resources, frames, stance_home_frame, stance_normalized_z_level);
  LimbPoseMarker frontLeft_limbMarker(server, &makeCubeBody, "leg_fl", 0.5*scale, resources, frames, frames[0], limbs_normalized_z_level);
  LimbPoseMarker frontRight_limbMarker(server, &makeCubeBody, "leg_fr", 0.5*scale, resources, frames, frames[1], limbs_normalized_z_level);
  LimbPoseMarker backLeft_limbMarker(server, &makeCubeBody, "leg_bl", 0.5*scale, resources, frames, frames[2], limbs_normalized_z_level);
  LimbPoseMarker backRight_limbMarker(server, &makeCubeBody, "leg_br", 0.5*scale, resources, frames, frames[3], limbs_normalized_z_level);
  LimbPoseMarker headMarker(server, &makeSphereBody, "head", 0.5*scale, resources, frames, frames[4], head_normalized_z_level);

  ROS_INFO("pose_marker is started!");

	// main loop()
	ros::spin();

  // shutdown
	server.reset();

  ROS_INFO("pose_marker is shutdown!");
}
