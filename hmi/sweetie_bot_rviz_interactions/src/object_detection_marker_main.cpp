#include "object_detection_marker.hpp"

using namespace interactive_markers;
using namespace sweetie_bot::hmi;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_detection_marker");

	// node hanler
	ros::NodeHandle nh("~");

	// get parameters
	std::vector<std::string> names;
	if (!ros::param::get("~names", names)) {
		names = { "Object marker" };
	}

	// interactive server
	auto server = std::make_shared<InteractiveMarkerServer>(ros::this_node::getNamespace(), ros::this_node::getName(), false);

	// create markers
	std::vector< std::unique_ptr<ObjectDetectionMarker> > markers;
	for(const std::string& name : names) {
		markers.emplace_back( new ObjectDetectionMarker(name, server, nh) );
	}

	ROS_INFO("object_detection_marker is started!");

	// main loop()
	ros::spin();

	ROS_INFO("object_detection_marker is shutdown!");
}
