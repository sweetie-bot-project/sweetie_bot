#include <ros/ros.h>

#include <tf/tf.h>
#include <interactive_markers/interactive_marker_server.h>
// #include <interactive_markers/menu_handler.h>

using namespace visualization_msgs;

// COMPONENT INTERFACE
// interactive marker server
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
//interactive_markers::MenuHandler menu_handler;
// publisers 
ros::Publisher pose_pub;

// event handler
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	if ( feedback->marker_name != "pose6D" ) return;

	std::ostringstream s;
	s << "Feedback from marker '" << feedback->marker_name << "' "
		<< " / control '" << feedback->control_name << "'";

	switch ( feedback->event_type )
	{
		case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
			ROS_DEBUG_STREAM( s.str() << ": pose changed"
					<< "\nposition = "
					<< feedback->pose.position.x
					<< ", " << feedback->pose.position.y
					<< ", " << feedback->pose.position.z
					<< "\norientation = "
					<< feedback->pose.orientation.w
					<< ", " << feedback->pose.orientation.x
					<< ", " << feedback->pose.orientation.y
					<< ", " << feedback->pose.orientation.z
					<< "\nframe: " << feedback->header.frame_id
					<< " time: " << feedback->header.stamp.sec << "sec, "
					<< feedback->header.stamp.nsec << " nsec" );
			{
				geometry_msgs::PoseStamped pose_stamped;

				pose_stamped.header = feedback->header;
				pose_stamped.pose = feedback->pose;
				pose_pub.publish(pose_stamped);
			}
			break;
	}

	server->applyChanges();
}


// Display platform shape as controlled body
Marker makeBody()
{
	Marker marker;

	marker.type = Marker::CUBE;
	marker.scale.x = 0.16;
	marker.scale.y = 0.08;
	marker.scale.z = 0.02;
	marker.color.r = 0.8;
	marker.color.g = 0.5;
	marker.color.b = 0.5;
	marker.color.a = 0.7;

	return marker;
}

void make6DofMarker()
{
	InteractiveMarker int_marker;
	//header setup
	int_marker.header.frame_id = "odom_combined";
	//int_marker.pose.position = ...;
	int_marker.scale = 0.1;
	int_marker.name = "pose6D";
	std::string node_name = ros::this_node::getName();
	size_t slash_pos = node_name.find_last_of('/');
	if (slash_pos != std::string::npos) node_name = node_name.substr(slash_pos + 1);
	int_marker.description = node_name;

	// insert base_link model 
	{
		InteractiveMarkerControl control;
		// TODO: with base mesh
		control.always_visible = true;
		control.markers.push_back( makeBody() );
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
		int_marker.controls.push_back( control );
	}

	// add iteractive controls
	{
		InteractiveMarkerControl control;
		tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
		orien.normalize();
		tf::quaternionTFToMsg(orien, control.orientation);
		control.name = "rotate_x";
		control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
		int_marker.controls.push_back(control);
		control.name = "move_x";
		control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back(control);

		orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
		orien.normalize();
		tf::quaternionTFToMsg(orien, control.orientation);
		control.name = "rotate_z";
		control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
		int_marker.controls.push_back(control);
		control.name = "move_z";
		control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back(control);

		orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
		orien.normalize();
		tf::quaternionTFToMsg(orien, control.orientation);
		control.name = "rotate_y";
		control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
		int_marker.controls.push_back(control);
		control.name = "move_y";
		control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back(control);
	}

	// add marker to server
	server->insert(int_marker);
	server->setCallback(int_marker.name, &processFeedback);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_marker");
	ros::NodeHandle n;

	//pose publishing
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 1);

	// marker server
	server.reset( new interactive_markers::InteractiveMarkerServer(ros::this_node::getNamespace(),"",false) );
	ros::Duration(0.1).sleep();
	// create marker
	make6DofMarker();
	server->applyChanges();

	ROS_INFO("pose_marker is started!");

	// main loop()
	ros::spin();
	// shutdown
	server.reset();
}
