#include "object_detection_marker.hpp"

#include <cstdlib>

#include <ros/ros.h>
#include <tf/tf.h>

using namespace visualization_msgs;
using namespace interactive_markers;

namespace sweetie_bot {
namespace hmi {

ObjectDetectionMarker::ObjectDetectionMarker(const std::string& _name, ros::Publisher& detection_publisher, std::shared_ptr<interactive_markers::InteractiveMarkerServer> server, ros::NodeHandle node_handle) : 
	name(_name),
	server(server),
	publisher(detection_publisher)
{
	double period;

	// GET PARAMETERS
	if (!node_handle.getParam("period", period)) {
		period = 0.1;
	}
	if (!node_handle.getParam("frame", frame)) {
		frame = "odom_combined";
	}
	// detection parameters
	if (!node_handle.getParam("labels", labels)) {
			labels = { "", "Alex", "Alice" };
	}
	if (!node_handle.getParam("types", types)) {
			types = { "human", "pony", "ball" };
	}
	// marker configuration
	if (!node_handle.getParam("scale", scale)) {
		scale = 1.0;
	}
	if (scale < 0) {
		ROS_ERROR("ObjectDetectionMarker: scale parameter cannot be negative.");
		exit(1);
	}
	if (!node_handle.getParam("normalized_z_level", normalized_z_level)) {
		normalized_z_level = 0.4;
	}
	if (!node_handle.getParam("is6DOF", is6DOF)) {
		is6DOF = false;
	}

	// create detection message
	detection.label = (labels.size() > 0) ? labels[0] : "";
	detection.type = (types.size() > 0) ? types[0] : "";
	detection.id = rand();

	// setup publlish timer
	publish_timer = node_handle.createTimer(ros::Duration(period), &ObjectDetectionMarker::publishCallback, this);
	publish_timer.stop();
	is_publishing = false;

	// put marker in intial pose
	geometry_msgs::Pose init_pose;
	init_pose.position.x = 0.4;
	visualization_msgs::InteractiveMarker int_marker = makeInteractiveMarker(init_pose);

	// update detection 
	detection.header.frame_id = frame;
	detection.pose = init_pose;

	// create menu
	makeMenu();	

	// add marker to server
	server->insert(int_marker, boost::bind( &ObjectDetectionMarker::processFeedback, this, _1 ));
	menu_handler.apply( *server, int_marker.name );

	server->applyChanges();
}

visualization_msgs::Marker ObjectDetectionMarker::makeSphereMarker(float r, float g, float b)
{
	visualization_msgs::Marker marker;

	marker.type = Marker::SPHERE;
	marker.scale.x = 0.2*scale;
	marker.scale.y = 0.2*scale;
	marker.scale.z = 0.2*scale;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = 1;

	return marker;
}

visualization_msgs::InteractiveMarker ObjectDetectionMarker::makeInteractiveMarker(const geometry_msgs::Pose& pose)
{
	visualization_msgs::InteractiveMarker int_marker;

	//header setup
	int_marker.header.frame_id = frame;
	int_marker.pose = pose;
	int_marker.scale = 0.2*std::min(scale, 1.0);
	int_marker.name = name;
	int_marker.description = "label: " + detection.label + " type: " + detection.type + " id: " + std::to_string(detection.id);

	// insert object model
	{
		visualization_msgs::InteractiveMarkerControl control;
		control.always_visible = true;
		control.markers.push_back( makeSphereMarker(0.2, 0.2, 0.2) );
		tf::Quaternion orien(0.0, 0.0, 1.0, 1.0);
		orien.normalize();
		tf::quaternionTFToMsg(orien, control.orientation);
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
		int_marker.controls.push_back( control );
	}

	// add iteractive controls
	{
		visualization_msgs::InteractiveMarkerControl control;
		tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
		orien.normalize();
		tf::quaternionTFToMsg(orien, control.orientation);
		if (is6DOF) {
			control.name = "rotate_x";
			control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
			int_marker.controls.push_back(control);
		}
		control.name = "move_x";
		control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back(control);

		orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
		orien.normalize();
		tf::quaternionTFToMsg(orien, control.orientation);
		if (is6DOF) {
			control.name = "rotate_z";
			control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
			int_marker.controls.push_back(control);
		}
		control.name = "move_z";
		control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back(control);

		orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
		orien.normalize();
		tf::quaternionTFToMsg(orien, control.orientation);
		if (is6DOF) {
			control.name = "rotate_y";
			control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
			int_marker.controls.push_back(control);
		}
		control.name = "move_y";
		control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back(control);
	}

	return int_marker;
}

void ObjectDetectionMarker::updateInteractiveMarker() 
{
	visualization_msgs::InteractiveMarker int_marker;
	// update marker description
	server->get(name, int_marker);
	int_marker.description = "label: " + detection.label + " type: " + detection.type + " id: " + std::to_string(detection.id);
	server->erase(name);
	server->insert(int_marker, boost::bind( &ObjectDetectionMarker::processFeedback, this, _1 ), visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
	menu_handler.apply( *server, int_marker.name );
	server->applyChanges();
}

void ObjectDetectionMarker::updateInteractiveMarkerColor(float r, float g, float b)
{
	visualization_msgs::InteractiveMarker int_marker;
	// update marker description
	server->get(name, int_marker);
	int_marker.controls[0].markers[0].color.r = r;
	int_marker.controls[0].markers[0].color.g = g;
	int_marker.controls[0].markers[0].color.b = b;
	server->erase(name);
	server->insert(int_marker, boost::bind( &ObjectDetectionMarker::processFeedback, this, _1 ), visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
	server->applyChanges();
}


void ObjectDetectionMarker::makeMenu()
{
	// visibility trigger
	auto menu_entry_visible = menu_handler.insert("VISIBLE", boost::bind( &ObjectDetectionMarker::processToggleVisibility, this, _1 ));
	menu_handler.setCheckState(menu_entry_visible, is_publishing ? MenuHandler::CHECKED : MenuHandler::UNCHECKED);
	// list of labels
	if (!labels.empty()) {
		auto labels_submenu_entry = menu_handler.insert("label");
		for ( const std::string& label : labels ) {
			menu_handler.insert(labels_submenu_entry, label, boost::bind( &ObjectDetectionMarker::processChangeLabel, this, _1 ));
		}
	}
	// list of types
	if (!types.empty()) {
		auto types_submenu_entry = menu_handler.insert("types");
		for ( const std::string& type : types ) {
			menu_handler.insert(types_submenu_entry, type, boost::bind( &ObjectDetectionMarker::processChangeType, this, _1 ));
		}
	}
	// new id
	menu_handler.insert("New id", boost::bind( &ObjectDetectionMarker::processClickNewId, this, _1 ));
	// to nominal
	menu_handler.insert("Normalize", boost::bind( &ObjectDetectionMarker::processClickNormalize, this, _1 ));
}

void ObjectDetectionMarker::publishCallback(const ros::TimerEvent&)
{
	// update header and publish detection
	detection.header.stamp = ros::Time::now();
	
	sweetie_bot_text_msgs::DetectionArray array;
	array.detections.push_back(detection);
	publisher.publish(array);
}

// MARKER FEEDBACK HANDLERS

void ObjectDetectionMarker::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
{
		detection.header = feedback->header;
		detection.pose = feedback->pose;
}


void ObjectDetectionMarker::processToggleVisibility( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
{
	MenuHandler::CheckState check;
	menu_handler.getCheckState(feedback->menu_entry_id, check);
	switch (check) {
		case MenuHandler::CHECKED:
			menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
			is_publishing = false;
			updateInteractiveMarkerColor(0.2, 0.2, 0.2);
			publish_timer.stop();
			break;
		case MenuHandler::UNCHECKED:
			menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
			is_publishing = true;
			updateInteractiveMarkerColor(0.5, 0.8, 0.5);
			publish_timer.start();
			break;
	}
	menu_handler.reApply(*server);
	server->applyChanges();
}

void ObjectDetectionMarker::processClickNewId( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback ) 
{
	detection.id = rand();	
	updateInteractiveMarker();
}

void ObjectDetectionMarker::processChangeLabel( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
{
	std::string label;
	if (menu_handler.getTitle(feedback->menu_entry_id, label)) {
		detection.label = label;
		updateInteractiveMarker();
	}
}

void ObjectDetectionMarker::processChangeType( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
{
	std::string type;
	if (menu_handler.getTitle(feedback->menu_entry_id, type)) {
		detection.type = type;
		updateInteractiveMarker();
	}
}

void ObjectDetectionMarker::processClickNormalize( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
{
	detection.pose = feedback->pose;
	// normalize pose
	detection.pose.position.z = normalized_z_level;
	tf::Quaternion orien(0.0, 0.0, detection.pose.orientation.z, detection.pose.orientation.w);
	orien.normalize();
	tf::quaternionTFToMsg(orien, detection.pose.orientation);
	// set pose of marker
	server->setPose(name, detection.pose);
	server->applyChanges();
}

RandomObjectDetectionMarker::RandomObjectDetectionMarker(const std::string& _name, ros::Publisher& detection_publisher, std::shared_ptr<interactive_markers::InteractiveMarkerServer>& server, ros::NodeHandle& node_handle) : 
	ObjectDetectionMarker(_name, detection_publisher, server, node_handle)
{
	// GET PARAMETERS
	double period = 0.1;
	node_handle.getParam("period", period);

	// time lag
	double max_time_lag = 0.0;
	node_handle.getParam("max_time_lag", max_time_lag);
	this->lag_dist = std::uniform_real_distribution<float>(0.0, max_time_lag);

	// appear phase duration 
	std::vector<double> interval;
	if (!node_handle.getParam("appear_duration_interval", interval)) {
		interval = { 0.5, 2.0 };
	}
	if (interval.size() != 2 || interval[0] < 0.0 || interval[1] < 0.0) {
		ROS_ERROR("RandomObjectDetectionMarker: appear_duration_interval must be pair of nonegative float.");
		exit(1);
	}
	this->appear_time_dist = std::uniform_real_distribution<float>(interval[0], interval[1]);

	// vanish phase duration 
	if (!node_handle.getParam("vanish_duration_interval", interval)) {
		interval = { 0.25, 1.0 };
	}
	if (interval.size() != 2 || interval[0] < 0.0 || interval[1] < 0.0) {
		ROS_ERROR("RandomObjectDetectionMarker: vanish_duration_interval must be pair of nonegative float.");
		exit(1);
	}
	this->vanish_time_dist = std::uniform_real_distribution<float>(interval[0], interval[1]);
	
	// add menu entries
	auto menu_entry_randomize = menu_handler.insert("Randomize visibility", boost::bind( &RandomObjectDetectionMarker::processToggleRandomize, this, _1 ));
	menu_handler.setCheckState(menu_entry_randomize, MenuHandler::UNCHECKED);
	menu_handler.reApply(*server);

	// recreate Timer with new callback
	publish_timer = node_handle.createTimer(ros::Duration(period), &RandomObjectDetectionMarker::publishCallback, this);
	publish_timer.stop();
	is_publishing = false;

	// state
	this->is_randomizing = false;
	this->is_visible = true;
	this->change_visibility_timestamp = ros::Time(0.0);

	// rand genertors
	std::random_device rd;
	rand_gen = std::mt19937(rd());
}

void RandomObjectDetectionMarker::publishCallback(const ros::TimerEvent& ev)
{
	if (is_randomizing && change_visibility_timestamp < ev.current_real) {
		// change state
		change_visibility_timestamp = ev.current_real;
		if (is_visible) {
			// vanish marker detection
			is_visible = false;
			change_visibility_timestamp += ros::Duration( vanish_time_dist(rand_gen) );
			updateInteractiveMarkerColor(0.8, 0.5, 0.5);
		}
		else {
			// appear marker detection
			is_visible = true;
			change_visibility_timestamp += ros::Duration( appear_time_dist(rand_gen) );
			updateInteractiveMarkerColor(0.5, 0.8, 0.5);
		}
	}
	// publish marker if it is visible
	if (is_visible) {
		// update header and set new randomized timestamp if necessary
		detection.header.stamp = ev.current_real;
		if (is_randomizing) {
			detection.header.stamp -= ros::Duration( lag_dist(rand_gen) );
		}
		// publish message
		sweetie_bot_text_msgs::DetectionArray array;
		array.detections.push_back(detection);
		publisher.publish(array);
	}
}

void RandomObjectDetectionMarker::processToggleRandomize( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback ) {
	MenuHandler::CheckState check;
	menu_handler.getCheckState(feedback->menu_entry_id, check);
	switch (check) {
		case MenuHandler::CHECKED:
			menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
			is_randomizing = false;
			is_visible = true;
			updateInteractiveMarkerColor(0.5, 0.8, 0.5);
			break;
		case MenuHandler::UNCHECKED:
			menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
			is_randomizing = true;
			break;
	}
	menu_handler.reApply(*server);
	server->applyChanges();
}

} // namespace hmi
} // namespace sweetie_bot
