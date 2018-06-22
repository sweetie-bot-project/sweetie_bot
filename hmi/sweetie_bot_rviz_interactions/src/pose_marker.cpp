#include <ros/ros.h>

#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <actionlib/client/simple_action_client.h>
#include <sweetie_bot_resource_control_msgs/SetOperationalAction.h>

using namespace visualization_msgs;
using namespace interactive_markers;


// Action type definitions
ACTION_DEFINITION(sweetie_bot_resource_control_msgs::SetOperationalAction);	
typedef actionlib::SimpleActionClient<sweetie_bot_resource_control_msgs::SetOperationalAction> ActionClient;
typedef actionlib::SimpleClientGoalState GoalState;

// COMPONENT INTERFACE

// CONNECTIONS
// action server 
std::shared_ptr<ActionClient> action_client;
// publisers 
ros::Publisher pose_pub;
// tf listener
tf2_ros::Buffer tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

// PARAMETERS
// node name 
std::string node_name;
// marker sacle parameter
double scale = 1.0;
// resources list: corresponding menu items will be displayed in context menu
std::vector<std::string> resources = { "leg1", "leg2", "leg3", "leg4" };
// allow select only one resource
bool resources_select_only_one = false;
// Home frame to place marker on start operation. Leave empty to 
std::string marker_home_frame;
// Basic z level. When `Normilize pose` command is executed the marker is placed parallel Oxy plane on normalized_z_level heigh over it.
double normalized_z_level = 0.0;

// COMPONENT STATE
// interactive marker server
std::shared_ptr<InteractiveMarkerServer> server;
// menu
MenuHandler menu_handler;
// menu index
struct {
	MenuHandler::EntryHandle set_operational;
	std::map<MenuHandler::EntryHandle, std::string> resources;
	MenuHandler::EntryHandle normalize_pose;
	MenuHandler::EntryHandle publish_pose;
} menu_entries;
// publish_pose flag
bool publish_pose = true;

void actionDoneCallback(const GoalState& state, const ResultConstPtr& result)
{
	ROS_INFO_STREAM("action client done: state: " << state.toString() << " state_text: " << state.getText() 
			<< " error_code: " << result->error_code << " error_string: " << result->error_string);

	action_client->cancelAllGoals();
	
	menu_handler.setCheckState(menu_entries.set_operational, MenuHandler::UNCHECKED);
	menu_handler.reApply(*server);
	server->applyChanges();
}

void actionActiveCallback()
{
	GoalState state = action_client->getState();
	ROS_INFO_STREAM(" action client active: state: " << state.toString() << " state_text: " << state.getText() );
	
	menu_handler.setCheckState(menu_entries.set_operational, MenuHandler::CHECKED);
	menu_handler.reApply(*server);
	server->applyChanges();
}


bool setOperational(bool is_operational) 
{
	// check connection
	if (!action_client->isServerConnected()) {
		if (!action_client->waitForServer(ros::Duration(0.3, 0))) {
			ROS_ERROR("SetOperational action server is unavailible");
			return false;
		}
	}

	if (is_operational) {
		// send new goal, old goal will be peempted
		ROS_INFO("setOperational(true)");

		// form goal message
		Goal goal;
		goal.operational = true;
		for( const auto& pair : menu_entries.resources ) {
			MenuHandler::CheckState check;
			menu_handler.getCheckState( pair.first, check );
			if (check == MenuHandler::CHECKED) goal.resources.push_back(pair.second);
		}

		// send goal to server
		action_client->sendGoal(goal, &actionDoneCallback, &actionActiveCallback);
		// return true if goal is being processed
		GoalState state = action_client->getState();
		return !state.isDone();
	}
	else {
		// assume that server is in operational state
		ROS_INFO("setOperational(false)");
	
		GoalState state = action_client->getState();
		if (!state.isDone()) action_client->cancelGoal();

		return false;
	}
}

// event handler
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	if ( feedback->marker_name != node_name ) return;

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
			if (publish_pose) {
				geometry_msgs::PoseStamped pose_stamped;

				pose_stamped.header = feedback->header;
				pose_stamped.pose = feedback->pose;
				pose_pub.publish(pose_stamped);
			}
			break;

		case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
			ROS_INFO_STREAM( s.str() << ": menu select, entry id: " << feedback->menu_entry_id );

			// check user toggled set operational meny entry
			if (feedback->menu_entry_id == menu_entries.set_operational) {
				// cahnge server mode
				GoalState state = action_client->getState();
				if (state.isDone()) {
					setOperational(true);
					// move market to prescribed frame
					if (marker_home_frame != "") {
						try {
							// get transform
							geometry_msgs::TransformStamped T;
							T = tf_buffer.lookupTransform("odom_combined", marker_home_frame, ros::Time(0));
							// convert to pose
							geometry_msgs::Pose pose;
							pose.position.x = T.transform.translation.x;
							pose.position.y = T.transform.translation.y;
							pose.position.z = T.transform.translation.z;
							pose.orientation = T.transform.rotation;
							// set pose
							server->setPose(node_name, pose);
						}
						catch (tf2::TransformException &ex) {
							ROS_WARN("lookupTransform: %s", ex.what());
						}
					}
				}
				else {
					setOperational(false);
				}
			}
			// check for pose normalization command
			else if (feedback->menu_entry_id == menu_entries.normalize_pose) {
				geometry_msgs::PoseStamped pose_stamped;
				pose_stamped.header = feedback->header;
				pose_stamped.pose = feedback->pose;
				// normilize pose
				pose_stamped.pose.position.z = normalized_z_level;
				tf::Quaternion orien(0.0, 0.0, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w);
				orien.normalize();
				tf::quaternionTFToMsg(orien, pose_stamped.pose.orientation);
				// set pose of marker
				server->setPose(node_name, pose_stamped.pose);
				// publish new pose
				if (publish_pose) pose_pub.publish(pose_stamped);
			}
			// check user toggled publish pose meny entry
			else if (feedback->menu_entry_id == menu_entries.publish_pose) {
				// toggle option
				MenuHandler::CheckState check;
				menu_handler.getCheckState(feedback->menu_entry_id, check);
				switch (check) {
					case MenuHandler::CHECKED:
						menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED); 
						publish_pose = false;
						break;
					case MenuHandler::UNCHECKED:
						menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED); 
						publish_pose = true;
						// publish current pose
						geometry_msgs::PoseStamped pose_stamped;
						pose_stamped.header = feedback->header;
						pose_stamped.pose = feedback->pose;
						pose_pub.publish(pose_stamped);
				}
			}
			else {
				// check if user toggled resource
				auto it_found = menu_entries.resources.find(feedback->menu_entry_id);
				if (it_found != menu_entries.resources.end()) {
					// toggle option
					MenuHandler::CheckState check;
					menu_handler.getCheckState(feedback->menu_entry_id, check);
					switch (check) {
						case MenuHandler::CHECKED:
							menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
							break;
						case MenuHandler::UNCHECKED:
							menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
							if (resources_select_only_one) {
								for(auto it = menu_entries.resources.begin(); it != menu_entries.resources.end(); it++)
									if (it != it_found) menu_handler.setCheckState(it->first, MenuHandler::UNCHECKED);
							}
							break;
					}
					// apply changes if controller is operational
					GoalState state = action_client->getState();
					if (!state.isDone()) setOperational(true);
				}
			}
			break;
	}

	menu_handler.reApply(*server);
	server->applyChanges();
}


// Display platform shape as controlled body
Marker makeBody()
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

void make6DofMarker()
{
	InteractiveMarker int_marker;
	//header setup
	int_marker.header.frame_id = "odom_combined";
	//int_marker.pose.position = ...;
	int_marker.scale = 0.1;
	int_marker.name = node_name;
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
	menu_handler.apply( *server, int_marker.name );
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_marker");
	ros::NodeHandle n;

	// get node name to distigush markers	
	node_name = ros::this_node::getName();
	size_t slash_pos = node_name.find_last_of('/');
	if (slash_pos != std::string::npos) node_name = node_name.substr(slash_pos + 1);

	// get parameters
	ros::param::get("~scale", scale);
	ros::param::get("~resources", resources);
	ros::param::get("~resources_select_only_one", resources_select_only_one);
	ros::param::get("~marker_home_frame", marker_home_frame);
	ros::param::get("~normalized_z_level", normalized_z_level);

	//actionlib client
	action_client.reset( new ActionClient("set_operational_action", false) );

	//pose publishing
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 1);

	//tf listener
	tf_listener.reset( new tf2_ros::TransformListener(tf_buffer) );

	// marker server
	server.reset( new InteractiveMarkerServer(ros::this_node::getNamespace(),"",false) );
	ros::Duration(0.1).sleep();

	// make menu
	menu_entries.set_operational = menu_handler.insert( "OPERATIONAL", &processFeedback );
	menu_handler.setCheckState(menu_entries.set_operational, MenuHandler::UNCHECKED);
	for ( const std::string& res : resources ) {
		MenuHandler::EntryHandle handle = menu_handler.insert( "  " + res, &processFeedback );
		if (!resources_select_only_one) menu_handler.setCheckState(handle, MenuHandler::CHECKED);
		else menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
		menu_entries.resources.emplace(handle, res);
	}
	menu_entries.normalize_pose = menu_handler.insert( "Normalize pose", &processFeedback );
	menu_entries.publish_pose = menu_handler.insert( "Publish pose", &processFeedback );
	menu_handler.setCheckState(menu_entries.publish_pose, MenuHandler::CHECKED); publish_pose = true;

	// create marker
	make6DofMarker();
	server->applyChanges();

	ROS_INFO("pose_marker is started!");

	// main loop()
	ros::spin();
	// shutdown
	server.reset();
	action_client.reset();
}
