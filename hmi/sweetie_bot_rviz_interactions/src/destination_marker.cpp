#include "destination_marker.hpp"

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Header.h>

#include <sweetie_bot_clop_generator/MoveBaseGoal.h>
#include <sweetie_bot_clop_generator/EndEffectorGoal.h>

using namespace visualization_msgs;
using namespace interactive_markers;

namespace sweetie_bot {
namespace hmi {

DestinationMarker::DestinationMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                                     const std::string& name,
                                     double scale,
                                     const std::string& gait_type,
                                     int n_steps,
                                     double duration,
                                     double nominal_height
                                    )
  : server(server),
    name(name),
    scale(scale),
    gait_type(gait_type),
    n_steps(n_steps),
    duration(duration),
    nominal_height(nominal_height),
    action_client( new ActionClient("move_base_action", false) )
{
  makeMenu();
  makeInteractiveMarker();

  server->applyChanges();
}

void DestinationMarker::actionDoneCallback(const GoalState& state, const ResultConstPtr& result)
{
  ROS_INFO_STREAM("action client done: state: " << state.toString() << " state_text: " << state.getText()
                  << " error_code: " << result->error_code << " error_string: " << result->error_string);

	action_client->cancelAllGoals();

  menu_handler.reApply(*server);
  server->applyChanges();
}

void DestinationMarker::actionActiveCallback()
{
  GoalState state = action_client->getState();
  ROS_INFO_STREAM(" action client active: state: " << state.toString() << " state_text: " << state.getText() );

  menu_handler.reApply(*server);
  server->applyChanges();
}

// TODO: Split this method into several
void DestinationMarker::invokeClopGenerator( const geometry_msgs::Pose& base_goal )
{
  ROS_INFO_STREAM("Invoking clop generator by interactive marker");

  sweetie_bot_clop_generator::MoveBaseGoal goal;

  goal.header = std_msgs::Header();
  goal.header.frame_id = "odom_combined";
  goal.gait_type = gait_type;
  goal.duration = duration;
  goal.n_steps = n_steps;
  goal.base_goal = base_goal;
  goal.base_goal.position.z = nominal_height;

  goal.visualize_only = false;
  goal.execute_only = false;

  goal.position_tolerance = 0.07;
  goal.orientation_tolerance = 0.50;

  // Add end effector targets
  std::vector<std::string> ee_names;
  ee_names.push_back("leg1");
  ee_names.push_back("leg2");
  ee_names.push_back("leg3");
  ee_names.push_back("leg4");
  for (std::string name : ee_names) {
    sweetie_bot_clop_generator::EndEffectorGoal ee_goal;
    ee_goal.name = name;
    ee_goal.frame_type = sweetie_bot_clop_generator::EndEffectorGoal::NOMINAL_POSE;
    ee_goal.contact = true;
    ee_goal.position_bounds = sweetie_bot_clop_generator::EndEffectorGoal::POSITION_FREE_Z;
    goal.ee_goal.push_back(ee_goal);
  }

  goal.header.stamp = ros::Time::now();
  action_client->sendGoal(goal, boost::bind( &DestinationMarker::actionDoneCallback, this, _1, _2 ), boost::bind( &DestinationMarker::actionActiveCallback, this ));
}

void DestinationMarker::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
{
	if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
		ROS_INFO_STREAM( "Feedback from marker '" << feedback->marker_name << "' "
                     << " / control '" << feedback->control_name << "': menu \"Start walk\" select, entry id = " << feedback->menu_entry_id );

    invokeClopGenerator(feedback->pose);
  }
}

Marker DestinationMarker::makePointMarker()
{
  Marker marker;

  marker.type = Marker::POINTS;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.color.g = 1.0;
  marker.color.a = 1;

  marker.points.resize(1);
  marker.points[0].x = 0.0f;
  marker.points[0].y = 0.0f;
  marker.points[0].z = 0.0f;

  return marker;
}

Marker DestinationMarker::makeArrowMarker()
{
  Marker marker;

  marker.type = Marker::ARROW;
  marker.pose.position.z = 0.27;
  marker.scale.x = 0.01;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 1;
  marker.color.a = 0.7;

  marker.points.resize(2);
  marker.points[0].x = -0.05f;
  marker.points[0].y =  0.0f;
  marker.points[0].z =  0.0f;
  marker.points[1].x =  0.05f;
  marker.points[1].y =  0.0f;
  marker.points[1].z =  0.0f;

  return marker;
}

Marker DestinationMarker::makeSphereMarker()
{
  Marker marker;

	marker.type = Marker::SPHERE;
  marker.pose.position.z = 0.19;
	marker.scale.x = 0.08*scale;
	marker.scale.y = 0.08*scale;
	marker.scale.z = 0.08*scale;
	marker.color.r = 0.8;
	marker.color.g = 0.5;
	marker.color.b = 0.5;
	marker.color.a = 1;

  return marker;
}

Marker DestinationMarker::makeConeMarker()
{
  Marker marker;

  marker.type = Marker::ARROW;
  marker.pose.position.z = 0.05;
  marker.scale.x = 0.08;
  marker.scale.y = 0.08;
  marker.scale.z = 0.13;
  marker.color.r = 0.8;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1;

  marker.points.resize(2);
  marker.points[0].x = 0.0f;
  marker.points[0].y = 0.0f;
  marker.points[0].z = 0.14f;
  marker.points[1].x = 0.0f;
  marker.points[1].y = 0.0f;
  marker.points[1].z = 0.0f;

  return marker;
}

void DestinationMarker::makeInteractiveMarker()
{
  InteractiveMarker int_marker;

  //header setup
	int_marker.header.frame_id = "odom_combined";
  int_marker.pose.position.x = 0.4;
	int_marker.scale = 0.15*std::min(scale, 1.0);
	int_marker.name = name;
	int_marker.description = name;

  // insert base_link model
	{
		InteractiveMarkerControl control;
		control.always_visible = true;
		control.markers.push_back( makeSphereMarker() );
		control.markers.push_back( makeConeMarker() );
		control.markers.push_back( makeArrowMarker() );
		control.markers.push_back( makePointMarker() );
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
		int_marker.controls.push_back( control );
	}

  {
    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
		int_marker.controls.push_back(control);
  }

	// add marker to server
	server->insert(int_marker);
	menu_handler.apply( *server, int_marker.name );
}

void DestinationMarker::makeMenu()
{
  MenuHandler::FeedbackCallback processFeedback = boost::bind( &DestinationMarker::processFeedback, this, _1 );

  menu_handler.insert("Start walk", processFeedback);
}

} // namespace hmi
} // namespace sweetie_bot
