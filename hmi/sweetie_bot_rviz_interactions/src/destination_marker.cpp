#include "destination_marker.hpp"

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <sstream>
#include <math.h>

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
                                     const std::vector<std::string>& gait_type_options,
                                     const std::vector<unsigned>& n_steps_options,
                                     double duration,
                                     double nominal_height
                                    )
  : server(server),
    name(name),
    scale(scale),
    gait_type(gait_type_options[0]), // default value for gait_type
    n_steps(4), // default value for n_steps
    duration(duration),
    nominal_height(nominal_height),
    action_client( new ActionClient("move_base_action", false) )
{
  makeMenu(gait_type_options, n_steps_options);
  makeInteractiveMarker();

  server->applyChanges();
}

void DestinationMarker::actionDoneCallback(const GoalState& state, const ResultConstPtr& result)
{
  ROS_INFO_STREAM("action client done: state: " << state.toString() << " state_text: " << state.getText()
                  << " error_code: " << result->error_code << " error_string: " << result->error_string);

	action_client->cancelAllGoals();
  // Pretty print error cause
  switch (result->error_code) {
  case sweetie_bot_clop_generator::MoveBaseResult::SUCCESS:
    ROS_INFO_STREAM("Clop generator completed successfully");
    break;
  case sweetie_bot_clop_generator::MoveBaseResult::SOLUTION_NOT_FOUND:
    ROS_ERROR_STREAM("Clop generator couldn't find a solution");
    break;
  case sweetie_bot_clop_generator::MoveBaseResult::INVALID_GOAL:
    ROS_ERROR_STREAM("MoveBase goal message invalid");
    break;
  case sweetie_bot_clop_generator::MoveBaseResult::INTERNAL_ERROR:
    ROS_ERROR_STREAM("Clop generator has stopped due to internal error");
    break;
  case sweetie_bot_clop_generator::MoveBaseResult::TOLERANCE_VIOLATED:
    ROS_ERROR_STREAM("Clop generator has stopped due to violation of tolerance bounds");
    break;
  case sweetie_bot_clop_generator::MoveBaseResult::INVALID_INITIAL_POSE:
    ROS_ERROR_STREAM("Initial robot pose invalid. For walk starting you need bended knees and legs in nominal positions");
    break;
  case sweetie_bot_clop_generator::MoveBaseResult::EXECUTION_FAILED:
    ROS_ERROR_STREAM("Execution of generated movement failed");
    break;
  }
}

void DestinationMarker::actionActiveCallback()
{
  GoalState state = action_client->getState();
  ROS_INFO_STREAM(" action client active: state: " << state.toString() << " state_text: " << state.getText() );

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

  // Add end effector targets to message
  if (!ee_goal.empty())
    goal.ee_goal.insert(goal.ee_goal.begin(), ee_goal.begin(), ee_goal.end());

  goal.header.stamp = ros::Time::now();
  action_client->sendGoal(goal, boost::bind( &DestinationMarker::actionDoneCallback, this, _1, _2 ), boost::bind( &DestinationMarker::actionActiveCallback, this ));
}

void DestinationMarker::setEndEffectorTargets(const std::vector<std::string>& ee_names, unsigned frame_type)
{
  // Add end effector targets
  this->ee_goal.clear();
  for (std::string name : ee_names) {
    sweetie_bot_clop_generator::EndEffectorGoal ee_goal;
    ee_goal.name = name;
    ee_goal.frame_type = frame_type;
    ee_goal.contact = true;
    ee_goal.position_bounds = sweetie_bot_clop_generator::EndEffectorGoal::POSITION_FREE_Z;
    this->ee_goal.push_back(ee_goal);
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
  marker.pose.position.z = 0.27*scale;
  marker.scale.x = 0.01;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 1;
  marker.color.a = 0.7;

  marker.points.resize(2);
  marker.points[0].x = -0.05f*scale;
  marker.points[0].y =  0.0f;
  marker.points[0].z =  0.0f;
  marker.points[1].x =  0.05f*scale;
  marker.points[1].y =  0.0f;
  marker.points[1].z =  0.0f;

  return marker;
}

Marker DestinationMarker::makeSphereMarker()
{
  Marker marker;

	marker.type = Marker::SPHERE;
  marker.pose.position.z = 0.19*scale;
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
  marker.pose.position.z = 0.05*scale;
  marker.scale.x = 0.08*scale;
  marker.scale.y = 0.08*scale;
  marker.scale.z = 0.13*scale;
  marker.color.r = 0.8;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1;

  marker.points.resize(2);
  marker.points[0].x = 0.0f;
  marker.points[0].y = 0.0f;
  marker.points[0].z = 0.14f*scale;
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
		tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
		tf::quaternionTFToMsg(orien, control.orientation);
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
		int_marker.controls.push_back( control );
	}

  {
    InteractiveMarkerControl control;
		tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
		tf::quaternionTFToMsg(orien, control.orientation);
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
		int_marker.controls.push_back(control);

		orien = tf::Quaternion(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
		tf::quaternionTFToMsg(orien, control.orientation);
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back(control);

		orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
		tf::quaternionTFToMsg(orien, control.orientation);
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back(control);
  }

	// add marker to server
	server->insert(int_marker);
	menu_handler.apply( *server, int_marker.name );
}

void DestinationMarker::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
{
	if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
		ROS_INFO_STREAM( "Feedback from marker '" << feedback->marker_name << "' "
                     << " / control '" << feedback->control_name << "': entry id = " << feedback->menu_entry_id );

    // check if user clicked on Start walk entry
    if (feedback->menu_entry_id == start_walk_entry) {
      invokeClopGenerator(feedback->pose);
    } else {
      if (feedback->menu_entry_id == increase_by_half_duration_entry) {
        duration += 0.5;
      } else if (feedback->menu_entry_id == decrease_by_half_duration_entry) {
        duration -= 0.5;
      } else if (feedback->menu_entry_id == increase_by_tenth_duration_entry) {
        duration += 0.1;
      } else if (feedback->menu_entry_id == decrease_by_tenth_duration_entry) {
        duration -= 0.1;
      }

      menu_handler = MenuHandler(base_menu_handler); // Restore base menu handler without last entry
      menu_handler.apply(*server, name);
      // Add duration submenu again with new title
      std::stringstream duration_ss;
      duration_ss << "Duration: " << std::setprecision(1) << std::fixed << duration << "s";
      MenuHandler::FeedbackCallback processFeedback = boost::bind( &DestinationMarker::processFeedback, this, _1 );
      MenuHandler::EntryHandle duration_entry = menu_handler.insert(duration_ss.str());
      increase_by_half_duration_entry = menu_handler.insert(duration_entry, "Increase by 0.5", processFeedback);
      decrease_by_half_duration_entry = menu_handler.insert(duration_entry, "Decrease by 0.5", processFeedback);
      increase_by_tenth_duration_entry = menu_handler.insert(duration_entry, "Increase by 0.1", processFeedback);
      decrease_by_tenth_duration_entry = menu_handler.insert(duration_entry, "Decrease by 0.1", processFeedback);

      menu_handler.reApply(*server);
      server->applyChanges();
    }
  }
}

void DestinationMarker::processGaitType( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
{
	if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
		ROS_INFO_STREAM( "Feedback from marker '" << feedback->marker_name << "' "
                     << " / control '" << feedback->control_name << "': submenu \"Gait type\" select, entry id = " << feedback->menu_entry_id );
    auto it_found = gait_type_submenu.find(feedback->menu_entry_id);
    if (it_found != gait_type_submenu.end()) {
      // toggle option
      for (auto entry: gait_type_submenu) {
        menu_handler.setCheckState(entry.first, MenuHandler::UNCHECKED);
      }
      menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
      gait_type = gait_type_submenu[feedback->menu_entry_id];

      menu_handler.reApply(*server);
      server->applyChanges();
    }
  }
}

void DestinationMarker::processStepsNum( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
{
	if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
		ROS_INFO_STREAM( "Feedback from marker '" << feedback->marker_name << "' "
                     << " / control '" << feedback->control_name << "': submenu \"Steps number\" select, entry id = " << feedback->menu_entry_id );
    auto it_found = n_steps_submenu.find(feedback->menu_entry_id);
    if (it_found != n_steps_submenu.end()) {
      // toggle option
      for (auto entry: n_steps_submenu) {
        menu_handler.setCheckState(entry.first, MenuHandler::UNCHECKED);
      }
      menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
      n_steps = n_steps_submenu[feedback->menu_entry_id];

      menu_handler.reApply(*server);
      server->applyChanges();
    }
  }
}

void DestinationMarker::makeMenu(const std::vector<std::string>& gait_type_options, const std::vector<unsigned>& n_steps_options)
{
  MenuHandler::FeedbackCallback processFeedback = boost::bind( &DestinationMarker::processFeedback, this, _1 );

  start_walk_entry = menu_handler.insert("Start walk", processFeedback);
  MenuHandler::EntryHandle gait_type_entry = menu_handler.insert("Gait type");
  MenuHandler::FeedbackCallback processGaitType = boost::bind( &DestinationMarker::processGaitType, this, _1 );
  for (auto gait_type_option : gait_type_options) {
    MenuHandler::EntryHandle handle = menu_handler.insert(gait_type_entry, gait_type_option, processGaitType);
    if (gait_type_option == gait_type)
      menu_handler.setCheckState(handle, MenuHandler::CHECKED);
    else
      menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
    gait_type_submenu.emplace(handle, gait_type_option);
  }
  MenuHandler::EntryHandle n_steps_entry = menu_handler.insert("Steps number");
  MenuHandler::FeedbackCallback processStepsNum = boost::bind( &DestinationMarker::processStepsNum, this, _1 );
  for (auto n_steps_option : n_steps_options) {
    MenuHandler::EntryHandle handle = menu_handler.insert(n_steps_entry, std::to_string(n_steps_option), processStepsNum);
    if (n_steps_option == n_steps)
      menu_handler.setCheckState(handle, MenuHandler::CHECKED);
    else
      menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
    n_steps_submenu.emplace(handle, n_steps_option);
  }
  base_menu_handler = menu_handler; // Save base menu handler without last entry
  std::stringstream duration_ss;
  duration_ss << "Duration: " << std::setprecision(1) << std::fixed << duration << "s";
  MenuHandler::EntryHandle duration_entry = menu_handler.insert(duration_ss.str());
  increase_by_half_duration_entry = menu_handler.insert(duration_entry, "Increase by 0.5", processFeedback);
  decrease_by_half_duration_entry = menu_handler.insert(duration_entry, "Decrease by 0.5", processFeedback);
  increase_by_tenth_duration_entry = menu_handler.insert(duration_entry, "Increase by 0.1", processFeedback);
  decrease_by_tenth_duration_entry = menu_handler.insert(duration_entry, "Decrease by 0.1", processFeedback);
}

} // namespace hmi
} // namespace sweetie_bot
