#include "generic_pose_marker.hpp"

namespace sweetie_bot {
namespace hmi {

GenericPoseMarker::GenericPoseMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                                     ros::NodeHandle node_handle
                                     )
  : PoseMarkerBase(server, ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("pose", 1), node_handle),
    action_client( new ActionClient("set_operational_action", false) )
{
  node_handle.getParam("resources", resources);
  node_handle.getParam("frames", frames);
  node_handle.getParam("select_only_one_resource", select_only_one_resource);

  makeMenu();
  makeInteractiveMarker(makeMarkerBody, boost::bind( &GenericPoseMarker::processFeedback, this, boost::placeholders::_1 ));

  server->applyChanges();
}

void GenericPoseMarker::setOperational(bool is_operational)
{
  // check connection
  if (!action_client->isServerConnected()) {
    if (!action_client->waitForServer(ros::Duration(0.3, 0))) {
      ROS_ERROR("SetOperational action server is unavailible");
      return;
    }
  }

  if (is_operational) {
    // send new goal, old goal will be peempted
    ROS_DEBUG("setOperational(true)");

    // form goal message
    Goal goal;
    goal.operational = true;
    for( const auto& pair : resources_entry_map ) {
			MenuHandler::CheckState check;
			menu_handler.getCheckState( pair.first, check );
			if (check == MenuHandler::CHECKED) goal.resources.push_back(pair.second);
		}


    // send goal to server
    action_client->sendGoal(goal, boost::bind( &GenericPoseMarker::actionDoneCallback, this, boost::placeholders::_1, boost::placeholders::_2 ), boost::bind( &GenericPoseMarker::actionActiveCallback, this ));
    // return true if goal is being processed
    GoalState state = action_client->getState();
    this->is_operational = !state.isDone();

    if (this->is_operational) {
      this->changeColor(0.0f, 0.39f, 0.0f);
    }
  }
  else {
    // assume that server is in operational state
    ROS_DEBUG("setOperational(false)");

    GoalState state = action_client->getState();
    if (!state.isDone()) action_client->cancelGoal();

    this->is_operational = false;

    this->changeColor(0.8f, 0.5f, 0.5f);
  }
}

GenericPoseMarker::~GenericPoseMarker()
{
  pose_publisher.shutdown();
  action_client.reset();
}

void GenericPoseMarker::actionDoneCallback(const GoalState& state, const ResultConstPtr& result)
{
  ROS_INFO_STREAM("action client done: state: " << state.toString() << " state_text: " << state.getText()
                  << " error_code: " << result->error_code << " error_string: " << result->error_string);

  setOperational(false);

  menu_handler.setCheckState(set_operational_entry, MenuHandler::UNCHECKED);
  menu_handler.reApply(*server);
  server->applyChanges();
}

void GenericPoseMarker::actionActiveCallback()
{
  GoalState state = action_client->getState();
  ROS_INFO_STREAM(" action client active: state: " << state.toString() << " state_text: " << state.getText() );

  menu_handler.setCheckState(set_operational_entry, MenuHandler::CHECKED);
  menu_handler.reApply(*server);
  server->applyChanges();
}

void GenericPoseMarker::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
{
  if ( feedback->marker_name != name ) return;

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

      if (is_pose_publishing && is_operational) {
        geometry_msgs::PoseStamped pose_stamped;

        pose_stamped.header = feedback->header;
        pose_stamped.pose = feedback->pose;
        pose_publisher.publish(pose_stamped);
      }
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu select, entry id: " << feedback->menu_entry_id );

      // check user toggled set operational meny entry
      if (feedback->menu_entry_id == set_operational_entry) {
        // cahnge server mode
        GoalState state = action_client->getState();
        if (state.isDone()) {
          setOperational(true);
        }
        else {
          setOperational(false);
        }
      }
      // check user toggled publish pose meny entry
      else if (feedback->menu_entry_id == publish_pose_entry) {
        // toggle option
        MenuHandler::CheckState check;
        menu_handler.getCheckState(feedback->menu_entry_id, check);
        switch (check) {
          case MenuHandler::CHECKED:
            menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
            setPublishPose(false);
            break;
          case MenuHandler::UNCHECKED:
            menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
            setPublishPose(true);
            if (is_operational) {
              // publish current pose
              geometry_msgs::PoseStamped pose_stamped;
              pose_stamped.header = feedback->header;
              pose_stamped.pose = feedback->pose;
              pose_publisher.publish(pose_stamped);
            }
        }
      }
      else {
        // check if user toggled a resource
        auto it_found = resources_entry_map.find(feedback->menu_entry_id);
        if (it_found != resources_entry_map.end()) {
          // toggle option
          MenuHandler::CheckState check;
          menu_handler.getCheckState(feedback->menu_entry_id, check);
          switch (check) {
          case MenuHandler::CHECKED:
            menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
            break;
          case MenuHandler::UNCHECKED:
            menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
            if (select_only_one_resource) {
              for(auto it = resources_entry_map.begin(); it != resources_entry_map.end(); it++)
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

void GenericPoseMarker::makeMenu()
{
  MenuHandler::FeedbackCallback processFeedback = boost::bind( &GenericPoseMarker::processFeedback, this, boost::placeholders::_1 );

  set_operational_entry = menu_handler.insert( "OPERATIONAL", processFeedback);
  menu_handler.setCheckState(set_operational_entry, MenuHandler::UNCHECKED);
  for ( const std::string& res : resources ) {
		MenuHandler::EntryHandle handle = menu_handler.insert( "  " + res, processFeedback );
		if (!select_only_one_resource) menu_handler.setCheckState(handle, MenuHandler::CHECKED);
		else menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
		resources_entry_map.emplace(handle, res);
	}
  normalize_pose_entry = menu_handler.insert( "Normalize pose", boost::bind( &GenericPoseMarker::processNormalize, this, boost::placeholders::_1 ));
  publish_pose_entry = menu_handler.insert( "Publish pose", processFeedback );
  menu_handler.setCheckState(publish_pose_entry, MenuHandler::CHECKED);
  enable_6DOF_entry = menu_handler.insert( "Enable 6-DOF", boost::bind( &GenericPoseMarker::processEnable6DOF, this, boost::placeholders::_1 ));
  menu_handler.setCheckState(enable_6DOF_entry, MenuHandler::CHECKED);
  if (!frames.empty()) {
    frames_submenu_entry = menu_handler.insert("Move to frame");
    for ( const std::string& frame : frames ) {
      menu_handler.insert(frames_submenu_entry, frame, boost::bind( &GenericPoseMarker::processMoveToFrame, this, boost::placeholders::_1 ));
    }
  }


  menu_handler.reApply(*server);
  server->applyChanges();
}

} // namespace hmi
} // namespace sweetie_bot
