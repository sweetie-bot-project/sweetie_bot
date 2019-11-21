#include "stance_pose_marker.hpp"

#include <boost/thread/mutex.hpp>

namespace sweetie_bot {
namespace hmi {


StancePoseMarker::StancePoseMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                                   visualization_msgs::Marker (*makeMarkerBody)(const double scale),
                                   const std::string& name,
                                   double scale,
                                   const std::string& marker_home_frame,
                                   double normalized_z_level
                                  )
  : PoseMarker(server, name, scale, marker_home_frame, normalized_z_level),
      pose_pub(ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("stance_pose", 1)),
      action_client( new ActionClient("stance_set_operational_action", false) )
{
  makeMenu();
  makeInteractiveMarker(makeMarkerBody, boost::bind( &StancePoseMarker::processFeedback, this, _1 ));

  if (marker_home_frame != "")
    moveToFrame(marker_home_frame);

	server->applyChanges();
}

StancePoseMarker::~StancePoseMarker()
{
	pose_pub.shutdown();
	action_client.reset();
}

void StancePoseMarker::actionDoneCallback(const GoalState& state, const ResultConstPtr& result)
{
	ROS_INFO_STREAM("action client done: state: " << state.toString() << " state_text: " << state.getText()
                  << " error_code: " << result->error_code << " error_string: " << result->error_string);

  is_operational = false;
	//action_client->cancelAllGoals();

	menu_handler.setCheckState(set_operational_entry, MenuHandler::UNCHECKED);
	menu_handler.reApply(*server);
	server->applyChanges();
}

void StancePoseMarker::actionActiveCallback()
{
	GoalState state = action_client->getState();
	ROS_INFO_STREAM(" action client active: state: " << state.toString() << " state_text: " << state.getText() );

	menu_handler.setCheckState(set_operational_entry, MenuHandler::CHECKED);
	menu_handler.reApply(*server);
	server->applyChanges();
}

bool StancePoseMarker::setOperational(bool is_operational)
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
    for (auto it=resource_markers.begin(); std::distance(resource_markers.begin(), it) < 4; ++it) {
      auto &limb_marker = *it;
      if (limb_marker->getState() == LimbPoseMarker::LimbState::SUPPORT && limb_marker->isControlled()) {
        goal.resources.push_back((*it)->getResourceName());
      }
    }

		// send goal to server
		action_client->sendGoal(goal, boost::bind( &StancePoseMarker::actionDoneCallback, this, _1, _2 ), boost::bind( &StancePoseMarker::actionActiveCallback, this ));
		// return true if goal is being processed
		GoalState state = action_client->getState();
		this->is_operational = !state.isDone();
		return this->is_operational;
	}
	else {
		// assume that server is in operational state
		ROS_INFO("setOperational(false)");

		GoalState state = action_client->getState();
		if (!state.isDone()) action_client->cancelGoal();

    this->is_operational = false;
		return this->is_operational;
	}
}

void StancePoseMarker::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
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
			if (publish_pose && is_operational) {
				geometry_msgs::PoseStamped pose_stamped;

				pose_stamped.header = feedback->header;
				pose_stamped.pose = feedback->pose;
				pose_pub.publish(pose_stamped);
			}
			break;

		case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
			ROS_INFO_STREAM( s.str() << ": menu select, entry id: " << feedback->menu_entry_id );

			// check user toggled set operational meny entry
			if (feedback->menu_entry_id == set_operational_entry) {
				// cahnge server mode
				GoalState state = action_client->getState();
				if (state.isDone()) {
					// move marker to prescribed frame
					if (marker_home_frame != "") {
						moveToFrame(marker_home_frame);
					}
          // and only after make it operational
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
						publish_pose = false;
						break;
					case MenuHandler::UNCHECKED:
						menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
						publish_pose = true;
            if (is_operational) {
              // publish current pose
              geometry_msgs::PoseStamped pose_stamped;
              pose_stamped.header = feedback->header;
              pose_stamped.pose = feedback->pose;
              pose_pub.publish(pose_stamped);
            }
				}
      }
			else {
				// check if user toggled resource control
				auto it_found_control = resources_control_entry_map.find(feedback->menu_entry_id);
				if (it_found_control != resources_control_entry_map.end()) {
          // calculate resource id
          int resource_id = std::distance(resources_control_entry_map.begin(), it_found_control);
					// toggle option
					MenuHandler::CheckState check;
					menu_handler.getCheckState(feedback->menu_entry_id, check);
					switch (check) {
						case MenuHandler::CHECKED:
              if (resource_id < resource_markers.size()) {
                std::shared_ptr<LimbPoseMarker>& res_marker = resource_markers[resource_id];
                res_marker->setControlState(false);
                this->setOperational(false);
                if (resource_id >= 4) {
                  // for all limbs exept legs we control markers visibility by controlled limb state
                  res_marker->changeVisibility(false); // hide bounded limb marker
                }
                rebuildMenu();
              }
							menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
							break;
						case MenuHandler::UNCHECKED:
              if (resource_id < resource_markers.size()) {
                std::shared_ptr<LimbPoseMarker>& res_marker = resource_markers[resource_id];
                res_marker->setControlState(true);
                this->setOperational(false);
                if (resource_id >= 4) {
                  // for all limbs we control markers visibility by controlled limb state
                  res_marker->changeVisibility(true); // show bounded limb marker
                  res_marker->moveToFrame(res_marker->getMarkerHomeFrame()); // also move it to its place
                }
                rebuildMenu();
              }
              menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
							break;
					}
				}
        // check if user toggled resource state
				auto it_found_state = resources_state_entry_map.find(feedback->menu_entry_id);
				if (it_found_state != resources_state_entry_map.end()) {
          // calculate resource id
          int resource_id = std::distance(resources_state_entry_map.begin(), it_found_state);
					// toggle option
					MenuHandler::CheckState check;
					menu_handler.getCheckState(feedback->menu_entry_id, check);
					switch (check) {
						case MenuHandler::CHECKED:
              if (resource_id < resource_markers.size()) {
                std::shared_ptr<LimbPoseMarker>& res_marker = resource_markers[resource_id];
                res_marker->changeVisibility(false); // hide bounded limb marker
                res_marker->setState(LimbPoseMarker::LimbState::SUPPORT);
                this->setOperational(false);
                rebuildMenu();
              }
							menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
							break;
						case MenuHandler::UNCHECKED:
              if (resource_id < resource_markers.size()) {
                std::shared_ptr<LimbPoseMarker>& res_marker = resource_markers[resource_id];
                res_marker->changeVisibility(true); // show bounded limb marker
                res_marker->moveToFrame(res_marker->getMarkerHomeFrame()); // also move it to its place
                res_marker->setState(LimbPoseMarker::LimbState::FREE);
                this->setOperational(false);
                rebuildMenu();
              }
              menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
							break;
					}
				}
			}
			break;
	}

  // apply changes if controller is operational
	menu_handler.reApply(*server);
	server->applyChanges();
}

void StancePoseMarker::processNormalizeLegs( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
		ROS_INFO_STREAM( "Feedback from marker '" << feedback->marker_name << "' "
                     << " / control '" << feedback->control_name << "': menu \"Normalize legs\" select, entry id = " << feedback->menu_entry_id );

    if (resource_markers.empty()) return;

    for (auto it=resource_markers.begin(); std::distance(resource_markers.begin(), it) < 4; ++it) {
      auto marker_ptr = *it;
      if (marker_ptr->isVisible()) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = feedback->header;
        // update marker pose
        marker_ptr->reloadMarker();
        pose_stamped.pose = marker_ptr->getInteractiveMarker().pose;
        // normalize marker
        marker_ptr->normalize(pose_stamped);
        // publish new pose
        if (marker_ptr->isPosePublishing() && marker_ptr->isOperational()) {
          marker_ptr->getPosePublisher().publish(pose_stamped);
        }
      }
    }

    server->applyChanges();
  }
}


void StancePoseMarker::processMoveAllToHome( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
		ROS_INFO_STREAM( "Feedback from marker '" << feedback->marker_name << "' "
                     << " / control '" << feedback->control_name << "': menu \"Move all to home\" select, entry id = " << feedback->menu_entry_id );

    // move stance marker to home
    moveToFrame(marker_home_frame);
    // move limbs markers to home
    for (auto& marker : resource_markers) {
      marker->moveToFrame(marker->getMarkerHomeFrame());
    }

    server->applyChanges();
  }
}
void StancePoseMarker::makeMenu()
{
  MenuHandler::FeedbackCallback processFeedback = boost::bind( &StancePoseMarker::processFeedback, this, _1 );

	set_operational_entry = menu_handler.insert( "OPERATIONAL", processFeedback);
	menu_handler.setCheckState(set_operational_entry, MenuHandler::UNCHECKED);
  int marker_idx = 0;
  for (auto it=resource_markers.begin(); it != resource_markers.end(); ++it, ++marker_idx) {
    MenuHandler::EntryHandle handle;
    auto limb_marker_ptr = *it;

    std::string control_marker_msg = "(CONTROLLED)";
    if (limb_marker_ptr->isControlled()) {
      control_marker_msg = "(CONTROLLED)";
    } else {
      control_marker_msg = "(UNCONTROLLED)";
    }
    handle = menu_handler.insert( "  " + limb_marker_ptr->getResourceName() + " " + control_marker_msg, processFeedback);
    if (limb_marker_ptr->isControlled()) {
      menu_handler.setCheckState(handle, MenuHandler::CHECKED);
    } else {
      menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
    }
		resources_control_entry_map.emplace(handle, limb_marker_ptr->getResourceName());

    if (marker_idx < 4) {
      std::string resource_state_msg = "SUPPORT";
      if (limb_marker_ptr->getState() == LimbPoseMarker::LimbState::FREE) {
        resource_state_msg = "FREE";
      } else {
        resource_state_msg = "SUPPORT";
      }
      handle = menu_handler.insert( " -- limb state: " + resource_state_msg, processFeedback);
      if (limb_marker_ptr->getState() == LimbPoseMarker::LimbState::FREE) {
        menu_handler.setCheckState(handle, MenuHandler::CHECKED);
      } else {
        menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
      }
      resources_state_entry_map.emplace(handle, limb_marker_ptr->getResourceName());
    }
	}
	normalize_pose_entry = menu_handler.insert( "Move all to home", boost::bind( &StancePoseMarker::processMoveAllToHome, this, _1 ));
	normalize_pose_entry = menu_handler.insert( "Normalize legs", boost::bind( &StancePoseMarker::processNormalizeLegs, this, _1 ));
	normalize_pose_entry = menu_handler.insert( "Normalize pose", boost::bind( &StancePoseMarker::processNormalize, this, _1, pose_pub, publish_pose ));
	publish_pose_entry = menu_handler.insert( "Publish pose", processFeedback);
	menu_handler.setCheckState(publish_pose_entry, MenuHandler::CHECKED);
	enable_6DOF_entry = menu_handler.insert( "Enable 6-DOF", boost::bind( &StancePoseMarker::processEnable6DOF, this, _1 ));
	menu_handler.setCheckState(enable_6DOF_entry, MenuHandler::UNCHECKED);
  menu_handler.insert("Move to home frame", boost::bind( &StancePoseMarker::processMoveToHomeFrame, this, _1 ));

  menu_handler.reApply(*server);
  server->applyChanges();
}

} // namespace hmi
} // namespace sweetie_bot
