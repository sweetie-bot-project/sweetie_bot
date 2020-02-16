#include "stance_pose_marker.hpp"

#include <boost/thread/mutex.hpp>

namespace sweetie_bot {
namespace hmi {


StancePoseMarker::StancePoseMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                                   visualization_msgs::Marker (*makeMarkerBody)(const double scale),
                                   ros::NodeHandle node_handle
                                  )
  : PoseMarker(server, node_handle),
    action_client( new ActionClient("stance_set_operational_action", false) ),
    pose_pub(ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("stance_pose", 1))
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

  setOperational(false);
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

void StancePoseMarker::setOperational(bool is_operational)
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
    for (auto it = resource_markers.begin(); std::distance(resource_markers.begin(), it) < 4; ++it) {
      auto &limb_marker = *it;

      if (limb_marker->getState() == LimbPoseMarker::LimbState::INACTIVE) {
        goal.resources.push_back((*it)->getResourceName());
      }
    }

		// send goal to server
		action_client->sendGoal(goal, boost::bind( &StancePoseMarker::actionDoneCallback, this, _1, _2 ), boost::bind( &StancePoseMarker::actionActiveCallback, this ));
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
				// check if user toggled resource
				auto it_found = resources_entry_map.find(feedback->menu_entry_id);
				if (it_found != resources_entry_map.end()) {
          // calculate resource id
          int resource_id = std::distance(resources_entry_map.begin(), it_found);
					// toggle option
					MenuHandler::CheckState check;
					menu_handler.getCheckState(feedback->menu_entry_id, check);
					switch (check) {
						case MenuHandler::CHECKED:
              if (resource_id < resource_markers.size()) {
                std::unique_ptr<LimbPoseMarker>& res_marker = resource_markers[resource_id];
                res_marker->changeVisibility(false); // hide bounded limb marker
                res_marker->setState(LimbPoseMarker::LimbState::INACTIVE);
                this->setOperational(false);
                rebuildMenu();
              }
							menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
							break;
						case MenuHandler::UNCHECKED:
              if (resource_id < resource_markers.size()) {
                std::unique_ptr<LimbPoseMarker>& res_marker = resource_markers[resource_id];
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
      auto& marker_ptr = *it;
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
    std::string state_msg = "(SUPPORT)";
    auto& marker_ptr = *it;
    if (marker_idx < 4) {
      if (marker_ptr->getState() == LimbPoseMarker::LimbState::INACTIVE) {
        state_msg = "(SUPPORT)";
      } else {
        state_msg = "(FREE)";
      }
    } else {
      state_msg = "";
    }
    handle = menu_handler.insert( "  " + marker_ptr->getResourceName() + " " + state_msg, processFeedback);
    if (marker_ptr->getState() == LimbPoseMarker::LimbState::INACTIVE) {
      menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
    } else {
      menu_handler.setCheckState(handle, MenuHandler::CHECKED);
    }
		resources_entry_map.emplace(handle, marker_ptr->getResourceName());
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
