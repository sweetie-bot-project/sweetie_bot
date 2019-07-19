#include "stance_pose_marker.hpp"

#include <boost/thread/mutex.hpp>

namespace sweetie_bot {
namespace hmi {


StancePoseMarker::StancePoseMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                                   visualization_msgs::Marker (*makeMarkerBody)(const double scale),
                                   const std::string& name,
                                   double scale,
                                   const std::vector<std::string>& resources,
                                   const std::vector<std::string>& frames,
                                   const std::string& marker_home_frame,
                                   double normalized_z_level
                                  )
    : PoseMarker(server, name, scale, frames, marker_home_frame, normalized_z_level),
      resources(resources),
      pose_pub(node_handle.advertise<geometry_msgs::PoseStamped>("stance_pose", 1)),
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

	action_client->cancelAllGoals();

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
    int counter = 0;
    for ( auto it = resources_entry_map.begin(); it != resources_entry_map.end(); ++it, ++counter ) {
      MenuHandler::CheckState check;
      menu_handler.getCheckState( it->first, check );
      if (check == MenuHandler::CHECKED) goal.resources.push_back(it->second);
      if (counter >= 3) break; // We operate only on four first resources
    }

		// send goal to server
		action_client->sendGoal(goal, boost::bind( &StancePoseMarker::actionDoneCallback, this, _1, _2 ), boost::bind( &StancePoseMarker::actionActiveCallback, this ));
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
						// publish current pose
						geometry_msgs::PoseStamped pose_stamped;
						pose_stamped.header = feedback->header;
						pose_stamped.pose = feedback->pose;
						pose_pub.publish(pose_stamped);
				}
      }
			else {
				// check if user toggled resource
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

void StancePoseMarker::makeMenu()
{
  MenuHandler::FeedbackCallback processFeedback = boost::bind( &StancePoseMarker::processFeedback, this, _1 );

	set_operational_entry = menu_handler.insert( "OPERATIONAL", processFeedback);
	menu_handler.setCheckState(set_operational_entry, MenuHandler::UNCHECKED);
  int counter = 0;
	for ( auto it=resources.begin(); it != resources.end(); ++it, ++counter ) {
		MenuHandler::EntryHandle handle = menu_handler.insert( "  " + *it, processFeedback);
    if (counter < 4) // Checking only 4 first resources
      menu_handler.setCheckState(handle, MenuHandler::CHECKED);
    else
      menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
		resources_entry_map.emplace(handle, *it);
	}
	normalize_pose_entry = menu_handler.insert( "Normalize pose", boost::bind( &StancePoseMarker::processNormalize, this, _1, pose_pub, publish_pose ));
	publish_pose_entry = menu_handler.insert( "Publish pose", processFeedback);
	menu_handler.setCheckState(publish_pose_entry, MenuHandler::CHECKED); publish_pose = true;
  if (!frames.empty()) {
    MenuHandler::EntryHandle frames_submenu = menu_handler.insert("Move to frame");
    for ( const std::string& frame : frames ) {
      menu_handler.insert(frames_submenu, frame, boost::bind( &StancePoseMarker::processMoveToFrame, this, _1 ));
    }
  }
}

} // namespace hmi
} // namespace sweetie_bot
