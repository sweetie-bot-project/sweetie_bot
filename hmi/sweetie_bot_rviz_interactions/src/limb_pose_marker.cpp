#include "limb_pose_marker.hpp"

#include <boost/thread/mutex.hpp>

namespace sweetie_bot {
namespace hmi {


LimbPoseMarker::LimbPoseMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                                   visualization_msgs::Marker (*makeMarkerBody)(double scale),
                                   const std::string& name,
                                   double scale,
                                   const std::vector<std::string>& resources,
                                   const std::vector<std::string>& frames,
                                   const std::string& marker_home_frame,
                                   double normalized_z_level
                                  )
    : PoseMarker(server, name, scale, frames, marker_home_frame, normalized_z_level),
      resources(resources),
      pose_pub(node_handle.advertise<geometry_msgs::PoseStamped>("limb_pose", 1)),
      action_client( new ActionClient("limb_set_operational_action", false) )
{
  makeMenu();
  makeInteractiveMarker(makeMarkerBody, boost::bind( &LimbPoseMarker::processFeedback, this, _1 ));

  if (marker_home_frame != "")
    moveToFrame(marker_home_frame);

	server->applyChanges();
}

LimbPoseMarker::~LimbPoseMarker()
{
	pose_pub.shutdown();
	action_client.reset();
}

void LimbPoseMarker::actionDoneCallback(const GoalState& state, const ResultConstPtr& result)
{
	ROS_INFO_STREAM("action client done: state: " << state.toString() << " state_text: " << state.getText()
                  << " error_code: " << result->error_code << " error_string: " << result->error_string);

	action_client->cancelAllGoals();

	menu_handler.setCheckState(set_operational_entry, MenuHandler::UNCHECKED);
	menu_handler.reApply(*server);
	server->applyChanges();
}

void LimbPoseMarker::actionActiveCallback()
{
	GoalState state = action_client->getState();
	ROS_INFO_STREAM(" action client active: state: " << state.toString() << " state_text: " << state.getText() );

	menu_handler.setCheckState(set_operational_entry, MenuHandler::CHECKED);
	menu_handler.reApply(*server);
	server->applyChanges();
}

int LimbPoseMarker::frameNameToResourceId(const std::string& frame)
{
  int resource_id = std::distance(frames.begin(), std::find(frames.begin(), frames.end(), frame));
  if (resource_id < resources.size())
    return resource_id;
  else
    return -1;
}

bool LimbPoseMarker::setOperational(bool is_operational)
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
    int resource_id = frameNameToResourceId(marker_home_frame);
    if (resource_id < 0) {
      ROS_ERROR("SetOperation action cannot perform, because marker's home frame doesn't match any existing resource");
      return false;
    }
    else {
      goal.resources.push_back(resources[resource_id]);
    }

		// send goal to server
		action_client->sendGoal(goal, boost::bind( &LimbPoseMarker::actionDoneCallback, this, _1, _2 ), boost::bind( &LimbPoseMarker::actionActiveCallback, this ));
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

void LimbPoseMarker::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
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
			else {
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
			break;
	}

	menu_handler.reApply(*server);
	server->applyChanges();
}

void LimbPoseMarker::makeMenu()
{
  MenuHandler::FeedbackCallback processFeedback = boost::bind( &LimbPoseMarker::processFeedback, this, _1 );

	set_operational_entry = menu_handler.insert( "OPERATIONAL", processFeedback);
	menu_handler.setCheckState(set_operational_entry, MenuHandler::UNCHECKED);
	normalize_pose_entry = menu_handler.insert( "Normalize pose", boost::bind( &LimbPoseMarker::processNormalize, this, _1, pose_pub, publish_pose ));
	publish_pose_entry = menu_handler.insert( "Publish pose", processFeedback);
	menu_handler.setCheckState(publish_pose_entry, MenuHandler::CHECKED); publish_pose = true;
  if (!frames.empty()) {
    MenuHandler::EntryHandle frames_submenu = menu_handler.insert("Move to frame");
    for ( const std::string& frame : frames ) {
      menu_handler.insert(frames_submenu, frame, boost::bind( &LimbPoseMarker::processMoveToFrame, this, _1 ));
    }
  }
}

} // namespace hmi
} // namespace sweetie_bot
