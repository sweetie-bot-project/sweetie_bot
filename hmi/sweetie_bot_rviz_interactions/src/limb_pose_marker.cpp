#include "limb_pose_marker.hpp"

#include <boost/thread/mutex.hpp>

namespace sweetie_bot {
namespace hmi {


LimbPoseMarker::LimbPoseMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                               visualization_msgs::Marker (*makeMarkerBody)(double scale),
                               const std::string& name,
                               ros::NodeHandle leg_node_handle,
                               const std::string& leg_name
                              )
  : PoseMarkerBase(server, name),
    action_client( new ActionClient("limb_set_operational_action", false) ),
    pose_pub(ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("limb_pose", 1)),
    resource_name(""),
    limb_state(LimbPoseMarker::LimbState::INACTIVE)
{
  leg_node_handle.getParam("scale", scale);
  if (scale < 0) {
    ROS_FATAL("LimbPoseMarker: scale parameter cannot be negative");
    exit(1);
  }

  leg_node_handle.getParam("normalized_z_level", normalized_z_level);
  if (normalized_z_level < 0) {
    ROS_FATAL("LimbPoseMarker: normalized_z_level parameter cannot be negative");
    exit(1);
  }

  leg_node_handle.getParam("list/" + leg_name + "/frame", marker_home_frame);
  leg_node_handle.getParam("list/" + leg_name + "/resource", resource_name);

  init(makeMarkerBody);
}


LimbPoseMarker::LimbPoseMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                               visualization_msgs::Marker (*makeMarkerBody)(double scale),
                               ros::NodeHandle limb_node_handle
                               )
  : PoseMarkerBase(server, limb_node_handle),
    action_client( new ActionClient("limb_set_operational_action", false) ),
    pose_pub(ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("limb_pose", 1)),
    resource_name(""),
    limb_state(LimbPoseMarker::LimbState::INACTIVE)
{
  limb_node_handle.getParam("resource", resource_name);

  // Set custom pose topic and activation action
  if (limb_node_handle.hasParam("pose_topic")) {
    std::string pose_topic;
    limb_node_handle.getParam("pose_topic", pose_topic);
    pose_pub = ros::NodeHandle().advertise<geometry_msgs::PoseStamped>(pose_topic, 1);
  }

  if (limb_node_handle.hasParam("activation_action")) {
    std::string activation_action;
    limb_node_handle.getParam("activation_action", activation_action);
    action_client.reset(new ActionClient(activation_action, false));
  }

  init(makeMarkerBody);
}

LimbPoseMarker::~LimbPoseMarker()
{
  pose_pub.shutdown();
  action_client.reset();
}

void LimbPoseMarker::init(visualization_msgs::Marker (*makeMarkerBody)(double scale)) {
  makeMenu();
  makeInteractiveMarker(makeMarkerBody, boost::bind( &LimbPoseMarker::processFeedback, this, _1 ));

  if (marker_home_frame != "")
    moveToFrame(marker_home_frame);

  server->applyChanges();

  // hide limb marker by default
  changeVisibility(false);
}

void LimbPoseMarker::actionDoneCallback(const GoalState& state, const ResultConstPtr& result)
{
  ROS_INFO_STREAM("action client done: state: " << state.toString() << " state_text: " << state.getText()
                  << " error_code: " << result->error_code << " error_string: " << result->error_string);

  if (limb_state == LimbState::OPERATIONAL) {
    setState(LimbState::FREE);
  } else {
    setOperational(false);
  }
  //action_client->cancelAllGoals();

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

void LimbPoseMarker::setState(LimbState state) {
  this->limb_state = state;
  if (state == LimbState::OPERATIONAL) {
    setOperational(true);
  } else {
    setOperational(false);
  }
}

void LimbPoseMarker::setOperational(bool is_operational)
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
    goal.resources.push_back(resource_name);

    // send goal to server
    action_client->sendGoal(goal, boost::bind( &LimbPoseMarker::actionDoneCallback, this, _1, _2 ), boost::bind( &LimbPoseMarker::actionActiveCallback, this ));
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
          setState(LimbState::OPERATIONAL);
        }
        else {
          setState(LimbState::FREE);
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
            if (is_operational) {
              // publish current pose
              geometry_msgs::PoseStamped pose_stamped;
              pose_stamped.header = feedback->header;
              pose_stamped.pose = feedback->pose;
              pose_pub.publish(pose_stamped);
            }
        }
      }
      break;

  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
    if (!is_operational) {
      setState(LimbState::OPERATIONAL);
    }
    break;
  }

  menu_handler.reApply(*server);
  server->applyChanges();
}

void LimbPoseMarker::makeMenu()
{
  MenuHandler::FeedbackCallback processFeedback = boost::bind( &LimbPoseMarker::processFeedback, this, _1 );

  set_operational_entry = menu_handler.insert( "Keep activated", processFeedback);
  menu_handler.setCheckState(set_operational_entry, MenuHandler::UNCHECKED);
  publish_pose_entry = menu_handler.insert( "Publish pose", processFeedback);
  menu_handler.setCheckState(publish_pose_entry, MenuHandler::CHECKED);
  normalize_pose_entry = menu_handler.insert( "Normalize pose", boost::bind( &LimbPoseMarker::processNormalize, this, _1, pose_pub, publish_pose ));
  enable_6DOF_entry = menu_handler.insert( "Enable 6-DOF", boost::bind( &LimbPoseMarker::processEnable6DOF, this, _1 ));
  menu_handler.setCheckState(enable_6DOF_entry, MenuHandler::CHECKED);
  menu_handler.insert("Move to home frame", boost::bind( &LimbPoseMarker::processMoveToHomeFrame, this, _1 ));

  menu_handler.reApply(*server);
  server->applyChanges();
}

} // namespace hmi
} // namespace sweetie_bot
