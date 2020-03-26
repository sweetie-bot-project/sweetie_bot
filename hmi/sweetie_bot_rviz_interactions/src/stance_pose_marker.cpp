#include "stance_pose_marker.hpp"

#include <boost/thread/mutex.hpp>

namespace sweetie_bot {
namespace hmi {


StancePoseMarker::StancePoseMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server, ros::NodeHandle node_handle)
  : PoseMarkerBase(server, node_handle),
    action_client( new ActionClient("stance_set_operational_action", false) ),
    pose_pub(ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("stance_pose", 1))
{
  makeMenu();
  makeInteractiveMarker(makeCubeBody, boost::bind( &StancePoseMarker::processFeedback, this, _1 ));

  // wait for transform gets available
  ros::Time now = ros::Time::now();
  tf_buffer.canTransform(world_frame, marker_home_frame, now, ros::Duration(5.0));

  // and move it to its home frame
  if (marker_home_frame != "")
    moveToFrame(marker_home_frame);

  // Add legs markers
  std::vector<std::unique_ptr<LimbPoseMarker>> leg_markers;
  if (node_handle.hasParam("inner_markers/legs/")) {
    ros::NodeHandle legs_common_nh("~inner_markers/legs/");

    XmlRpc::XmlRpcValue legs;
    node_handle.getParam("inner_markers/legs/list/", legs);

    for (auto& leg: legs) {
      ros::NodeHandle leg_nh("~inner_markers/legs/list/" + leg.first);

      leg_markers.push_back(std::unique_ptr<LimbPoseMarker>(new LimbPoseMarker(server, &makeCubeBody, legs_common_nh, leg_nh)));
    }
  } else {
    ROS_WARN("RobotPoseMarker: Less than 4 legs has defined in yaml file. Marker may not function properly");
  }

  setLegMarkers(leg_markers);

  // Add limbs markers
  std::vector<std::unique_ptr<LimbPoseMarker>> limb_markers;
  if (node_handle.hasParam("inner_markers/limbs/")) {
    XmlRpc::XmlRpcValue limbs;
    node_handle.getParam("inner_markers/limbs/", limbs);

    for (auto& limb: limbs) {
      ros::NodeHandle limb_nh("~inner_markers/limbs/" + limb.first);

      bool is_sphere;
      limb_nh.param<bool>("is_sphere", is_sphere, true);

      const MakeMarkerBodyFuncPtr& makeBodyRef = is_sphere ? &makeSphereBody : &makeCubeBody;

      limb_markers.push_back(std::unique_ptr<LimbPoseMarker>(new LimbPoseMarker(server, makeBodyRef, limb_nh)));
    }
  }

  setLimbMarkers(limb_markers);

  server->applyChanges();
}

StancePoseMarker::~StancePoseMarker()
{
  pose_pub.shutdown();
  action_client.reset();
}

// Display platform shape as controlled body
Marker StancePoseMarker::makeCubeBody(double scale)
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

// Display sphere as controlled body
Marker StancePoseMarker::makeSphereBody(double scale)
{
  Marker marker;

  marker.type = Marker::SPHERE;
  marker.scale.x = 0.08*scale;
  marker.scale.y = 0.08*scale;
  marker.scale.z = 0.08*scale;
  marker.color.r = 0.8;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.7;

  return marker;
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
    for (auto& leg_marker : leg_markers) {
      if (leg_marker->getState() == LimbPoseMarker::LimbState::INACTIVE) {
        goal.resources.push_back(leg_marker->getResourceName());
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

          std::unique_ptr<LimbPoseMarker>& res_marker = (resource_id < leg_markers.size()) ? leg_markers[resource_id] : limb_markers[resource_id - leg_markers.size()];
          switch (check) {
            case MenuHandler::CHECKED:
              res_marker->changeVisibility(false); // hide bounded limb marker
              res_marker->setState(LimbPoseMarker::LimbState::INACTIVE);
              if (this->is_operational) {
                // We update operational state after resource marker deactivates
                // in order to keep controlled resources list valid
                this->setOperational(true);
              }
              rebuildMenu();
              menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
              break;
            case MenuHandler::UNCHECKED:
              res_marker->changeVisibility(true); // show bounded limb marker
              res_marker->moveToFrame(res_marker->getMarkerHomeFrame()); // also move it to its place
              res_marker->setState(LimbPoseMarker::LimbState::FREE);
              if (this->is_operational) {
                // We update operational state after resource marker activates
                // in order to keep controlled resources list valid
                this->setOperational(true);
              }
              rebuildMenu();
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

    if (leg_markers.empty()) return;

    for (auto& leg_marker: leg_markers) {
      if (leg_marker->isVisible()) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = std_msgs::Header();
        pose_stamped.header.frame_id = world_frame;
        // update marker pose
        leg_marker->reloadMarker();
        pose_stamped.pose = leg_marker->getInteractiveMarker().pose;
        // normalize marker
        leg_marker->normalize(pose_stamped);
        // publish new pose
        if (leg_marker->isPosePublishing() && leg_marker->isOperational()) {
          leg_marker->getPosePublisher().publish(pose_stamped);
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
    // move leg markers to home
    for (auto& marker : leg_markers) {
      marker->moveToFrame(marker->getMarkerHomeFrame());
    }
    // move limb markers to home
    for (auto& marker : limb_markers) {
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

  // Add menu entries for legs
  for (auto& leg_marker: leg_markers) {
    MenuHandler::EntryHandle handle;
    std::string state_msg = "(SUPPORT)";

    if (leg_marker->getState() == LimbPoseMarker::LimbState::INACTIVE) {
      state_msg = "(SUPPORT)";
    } else {
      state_msg = "(FREE)";
    }
    handle = menu_handler.insert( "  " + leg_marker->getResourceName() + " " + state_msg, processFeedback);
    if (leg_marker->getState() == LimbPoseMarker::LimbState::INACTIVE) {
      menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
    } else {
      menu_handler.setCheckState(handle, MenuHandler::CHECKED);
    }
    resources_entry_map.emplace(handle, leg_marker->getResourceName());
  }

  // Add menu entries for other limbs
  for (auto& limb_marker: limb_markers) {
    MenuHandler::EntryHandle handle;

    handle = menu_handler.insert( "  " + limb_marker->getResourceName(), processFeedback);
    if (limb_marker->getState() == LimbPoseMarker::LimbState::INACTIVE) {
      menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
    } else {
      menu_handler.setCheckState(handle, MenuHandler::CHECKED);
    }
    resources_entry_map.emplace(handle, limb_marker->getResourceName());
  }

  normalize_pose_entry = menu_handler.insert( "Move all to home", boost::bind( &StancePoseMarker::processMoveAllToHome, this, _1 ));
  normalize_pose_entry = menu_handler.insert( "Normalize legs", boost::bind( &StancePoseMarker::processNormalizeLegs, this, _1 ));
  normalize_pose_entry = menu_handler.insert( "Normalize pose", boost::bind( &StancePoseMarker::processNormalize, this, _1, pose_pub, publish_pose ));
  publish_pose_entry = menu_handler.insert( "Publish pose", processFeedback);
  menu_handler.setCheckState(publish_pose_entry, MenuHandler::CHECKED);
  enable_6DOF_entry = menu_handler.insert( "Enable 6-DOF", boost::bind( &StancePoseMarker::processEnable6DOF, this, _1 ));
  menu_handler.setCheckState(enable_6DOF_entry, MenuHandler::CHECKED);
  menu_handler.insert("Move to home frame", boost::bind( &StancePoseMarker::processMoveToHomeFrame, this, _1 ));

  menu_handler.reApply(*server);
  server->applyChanges();
}

} // namespace hmi
} // namespace sweetie_bot
