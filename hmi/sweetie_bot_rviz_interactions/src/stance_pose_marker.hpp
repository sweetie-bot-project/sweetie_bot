#ifndef STANCE_POSE_MARKER_HPP
#define STANCE_POSE_MARKER_HPP

#include "pose_marker_base.hpp"
#include "limb_pose_marker.hpp"

#include <actionlib/client/simple_action_client.h>
#include <sweetie_bot_control_msgs/SetOperationalAction.h>

using visualization_msgs::Marker;

namespace sweetie_bot {
namespace hmi {


class StancePoseMarker : public PoseMarkerBase {
public:
  // Action type definitions
  ACTION_DEFINITION(sweetie_bot_control_msgs::SetOperationalAction);
  typedef actionlib::SimpleActionClient<sweetie_bot_control_msgs::SetOperationalAction> ActionClient;
  typedef actionlib::SimpleClientGoalState GoalState;

public:
  StancePoseMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server, ros::NodeHandle node_handle);
  ~StancePoseMarker();

private:
  void actionDoneCallback(const GoalState& state, const ResultConstPtr& result);
  void actionActiveCallback();

  void setOperational(bool is_operational);
  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void processNormalizeLegs( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void processMoveAllToHome( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void makeMenu();
  inline void rebuildMenu() {
    menu_handler = MenuHandler(); // Reset menu handler

    menu_handler.apply(*server, name);
    makeMenu(); // Remake menu

    // Restore original checkbox states
    if (is_pose_publishing)
      menu_handler.setCheckState(publish_pose_entry, MenuHandler::CHECKED);
    else
      menu_handler.setCheckState(publish_pose_entry, MenuHandler::UNCHECKED);

    if (is_6DOF)
      menu_handler.setCheckState(enable_6DOF_entry, MenuHandler::CHECKED);
    else
      menu_handler.setCheckState(enable_6DOF_entry, MenuHandler::UNCHECKED);

    menu_handler.reApply(*server);
    server->applyChanges();
 }

  static Marker makeCubeBody(double scale);
  static Marker makeSphereBody(double scale);

  void setLegMarkers(std::vector< std::unique_ptr<LimbPoseMarker> >& leg_markers) { this->leg_markers = std::move(leg_markers); rebuildMenu(); }
  void setLimbMarkers(std::vector< std::unique_ptr<LimbPoseMarker> >& limb_markers) { this->limb_markers = std::move(limb_markers); rebuildMenu(); }

private:

  // COMPONENT INTERFACE

  // CONNECTIONS
  // action server
  std::unique_ptr<ActionClient> action_client;

  // PARAMETERS
  // resource markers vector
  std::vector< std::unique_ptr<LimbPoseMarker> > leg_markers;
  // resource markers vector
  std::vector< std::unique_ptr<LimbPoseMarker> > limb_markers;

  // COMPONENT STATE
  // menu index
  MenuHandler::EntryHandle set_operational_entry;
  std::map<MenuHandler::EntryHandle, std::string> resources_entry_map;
  MenuHandler::EntryHandle move_all_to_home_etry;
  MenuHandler::EntryHandle normalize_legs;
};

} // namespace hmi
} // namespace sweetie_bot

#endif /*STANCE_POSE_MARKER_HPP*/
