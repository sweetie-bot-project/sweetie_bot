#ifndef STANCE_POSE_MARKER
#define STANCE_POSE_MARKER

#include "pose_marker.hpp"
#include "limb_pose_marker.hpp"

#include <actionlib/client/simple_action_client.h>
#include <sweetie_bot_control_msgs/SetOperationalAction.h>

namespace sweetie_bot {
namespace hmi {


class StancePoseMarker : public PoseMarker {
public:
  // Action type definitions
  ACTION_DEFINITION(sweetie_bot_control_msgs::SetOperationalAction);
  typedef actionlib::SimpleActionClient<sweetie_bot_control_msgs::SetOperationalAction> ActionClient;
  typedef actionlib::SimpleClientGoalState GoalState;

public:
  StancePoseMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                   visualization_msgs::Marker (*makeMarkerBody)(double scale),
                   const std::string& name,
                   double scale = 1.0,
                   const std::vector<std::string>& resources = { "leg1", "leg2", "leg3", "leg4", "head" },
                   const std::vector<std::string>& frames = { "bone15", "bone25", "bone35", "bone45", "bone55", "base_link" },
                   const std::string& marker_home_frame = "base_link",
                   double normalized_z_level = 0.0
                  );
  ~StancePoseMarker();

  void actionDoneCallback(const GoalState& state, const ResultConstPtr& result);
  void actionActiveCallback();

  bool setOperational(bool is_operational);
  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void processNormalizeLegs( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void processMoveAllToHome( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void makeMenu();
  void rebuildMenu() {
    menu_handler = MenuHandler(); // Reset menu handler

    menu_handler.apply(*server, name);
    makeMenu(); // Remake menu

    // Restore original checkbox states
    if (publish_pose)
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

  void setResourceMarkers(std::vector< std::shared_ptr<LimbPoseMarker> > resource_markers) { this->resource_markers = resource_markers; rebuildMenu(); }

private:

  // COMPONENT INTERFACE

  // CONNECTIONS
  // action server
  std::unique_ptr<ActionClient> action_client;
  // publisers
  ros::Publisher pose_pub;

  // PARAMETERS
  // resources name list: corresponding menu items will be displayed in context menu
  std::vector<std::string> resource_names;

  // COMPONENT STATE
  // menu index
  MenuHandler::EntryHandle set_operational_entry;
  std::map<MenuHandler::EntryHandle, std::string> resources_entry_map;
  MenuHandler::EntryHandle publish_pose_entry;
  MenuHandler::EntryHandle move_all_to_home_etry;
  MenuHandler::EntryHandle normalize_legs;
  // publish_pose flag
  bool publish_pose = true;
  // resource markers vector
  std::vector< std::shared_ptr<LimbPoseMarker> > resource_markers;
};

} // namespace hmi
} // namespace sweetie_bot

#endif /*STANCE_POSE_MARKER*/
