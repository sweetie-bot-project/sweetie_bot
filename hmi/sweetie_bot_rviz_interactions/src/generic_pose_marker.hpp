#ifndef GENERIC_POSE_MARKER_HPP
#define GENERIC_POSE_MARKER_HPP

#include "pose_marker_base.hpp"

#include <actionlib/client/simple_action_client.h>
#include <sweetie_bot_control_msgs/SetOperationalAction.h>

namespace sweetie_bot {
namespace hmi {


class GenericPoseMarker : public PoseMarkerBase {
public:
  // Action type definitions
  ACTION_DEFINITION(sweetie_bot_control_msgs::SetOperationalAction);
  typedef actionlib::SimpleActionClient<sweetie_bot_control_msgs::SetOperationalAction> ActionClient;
  typedef actionlib::SimpleClientGoalState GoalState;

public:
  GenericPoseMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                 ros::NodeHandle node_handle
                );
  ~GenericPoseMarker();

  void actionDoneCallback(const GoalState& state, const ResultConstPtr& result);
  void actionActiveCallback();

  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );

  void makeMenu();

private:

  // Display platform shape as controlled body
  static visualization_msgs::Marker makeMarkerBody(double scale)
  {
    visualization_msgs::Marker marker;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.16*scale;
    marker.scale.y = 0.08*scale;
    marker.scale.z = 0.02*scale;
    marker.color.r = 0.8;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.7;

    return marker;
  }

  void setOperational(bool is_operational);

  // COMPONENT INTERFACE

  // CONNECTIONS
  // action server
  std::unique_ptr<ActionClient> action_client;

  // PARAMETERS
  // resource_select_only_one flag
  bool select_only_one_resource = false;
  // frames
  std::vector<std::string> frames = { "bone15", "bone25", "bone35", "bone45", "bone55", "base_link" };
  // resources
  std::vector<std::string> resources = { "leg1", "leg2", "leg3", "leg4", "nose" };

  // COMPONENT STATE
  // menu index
  MenuHandler::EntryHandle set_operational_entry;
  std::map<MenuHandler::EntryHandle, std::string> resources_entry_map;
  MenuHandler::EntryHandle frames_submenu_entry;
};

} // namespace hmi
} // namespace sweetie_bot

#endif /*GENERIC_POSE_MARKER_HPP*/
