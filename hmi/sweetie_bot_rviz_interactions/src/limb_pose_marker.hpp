#ifndef LIMB_POSE_MARKER
#define LIMB_POSE_MARKER

#include "pose_marker.hpp"

#include <actionlib/client/simple_action_client.h>
#include <sweetie_bot_control_msgs/SetOperationalAction.h>

namespace sweetie_bot {
namespace hmi {


class LimbPoseMarker : public PoseMarker {
public:
  // Action type definitions
  ACTION_DEFINITION(sweetie_bot_control_msgs::SetOperationalAction);
  typedef actionlib::SimpleActionClient<sweetie_bot_control_msgs::SetOperationalAction> ActionClient;
  typedef actionlib::SimpleClientGoalState GoalState;

public:
  LimbPoseMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                 visualization_msgs::Marker (*makeMarkerBody)(double scale),
                 const std::string& name,
                 double scale = 1.0,
                 const std::vector<std::string>& resources = { "leg1", "leg2", "leg3", "leg4" },
                 const std::vector<std::string>& frames = { "bone15", "bone25", "bone35", "bone45", "bone55", "base_link" },
                 const std::string& marker_home_frame = "",
                 double normalized_z_level = 0.0
                );
  ~LimbPoseMarker();

  void actionDoneCallback(const GoalState& state, const ResultConstPtr& result);
  void actionActiveCallback();

  int  frameNameToResourceId(const std::string& frame);
  bool setOperational(bool is_operational);
  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );

  void makeMenu();

  ros::Publisher const & getPosePublisher() const { return pose_pub; }
  bool isPosePublishing() const { return publish_pose; }
  bool isOperational() const { return is_operational; }

private:

  // COMPONENT INTERFACE

  // CONNECTIONS
  // action server
  std::unique_ptr<ActionClient> action_client;
  // publisers
  ros::Publisher pose_pub;

  // PARAMETERS
  // resources list: corresponding menu items will be displayed in context menu
  std::vector<std::string> resources;

  // COMPONENT STATE
  // menu index
  MenuHandler::EntryHandle set_operational_entry;
  MenuHandler::EntryHandle publish_pose_entry;
  // publish_pose flag
  bool publish_pose = true;
};

} // namespace hmi
} // namespace sweetie_bot

#endif /*LIMB_POSE_MARKER*/
