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
                 const std::string& resource_name = "",
                 const std::string& marker_home_frame = "",
                 double normalized_z_level = 0.0
                );
  ~LimbPoseMarker();

  void actionDoneCallback(const GoalState& state, const ResultConstPtr& result);
  void actionActiveCallback();

  bool setOperational(bool is_operational);
  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );

  void makeMenu();

  ros::Publisher const & getPosePublisher() const { return pose_pub; }
  std::string const & getResourceName() const { return resource_name; }
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
  // resource name bounded with marker
  std::string resource_name;
  // publish_pose flag
  bool publish_pose = true;

  // COMPONENT STATE
  // menu index
  MenuHandler::EntryHandle set_operational_entry;
  MenuHandler::EntryHandle publish_pose_entry;
};

} // namespace hmi
} // namespace sweetie_bot

#endif /*LIMB_POSE_MARKER*/
