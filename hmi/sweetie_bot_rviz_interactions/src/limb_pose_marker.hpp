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

  typedef enum {
    UNCONTROLLED,
    FREE,
    SUPPORT
  } LimbState;

public:
  LimbPoseMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                 visualization_msgs::Marker (*makeMarkerBody)(double scale),
                 const std::string& name,
                 ros::NodeHandle leg_node_handle,
                 const std::string& leg_name
                );
  LimbPoseMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                 visualization_msgs::Marker (*makeMarkerBody)(double scale),
                 ros::NodeHandle limb_node_handle
                );
  ~LimbPoseMarker();

  void actionDoneCallback(const GoalState& state, const ResultConstPtr& result);
  void actionActiveCallback();

  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );

  void makeMenu();

  ros::Publisher const & getPosePublisher() const { return pose_pub; }
  std::string const & getResourceName() const { return resource_name; }
  LimbState const & getState() const { return limb_state; }

  void setOperational(bool is_operational);

  bool isPosePublishing() const { return publish_pose; }
  bool isOperational() const { return is_operational; }

private:

  void init(visualization_msgs::Marker (*makeMarkerBody)(double scale));

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
  // limb state
  LimbState limb_state;
  // is_leg flag
  bool is_support;

  // COMPONENT STATE
  // menu index
  MenuHandler::EntryHandle set_operational_entry;
  MenuHandler::EntryHandle publish_pose_entry;
};

} // namespace hmi
} // namespace sweetie_bot

#endif /*LIMB_POSE_MARKER*/
