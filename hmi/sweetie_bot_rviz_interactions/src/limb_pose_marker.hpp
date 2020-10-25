#ifndef LIMB_POSE_MARKER_HPP
#define LIMB_POSE_MARKER_HPP

#include "pose_marker_base.hpp"

#include <actionlib/client/simple_action_client.h>
#include <sweetie_bot_control_msgs/SetOperationalAction.h>

namespace sweetie_bot {
namespace hmi {


class LimbPoseMarker : public PoseMarkerBase {
public:
  // Action type definitions
  ACTION_DEFINITION(sweetie_bot_control_msgs::SetOperationalAction);
  typedef actionlib::SimpleActionClient<sweetie_bot_control_msgs::SetOperationalAction> ActionClient;
  typedef actionlib::SimpleClientGoalState GoalState;

  typedef enum {
    INACTIVE,
    FREE,
    OPERATIONAL
  } LimbState;

public:
  LimbPoseMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                 MakeMarkerBodyFuncPtr makeMarkerBody,
                 ros::NodeHandle legs_common_node_handle,
                 ros::NodeHandle leg_node_handle
                );
  LimbPoseMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                 MakeMarkerBodyFuncPtr makeMarkerBody,
                 ros::NodeHandle limb_node_handle
                );
  ~LimbPoseMarker();

  std::string const & getResourceName() const { return resource_name; }
  LimbState const & getState() const { return limb_state; }

  void setState(LimbState state);

private:

  void init(MakeMarkerBodyFuncPtr makeMarkerBody);

  void actionDoneCallback(const GoalState& state, const ResultConstPtr& result);
  void actionActiveCallback();

  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );

  void makeMenu();

  void setOperational(bool is_operational);

  // COMPONENT INTERFACE

  // CONNECTIONS
  // action server
  std::unique_ptr<ActionClient> action_client;

  // PARAMETERS
  // resource name bounded with marker
  std::string resource_name;
  // limb state
  LimbState limb_state;

  // COMPONENT STATE
  // menu index
  MenuHandler::EntryHandle set_operational_entry;
};

} // namespace hmi
} // namespace sweetie_bot

#endif /*LIMB_POSE_MARKER_HPP*/
