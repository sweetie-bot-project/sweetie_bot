#ifndef DESTINATION_MARKER_HPP
#define DESTINATION_MARKER_HPP

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <actionlib/client/simple_action_client.h>
#include <sweetie_bot_clop_generator/MoveBaseAction.h>

namespace sweetie_bot {
namespace hmi {

class DestinationMarker {
public:
  // Action type definitions
  ACTION_DEFINITION(sweetie_bot_clop_generator::MoveBaseAction);
  typedef actionlib::SimpleActionClient<sweetie_bot_clop_generator::MoveBaseAction> ActionClient;
  typedef actionlib::SimpleClientGoalState GoalState;

public:
  DestinationMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                    const std::string& name,
                    double scale = 1.0,
                    const std::string& gait_type = "walk_overlap",
                    int n_steps = 4,
                    double duration = 3.4,
                    double nominal_height = 0.1825
                   );
  void invokeClopGenerator( const geometry_msgs::Pose& base_goal );
  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );

private:

  void actionDoneCallback(const GoalState& state, const ResultConstPtr& result);
  void actionActiveCallback();

  void makeInteractiveMarker();
  visualization_msgs::Marker makePointMarker();
  visualization_msgs::Marker makeArrowMarker();
  visualization_msgs::Marker makeSphereMarker();
  visualization_msgs::Marker makeConeMarker();
  void makeMenu();

private:

  // Node handle
	ros::NodeHandle node_handle;

  // CONNECTION
  // action server
  std::unique_ptr<ActionClient> action_client;

  // PARAMETERS
  // marker name
  std::string name;
  // marker sacle parameter
  double scale;
  // gait type
  std::string gait_type;
  // number of steps cycles
  int n_steps;
  // movement duration
  double duration;
  // nominal base height
  double nominal_height;

  // COMPONENT STATE
  // interactive marker server
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  // menu
  interactive_markers::MenuHandler menu_handler;
};

} // namespace hmi
} // namespace sweetie_bot

#endif /*DESTINATION_MARKER_HPP*/
