#ifndef DESTINATION_MARKER_HPP
#define DESTINATION_MARKER_HPP

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <actionlib/client/simple_action_client.h>
#include <sweetie_bot_clop_generator/MoveBaseAction.h>

using interactive_markers::MenuHandler;

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
                    ros::NodeHandle node_handle
                    );

private:
  void setEndEffectorTargets(const std::vector<std::string>& ee_names, unsigned frame_type = sweetie_bot_clop_generator::EndEffectorGoal::NOMINAL_POSE);

  void actionDoneCallback(const GoalState& state, const ResultConstPtr& result);
  void actionActiveCallback();

  void invokeClopGenerator( const geometry_msgs::Pose& base_goal );
  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
  void processGaitType( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
  void processStepsNum( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );

  void makeInteractiveMarker();
  visualization_msgs::Marker makePointMarker();
  visualization_msgs::Marker makeArrowMarker();
  visualization_msgs::Marker makeSphereMarker();
  visualization_msgs::Marker makeConeMarker();
  void makeMenu(const std::vector<std::string>& gait_type_options, const std::vector<unsigned>& n_steps_options);
  inline void rebuildMenu() {
    menu_handler = MenuHandler(base_menu_handler); // Restore base menu handler without last entry

    menu_handler.apply(*server, name);
    makeMenu(std::vector<std::string>(), std::vector<unsigned>()); // Remake menu

    // Restore original checkbox states
    for (auto& entry: gait_type_submenu) {
      if (entry.second == gait_type)
        menu_handler.setCheckState(entry.first, MenuHandler::CHECKED);
      else
        menu_handler.setCheckState(entry.first, MenuHandler::UNCHECKED);
    }
    for (auto& entry: n_steps_submenu) {
      if (entry.second == n_steps)
        menu_handler.setCheckState(entry.first, MenuHandler::CHECKED);
      else
        menu_handler.setCheckState(entry.first, MenuHandler::UNCHECKED);
    }

    menu_handler.reApply(*server);
    server->applyChanges();
 }

private:
  // CONNECTION
  // action server
  std::unique_ptr<ActionClient> action_client;

  // PARAMETERS
  // marker name
  std::string name;
  // marker sacle parameter
  double scale;
  // selected gait type
  std::string gait_type;
  // selected number of steps cycles
  unsigned n_steps;
  // movement duration
  double duration;
  // nominal base height
  double nominal_height;
  // end effector goals
  std::vector<sweetie_bot_clop_generator::EndEffectorGoal> ee_goal;
  // Name of trajectory for saving
  std::string trajectory_name;

  // COMPONENT STATE
  // interactive marker server
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  // menu
  MenuHandler base_menu_handler;
  MenuHandler menu_handler;
  MenuHandler::EntryHandle start_walk_entry;
  std::map<MenuHandler::EntryHandle, std::string> gait_type_submenu;
  std::map<MenuHandler::EntryHandle, unsigned> n_steps_submenu;
  MenuHandler::EntryHandle change_duration_entry;
  MenuHandler::EntryHandle increase_by_half_duration_entry;
  MenuHandler::EntryHandle decrease_by_half_duration_entry;
  MenuHandler::EntryHandle change_trajectory_name_entry;

};

} // namespace hmi
} // namespace sweetie_bot

#endif /*DESTINATION_MARKER_HPP*/
