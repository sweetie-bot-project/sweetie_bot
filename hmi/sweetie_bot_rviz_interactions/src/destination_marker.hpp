#ifndef DESTINATION_MARKER_HPP
#define DESTINATION_MARKER_HPP

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <actionlib/client/simple_action_client.h>
#include <sweetie_bot_gait_generator/MoveBaseAction.h>

using interactive_markers::MenuHandler;
using sweetie_bot_gait_generator::MoveBaseGoal;
using sweetie_bot_gait_generator::MoveBaseAction;
using sweetie_bot_gait_generator::EndEffectorGoal;

namespace sweetie_bot {
namespace hmi {

class DestinationMarker {
public:
  // Action type definitions
  ACTION_DEFINITION(MoveBaseAction);
  typedef actionlib::SimpleActionClient<MoveBaseAction> ActionClient;
  typedef actionlib::SimpleClientGoalState GoalState;

public:
  DestinationMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server, ros::NodeHandle node_handle);

  // mark class not copy/move assignable: boost::bind stores this value to call callbacks
  DestinationMarker(const DestinationMarker&) = delete; 
  DestinationMarker& operator= (const DestinationMarker&) = delete;

private:
  static MoveBaseGoal buildMoveBaseGoal(const std::string& frame_id, const std::string& gait_type, double duration, unsigned n_steps, bool execute_only);
  static void setBaseGoal(MoveBaseGoal& msg, const geometry_msgs::Pose& base_goal, double nominal_height);
  static void setEndEffectorTargets(std::vector<EndEffectorGoal>& ee_goals, std::vector<std::string>& ee_names, unsigned frame_type = EndEffectorGoal::NOMINAL_POSE);
  static void setEndEffectorPosition(EndEffectorGoal& ee_goal, double x, double y, double z);

  void toNominal();
  void invokeClopGenerator(const geometry_msgs::Pose& base_goal, bool execute_only = false);
  void actionDoneCallback(const GoalState& state, const ResultConstPtr& result);
  void actionActiveCallback();

  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
  void processGaitType( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
  void processStepsNum( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );

  void makeInteractiveMarker();
  void makeInteractiveMarker(const geometry_msgs::Pose& position);
  visualization_msgs::Marker makePointMarker();
  visualization_msgs::Marker makeArrowMarker();
  visualization_msgs::Marker makeSphereMarker(float r, float g, float b);
  visualization_msgs::Marker makeConeMarker(float r, float g, float b, float scale, float z);
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
  // service clients
  ros::ServiceClient display_ee_limits_client;

  // PARAMETERS
  // marker name
  std::string name;
  // world frame
  std::string world_frame;
  // gait generator parameters
  std::string gait_generator_ns;
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
  // end effectors' names
  std::vector<std::string> ee_names;
  // Name of trajectory for saving
  std::string trajectory_name;

  // COMPONENT STATE
  // interactive marker server
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  // menu
  MenuHandler base_menu_handler;
  MenuHandler menu_handler;
  MenuHandler::EntryHandle start_motion_entry;
  MenuHandler::EntryHandle repeat_last_motion_entry;
  MenuHandler::EntryHandle to_nominal_entry;
  std::map<MenuHandler::EntryHandle, std::string> gait_type_submenu;
  std::map<MenuHandler::EntryHandle, unsigned> n_steps_submenu;
  MenuHandler::EntryHandle change_duration_entry;
  MenuHandler::EntryHandle increase_by_half_duration_entry;
  MenuHandler::EntryHandle decrease_by_half_duration_entry;
  MenuHandler::EntryHandle change_trajectory_name_entry;
  MenuHandler::EntryHandle display_ee_limits_entry;

};

} // namespace hmi
} // namespace sweetie_bot

#endif /*DESTINATION_MARKER_HPP*/
