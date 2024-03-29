#include "destination_marker.hpp"

#include <ros/ros.h>
#include <boost/bind/bind.hpp>
#include <sstream>
#include <math.h>
#include <stdlib.h>

#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Header.h>

#include <sweetie_bot_gait_generator/MoveBaseGoal.h>
#include <sweetie_bot_gait_generator/EndEffectorGoal.h>
#include <sweetie_bot_gait_generator/SaveTrajectory.h>

#include <QInputDialog>
#include <QDesktopWidget>
#include <QApplication>

using namespace visualization_msgs;
using namespace interactive_markers;

namespace sweetie_bot {
namespace hmi {

DestinationMarker::DestinationMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                                     ros::NodeHandle node_handle
                                    )
  : action_client( new ActionClient("move_base_action", false) ),
    name(""),
    world_frame("odom_combined"),
    scale(1.0),
    gait_type("walk_overlap"),
    n_steps(4),
    duration(4.0),
    nominal_height(0.1825),
    trajectory_name("recorded_trajectory"),
    server(server)
{
  std::vector<std::string> gait_type_options;
  std::vector<int> _n_steps_options;

  node_handle.getParam("name", name);
  node_handle.getParam("world_frame", world_frame);

  node_handle.getParam("scale", scale);
  if (scale < 0) {
    ROS_FATAL("DestinationMarker: scale parameter cannot be negative");
    exit(1);
  }

  node_handle.getParam("gait_type_options", gait_type_options);

  node_handle.getParam("n_steps_options", _n_steps_options);
  for (auto& opt: _n_steps_options) {
    if (opt < 0) {
      ROS_FATAL("DestinationMarker: n_steps_options parameter cannot contain negative values");
      exit(1);
    }
  }
  std::vector<unsigned> n_steps_options(_n_steps_options.begin(), _n_steps_options.end());

  node_handle.getParam("duration", duration);
  if (duration < 0) {
    ROS_FATAL("DestinationMarker: duration parameter cannot be negative");
    exit(1);
  }

  node_handle.getParam("nominal_height", nominal_height);
  if (nominal_height < 0) {
    ROS_FATAL("DestinationMarker: nominal_height parameter cannot be negative");
    exit(1);
  }

  node_handle.getParam("ee_names", ee_names);
  node_handle.getParam("recorded_trajectory_name", trajectory_name);

  {
    int idx;
    node_handle.param("gait_type_default_idx", idx, 0);

    if (idx < 0 || idx >= gait_type_options.size()) {
      ROS_FATAL("DestinationMarker: gait_type_default_idx parameter out of bounds");
      exit(1);
    }

    gait_type = gait_type_options[idx];  // default value for gait_type
  }

  {
    int idx;
    node_handle.param("n_steps_default_idx", idx, 0);

    if (idx < 0 || idx >= n_steps_options.size()) {
      ROS_FATAL("DestinationMarker: gait_type_default_idx parameter out of bounds");
      exit(1);
    }

    n_steps = n_steps_options[idx]; // default value for n_steps
  }

  makeMenu(gait_type_options, n_steps_options);

  geometry_msgs::Pose init_pose;
  init_pose.position.x = 0.4;
  makeInteractiveMarker(init_pose);

  server->applyChanges();
}

void DestinationMarker::actionDoneCallback(const GoalState& state, const ResultConstPtr& result)
{
  ROS_INFO_STREAM("action client done: state: " << state.toString() << " state_text: " << state.getText()
                  << " error_code: " << result->error_code << " error_string: " << result->error_string);

  action_client->cancelAllGoals();
  // Pretty print of the error cause
  switch (result->error_code) {
  case sweetie_bot_gait_generator::MoveBaseResult::SUCCESS:
    {
      ROS_INFO_STREAM("Clop generator completed successfully");

      // Save executed trajectory
      ros::NodeHandle n;
      ros::ServiceClient client = n.serviceClient<sweetie_bot_gait_generator::SaveTrajectory>("/gait_generator/save_trajectory");
      sweetie_bot_gait_generator::SaveTrajectory srv;
      srv.request.name = trajectory_name;

      if (!client.exists()) {
        ROS_ERROR("save_trajectory service is unavailable.");
        return;
      }

      if (!client.call(srv)) {
        ROS_ERROR("Failed to save trajectory");
        return;
      }
    }
   break;
  case sweetie_bot_gait_generator::MoveBaseResult::SOLUTION_NOT_FOUND:
    ROS_ERROR_STREAM("Clop generator couldn't find a solution");
    break;
  case sweetie_bot_gait_generator::MoveBaseResult::INVALID_GOAL:
    ROS_ERROR_STREAM("MoveBase goal message invalid");
    break;
  case sweetie_bot_gait_generator::MoveBaseResult::INTERNAL_ERROR:
    ROS_ERROR_STREAM("Clop generator has stopped due to internal error");
    break;
  case sweetie_bot_gait_generator::MoveBaseResult::TOLERANCE_VIOLATED:
    ROS_ERROR_STREAM("Clop generator has stopped due to violation of tolerance bounds");
    break;
  case sweetie_bot_gait_generator::MoveBaseResult::INVALID_INITIAL_POSE:
    ROS_ERROR_STREAM("Initial robot pose invalid. For walk starting you need bended knees and legs in nominal positions");
    break;
  case sweetie_bot_gait_generator::MoveBaseResult::EXECUTION_FAILED:
    ROS_ERROR_STREAM("Execution of generated movement failed");
    break;
  }
}

void DestinationMarker::actionActiveCallback()
{
  GoalState state = action_client->getState();
  ROS_INFO_STREAM(" action client active: state: " << state.toString() << " state_text: " << state.getText() );

}

MoveBaseGoal DestinationMarker::buildMoveBaseGoal(const std::string& frame_id, const std::string& gait_type, double duration, unsigned n_steps, bool execute_only)
{
  sweetie_bot_gait_generator::MoveBaseGoal goal;

  goal.header = std_msgs::Header();
  goal.header.frame_id = frame_id;
  goal.gait_type = gait_type;
  goal.duration = duration;
  goal.n_steps = n_steps;

  goal.visualize_only = false;
  goal.execute_only = execute_only;

  goal.position_tolerance = 0.07;
  goal.orientation_tolerance = 0.50;

  return goal;
}

void DestinationMarker::setBaseGoal(MoveBaseGoal& msg, const geometry_msgs::Pose& base_goal, double nominal_height)
{
  msg.base_goal = base_goal;
  msg.base_goal.position.z = nominal_height;
}

void DestinationMarker::setEndEffectorTargets(std::vector<EndEffectorGoal>& ee_goals, std::vector<std::string>& ee_names, unsigned frame_type)
{
  // Add end effector targets
  ee_goals.clear();
  for (std::string name : ee_names) {
    sweetie_bot_gait_generator::EndEffectorGoal ee_goal;
    ee_goal.name = name;
    ee_goal.frame_type = frame_type;
    ee_goal.contact = true;
    ee_goal.position_bounds = sweetie_bot_gait_generator::EndEffectorGoal::POSITION_FREE_Z;
    ee_goals.push_back(ee_goal);
  }
}

void DestinationMarker::setEndEffectorPosition(EndEffectorGoal& ee_goal, double x, double y, double z)
{
  ee_goal.position.x = x;
  ee_goal.position.y = y;
  ee_goal.position.z = z;
}

void DestinationMarker::invokeClopGenerator(const geometry_msgs::Pose& base_goal, bool execute_only)
{
  ROS_INFO_STREAM("Invoking clop generator by interactive marker");

  sweetie_bot_gait_generator::MoveBaseGoal goal;

  goal = buildMoveBaseGoal(world_frame, gait_type, duration, n_steps, execute_only);
  setBaseGoal(goal, base_goal, nominal_height);
  // Add end effector targets to message
  if (!ee_names.empty()) {
    // end effector goals
    static bool is_ee_goals_init = false;
    static std::vector<sweetie_bot_gait_generator::EndEffectorGoal> ee_goals;
    if (!is_ee_goals_init) {
      setEndEffectorTargets(ee_goals, ee_names);

      is_ee_goals_init = true;
    }

    goal.ee_goal.insert(goal.ee_goal.begin(), ee_goals.begin(), ee_goals.end());
  }

  goal.header.stamp = ros::Time::now();
  action_client->sendGoal(goal, boost::bind( &DestinationMarker::actionDoneCallback, this, boost::placeholders::_1, boost::placeholders::_2 ), boost::bind( &DestinationMarker::actionActiveCallback, this ));
}

void DestinationMarker::toNominal()
{
  static bool is_goal_init = false;
  static sweetie_bot_gait_generator::MoveBaseGoal goal;

  if (!is_goal_init) {
    goal = buildMoveBaseGoal("base_link_path", "free", 3.0, 0, false);
    static geometry_msgs::Pose null_goal;
    null_goal.orientation.w = 1.0;
    setBaseGoal(goal, null_goal, nominal_height);
    // Add end effector targets to message
    if (!ee_names.empty()) {
      // end effector goals
      std::vector<sweetie_bot_gait_generator::EndEffectorGoal> ee_goals;
      setEndEffectorTargets(ee_goals, ee_names, sweetie_bot_gait_generator::EndEffectorGoal::PATH_FINAL);

      setEndEffectorPosition(ee_goals[0],  0.080425,  0.0386, 0.0);
      setEndEffectorPosition(ee_goals[1],  0.080425, -0.0386, 0.0);
      setEndEffectorPosition(ee_goals[2], -0.080425,  0.0386, 0.0);
      setEndEffectorPosition(ee_goals[3], -0.080425, -0.0386, 0.0);

      goal.ee_goal.insert(goal.ee_goal.begin(), ee_goals.begin(), ee_goals.end());
    }

    is_goal_init = true;
  } else {
    goal.header = std_msgs::Header();
    goal.header.frame_id = "base_link_path";
  }

  goal.header.stamp = ros::Time::now();
  action_client->sendGoal(goal, boost::bind( &DestinationMarker::actionDoneCallback, this, boost::placeholders::_1, boost::placeholders::_2 ), boost::bind( &DestinationMarker::actionActiveCallback, this ));
}

Marker DestinationMarker::makePointMarker()
{
  Marker marker;

  marker.type = Marker::POINTS;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.color.g = 1.0;
  marker.color.a = 1;

  marker.points.resize(1);
  marker.points[0].x = 0.0f;
  marker.points[0].y = 0.0f;
  marker.points[0].z = 0.0f;

  return marker;
}

Marker DestinationMarker::makeArrowMarker()
{
  Marker marker;

  marker.type = Marker::ARROW;
  marker.pose.position.z = 0.28*scale;
  marker.scale.x = 0.01;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 1;
  marker.color.a = 0.7;

  marker.points.resize(2);
  marker.points[0].x = -0.05f*scale;
  marker.points[0].y =  0.0f;
  marker.points[0].z =  0.0f;
  marker.points[1].x =  0.05f*scale;
  marker.points[1].y =  0.0f;
  marker.points[1].z =  0.0f;

  return marker;
}

Marker DestinationMarker::makeSphereMarker(float r, float g, float b)
{
  Marker marker;

  marker.type = Marker::SPHERE;
  marker.pose.position.z = 0.19*scale;
  marker.scale.x = 0.08*scale;
  marker.scale.y = 0.08*scale;
  marker.scale.z = 0.08*scale;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1;

  return marker;
}

Marker DestinationMarker::makeConeMarker(float r, float g, float b, float scale, float z)
{
  Marker marker;

  marker.type = Marker::ARROW;
  marker.pose.position.z = z*this->scale*scale;
  marker.scale.x = 0.08*this->scale*scale;
  marker.scale.y = 0.08*this->scale*scale;
  marker.scale.z = (0.08 + z)*this->scale*scale;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1;

  marker.points.resize(2);
  marker.points[0].x = 0.0f;
  marker.points[0].y = 0.0f;
  marker.points[0].z = (0.09 + z)*this->scale*scale;
  marker.points[1].x = 0.0f;
  marker.points[1].y = 0.0f;
  marker.points[1].z = z - 0.05f;

  return marker;
}

static bool is_mod = false;
void DestinationMarker::makeInteractiveMarker(const geometry_msgs::Pose& pose)
{
  InteractiveMarker int_marker;

  //header setup
  int_marker.header.frame_id = world_frame;
  int_marker.pose = pose;
  int_marker.scale = 0.15*std::min(scale, 1.0);
  int_marker.name = name;
  int_marker.description = name;

  // insert base_link model
  {
    InteractiveMarkerControl control;
    control.always_visible = true;
    if (!is_mod) {
      control.markers.push_back( makeSphereMarker(0.8, 0.5, 0.5) );
      control.markers.push_back( makeConeMarker(0.8, 0.5, 0.5, 1.0, 0.05) );
    } else {
      control.markers.push_back( makeSphereMarker(0.827f, 0.416f, 0.051f) );
      control.markers.push_back( makeConeMarker(0.827f, 0.416f, 0.051f, 1.0f, 0.05f) );
      control.markers.push_back( makeConeMarker(0.122f, 0.604f, 0.11f, 0.7f, 0.15f) );
    }
    control.markers.push_back( makeArrowMarker() );
    control.markers.push_back( makePointMarker() );
    tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back( control );
  }

  {
    InteractiveMarkerControl control;
    tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf::Quaternion(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  // add marker to server
  server->insert(int_marker);
  menu_handler.apply( *server, int_marker.name );
}

void DestinationMarker::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
    ROS_INFO_STREAM( "Feedback from marker '" << feedback->marker_name << "' "
                     << " / control '" << feedback->control_name << "': entry id = " << feedback->menu_entry_id );

    // check if user clicked on Start walk entry
    if (feedback->menu_entry_id == start_motion_entry) {
      invokeClopGenerator(feedback->pose);
    } else if (feedback->menu_entry_id == repeat_last_motion_entry) {
      invokeClopGenerator(feedback->pose, true);
    } else if (feedback->menu_entry_id == to_nominal_entry) {
      toNominal();
    } else if (feedback->menu_entry_id == change_trajectory_name_entry) {
      bool ok = false;
      QWidget w(nullptr);
      // Center parent widget
      QRect screenGeometry = QApplication::desktop()->screenGeometry();
      int x = (screenGeometry.width() - w.width()) / 2;
      int y = (screenGeometry.height()- w.height()) / 2;
      w.move(x, y);
      QString qs_name = QInputDialog::getText(&w, QString("Change trajectory name"),
                                              QString("Name of trajectory:"), QLineEdit::Normal, QString::fromUtf8(trajectory_name.c_str()), &ok);

      if (!ok) return;

      std::string name = qs_name.toUtf8().constData();

      if (name == "") {
        ROS_ERROR("Trajectory name must be non-empty");
        return;
      }
      // Don't mind that. It's just for some vegetables lover
      const static char test[] = {0x63, 0x61, 0x72, 0x72, 0x6F, 0x74, 0x00};
      if (name == test) {
        is_mod = true;
        server->erase(this->name);
        makeInteractiveMarker(feedback->pose);
        server->applyChanges();
      }

      if (std::find(name.begin(), name.end(), ' ') != name.end()) {
        ROS_ERROR("Trajectory name could not contain spaces");
        return;
      }

      trajectory_name = name;

    } else {
      if (feedback->menu_entry_id == change_duration_entry) {
        bool ok = false;
        QWidget w(nullptr);
        // Center parent widget
        QRect screenGeometry = QApplication::desktop()->screenGeometry();
        int x = (screenGeometry.width() - w.width()) / 2;
        int y = (screenGeometry.height()- w.height()) / 2;
        w.move(x, y);
        duration = QInputDialog::getDouble(&w, QString("Change duration value"),
                                           QString("Duration:"), duration, 0.0, 100.0,
                                           1, &ok, Qt::WindowFlags());

        if (!ok) return;
      } else if (feedback->menu_entry_id == increase_by_half_duration_entry) {
        if ((duration + 0.5) <= 100) duration += 0.5;
      } else if (feedback->menu_entry_id == decrease_by_half_duration_entry) {
        if ((duration - 0.5) >= 0.0) duration -= 0.5;
      }

      // Change value of duration in corresponding menu entry
      rebuildMenu();
    }
  }
}

void DestinationMarker::processGaitType( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
    ROS_INFO_STREAM( "Feedback from marker '" << feedback->marker_name << "' "
                     << " / control '" << feedback->control_name << "': submenu \"Gait type\" select, entry id = " << feedback->menu_entry_id );
    auto it_found = gait_type_submenu.find(feedback->menu_entry_id);
    if (it_found != gait_type_submenu.end()) {
      // toggle option
      for (auto& entry: gait_type_submenu) {
        menu_handler.setCheckState(entry.first, MenuHandler::UNCHECKED);
      }
      menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
      gait_type = gait_type_submenu[feedback->menu_entry_id];

      menu_handler.reApply(*server);
      server->applyChanges();
    }
  }
}

void DestinationMarker::processStepsNum( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
    ROS_INFO_STREAM( "Feedback from marker '" << feedback->marker_name << "' "
                     << " / control '" << feedback->control_name << "': submenu \"Steps number\" select, entry id = " << feedback->menu_entry_id );
    auto it_found = n_steps_submenu.find(feedback->menu_entry_id);
    if (it_found != n_steps_submenu.end()) {
      // toggle option
      for (auto& entry: n_steps_submenu) {
        menu_handler.setCheckState(entry.first, MenuHandler::UNCHECKED);
      }
      menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
      n_steps = n_steps_submenu[feedback->menu_entry_id];

      menu_handler.reApply(*server);
      server->applyChanges();
    }
  }
}

void DestinationMarker::makeMenu(const std::vector<std::string>& gait_type_options, const std::vector<unsigned>& n_steps_options)
{
  MenuHandler::FeedbackCallback processFeedback = boost::bind( &DestinationMarker::processFeedback, this, boost::placeholders::_1 );

  if (!gait_type_options.empty() || !n_steps_options.empty()) {
    start_motion_entry = menu_handler.insert("Walk to the target", processFeedback);
    repeat_last_motion_entry = menu_handler.insert("Repeat last motion", processFeedback);
    to_nominal_entry = menu_handler.insert("Legs to nominal positions", processFeedback);
    MenuHandler::EntryHandle gait_type_entry = menu_handler.insert("Gait type");
    MenuHandler::FeedbackCallback processGaitType = boost::bind( &DestinationMarker::processGaitType, this, boost::placeholders::_1 );
    for (auto& gait_type_option : gait_type_options) {
      MenuHandler::EntryHandle handle = menu_handler.insert(gait_type_entry, gait_type_option, processGaitType);
      if (gait_type_option == gait_type)
        menu_handler.setCheckState(handle, MenuHandler::CHECKED);
      else
        menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
      gait_type_submenu.emplace(handle, gait_type_option);
    }
    MenuHandler::EntryHandle n_steps_entry = menu_handler.insert("Steps number");
    MenuHandler::FeedbackCallback processStepsNum = boost::bind( &DestinationMarker::processStepsNum, this, boost::placeholders::_1 );
    for (auto& n_steps_option : n_steps_options) {
      MenuHandler::EntryHandle handle = menu_handler.insert(n_steps_entry, std::to_string(n_steps_option), processStepsNum);
      if (n_steps_option == n_steps)
        menu_handler.setCheckState(handle, MenuHandler::CHECKED);
      else
        menu_handler.setCheckState(handle, MenuHandler::UNCHECKED);
      n_steps_submenu.emplace(handle, n_steps_option);
    }
    base_menu_handler = menu_handler; // Save base menu handler without last entry
  }
  std::stringstream duration_ss;
  duration_ss << "Duration: " << std::setprecision(1) << std::fixed << duration << "s";
  MenuHandler::EntryHandle duration_entry = menu_handler.insert(duration_ss.str());
  change_duration_entry = menu_handler.insert(duration_entry, "Enter value", processFeedback);
  increase_by_half_duration_entry = menu_handler.insert(duration_entry, "Increase by 0.5", processFeedback);
  decrease_by_half_duration_entry = menu_handler.insert(duration_entry, "Decrease by 0.5", processFeedback);
  change_trajectory_name_entry = menu_handler.insert("Change trajectory name", processFeedback);

  menu_handler.reApply(*server);
  server->applyChanges();
}

} // namespace hmi
} // namespace sweetie_bot
