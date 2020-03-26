#ifndef POSE_MARKER_BASE_HPP
#define POSE_MARKER_BASE_HPP

#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

using interactive_markers::MenuHandler;

namespace sweetie_bot {
namespace hmi {


class PoseMarkerBase {
public:
  PoseMarkerBase(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                 const std::string& name = "",
                 double scale = 1.0,
                 const std::string& marker_home_frame = "",
                 double normalized_z_level = 0.0
                 )
    : server(server),
      tf_listener( new tf2_ros::TransformListener(tf_buffer) ),
      name(name),
      scale(scale),
      marker_home_frame(marker_home_frame),
      normalized_z_level(normalized_z_level)
  {
  }

  PoseMarkerBase(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
                 ros::NodeHandle node_handle
                 )
    : server(server),
      tf_listener( new tf2_ros::TransformListener(tf_buffer) ),
      name(""),
      scale(1.0),
      marker_home_frame(""),
      normalized_z_level(0.0)
  {
    node_handle.getParam("name", name);

    node_handle.getParam("scale", scale);
    if (scale < 0) {
      ROS_FATAL("PoseMarkerBase: scale parameter cannot be negative");
      exit(1);
    }

    node_handle.getParam("frame", marker_home_frame);

    node_handle.getParam("normalized_z_level", normalized_z_level);
    if (normalized_z_level < 0) {
      ROS_FATAL("PoseMarkerBase: normalized_z_level parameter cannot be negative");
      exit(1);
    }
  }

  virtual ~PoseMarkerBase() = 0;

protected:
  void processEnable6DOF( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
  void processNormalize( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback, const ros::Publisher& pose_pub, bool publish_pose);
  void processMoveToHomeFrame( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
  void processMoveToFrame( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );

  void updateInteractiveMarker(bool is6DOF);
  void makeInteractiveMarker(visualization_msgs::Marker (*makeMarkerBody)(double scale),
                             const MenuHandler::FeedbackCallback& processFeedback,
                             bool is6DOF = true);

  virtual void makeMenu() = 0;

public:
  void changeVisibility(bool isVisible);
  void changeColor(float r, float g, float b, float a = 0.7);
  void moveToFrame(const std::string& frame);
  void normalize(geometry_msgs::PoseStamped& pose_stamped);

  void reloadMarker() { if (is_visible) server->get(name, int_marker); }

  std::string const & getMarkerHomeFrame() const { return marker_home_frame; }
  visualization_msgs::InteractiveMarker const & getInteractiveMarker() const { return int_marker; }
  bool isVisible() const { return is_visible; }

protected:
  // COMPONENT INTERFACE

  // CONNECTIONS
  // tf listener
  tf2_ros::Buffer tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;

  // PARAMETERS
  // marker name
  std::string name;
  // marker sacle parameter
  double scale;
  // Home frame to place marker on start operation. Leave empty to ???
  std::string marker_home_frame;
  // Basic z level. When `Normilize pose` command is executed the marker is placed parallel Oxy plane on normalized_z_level heigh over it.
  double normalized_z_level;

  // COMPONENT STATE
  // interactive marker server
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  // interactive marker itself
  visualization_msgs::InteractiveMarker int_marker;
  // dummy interactive marker for real marker replacement in changeVisibility()
  visualization_msgs::InteractiveMarker dummy_marker;
  // flag, indicating "visibility" of interactive marker
  bool is_visible = false;
  // is_operational flag
  bool is_operational = false;
  // is_6DOF flag
  bool is_6DOF = true;
  // menu
  MenuHandler menu_handler;
  // menu index
  MenuHandler::EntryHandle normalize_pose_entry;
  MenuHandler::EntryHandle enable_6DOF_entry;
};

} // namespace hmi
} // namespace sweetie_bot

#endif /*POSE_MARKER_BASE_HPP*/
