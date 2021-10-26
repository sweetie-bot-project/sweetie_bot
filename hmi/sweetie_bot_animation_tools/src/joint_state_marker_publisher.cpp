#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <urdf/model.h>

#include <math.h>

using namespace visualization_msgs;
using namespace urdf;

urdf::Model model;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
std::unique_ptr<tf2_ros::TransformListener> tf_listener;

std::vector<std::string> joint_frames({
  "bone11", "bone12", "bone13", "bone14", "bone15",
  "bone21", "bone22", "bone23", "bone24", "bone25",
  "bone31", "bone32", "bone33", "bone34", "bone35",
  "bone41", "bone42", "bone43", "bone44", "bone45",
  "bone51", "bone52", "bone53", "bone54",
});

std::vector<std::string> joint_parent_frames({
  "base_link", "bone11", "bone12", "bone13", "bone14",
  "base_link", "bone21", "bone22", "bone23", "bone24",
  "base_link", "bone31", "bone32", "bone33", "bone34",
  "base_link", "bone41", "bone42", "bone43", "bone44",
  "base_link", "bone51", "bone52", "bone53",
});

std::string frame_to_joint_name(std::string frame_name) {
  std::ostringstream ss_joint_name;
  ss_joint_name << "joint" << frame_name[frame_name.size() - 2] << frame_name[frame_name.size() - 1];
  return ss_joint_name.str();
}

tf2_ros::Buffer tf_buffer;
ros::Publisher pub;
std::string world_frame = "odom_combined";

void publish_position_message(std::string joint_name, std::string frame, double position) {
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header = std_msgs::Header();
  joint_state_msg.header.frame_id = frame;
  joint_state_msg.name = std::vector<std::string>({joint_name});
  joint_state_msg.position = std::vector<double>({position});
  joint_state_msg.velocity = std::vector<double>({0.0});
  joint_state_msg.effort   = std::vector<double>({0.0});

  pub.publish(joint_state_msg);
}

void move_all_to_home() {
  for (int i = 0; i < joint_frames.size(); i++) {
    auto &parent_frame = joint_parent_frames[i];
    auto &frame = joint_frames[i];

    try {
      if (tf_buffer.canTransform(parent_frame, frame, ros::Time(0))) {
        // get transform
        geometry_msgs::TransformStamped T;
        T = tf_buffer.lookupTransform(parent_frame, frame, ros::Time(0));
        // convert to pose
        geometry_msgs::Pose pose;
        pose.position.x = T.transform.translation.x;
        pose.position.y = T.transform.translation.y;
        pose.position.z = T.transform.translation.z;
        pose.orientation = T.transform.rotation;

        // set pose
        server->setPose(frame + "_marker", pose);
      }
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("lookupTransform: %s", ex.what());
    }
  }
}

void process_feedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  bool is_mouse_down = false;

  switch ( feedback->event_type ) {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE: {
      auto &marker_pose = feedback->pose;
      tf::Quaternion quat;
      tf::quaternionMsgToTF(marker_pose.orientation, quat);

      double rpy[3];
      tf::Matrix3x3(quat).getRPY(rpy[0], rpy[1], rpy[2]);

      ROS_DEBUG_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation (rpy) = "
          << rpy[0]
          << ", " << rpy[1]
          << ", " << rpy[2]
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << " sec, "
          << feedback->header.stamp.nsec << " nsec" );
      
          // Extract bounded frame from the marker name
          std::string frame(feedback->marker_name.begin(), feedback->marker_name.end() - 7);

          auto joint_name = frame_to_joint_name(frame);
          auto joint = model.getJoint(joint_name);

          auto &rotation = joint->parent_to_joint_origin_transform.rotation;
          auto &position = joint->parent_to_joint_origin_transform.position;

          // Update marker pose only if it moved with the mouse
          if (is_mouse_down) {
            if (tf_buffer.canTransform(world_frame, frame, ros::Time(0))) {
              geometry_msgs::Pose pose_relative_to_parent;
              pose_relative_to_parent.position.x = position.x;
              pose_relative_to_parent.position.y = position.y;
              pose_relative_to_parent.position.z = position.z;

              // get transform
              geometry_msgs::TransformStamped T;
              T = tf_buffer.lookupTransform(world_frame, frame, ros::Time(0));

              pose_relative_to_parent.orientation = T.transform.rotation;
              server->setPose(feedback->marker_name, pose_relative_to_parent);
            }
          }

          double angle = 0.0;
          char last_char = frame[frame.size() - 1];
          char second_last_char = frame[frame.size() - 2];
          if (second_last_char != '5') { // Legs handling
            switch (last_char) {
            case '1':
            case '5':
              angle = -rpy[0];
              break;
            case '2':
            case '3':
            case '4':
              angle = -rpy[1];
              break;
            default:
              angle = 0.0;
              break;
            }
          } else { // Head handling
            switch (last_char) {
            case '1':
            case '3':
              angle = -rpy[1];
              break;
            case '2':
              angle = rpy[2];
              break;
            case '4':
              angle = rpy[0];
              break;
            default:
              angle = 0.0;
              break;
            }
          }

          // Cutting angle off to the limits
          angle = std::max(angle, joint->limits->lower);
          angle = std::min(angle, joint->limits->upper);

          publish_position_message(joint_name, feedback->header.frame_id, angle);
      }
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_DEBUG_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      is_mouse_down = true;
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_DEBUG_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      is_mouse_down = false;
      break;
  }

  server->applyChanges();
}

#define X_AXIS 0x1
#define Y_AXIS 0x2
#define Z_AXIS 0x4

void makeJointControlMarker(std::string home_frame, std::string parent_frame, int movable_axis)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = parent_frame;
  int_marker.scale = 0.06;

  InteractiveMarkerControl control;

  int_marker.name = home_frame + "_marker";
  int_marker.description = "";

  // @Note: Only one axis is available per marker
  if (movable_axis & X_AXIS) {
    control.orientation.w = 0.71;
    control.orientation.x = 0.71;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
  } else if (movable_axis & Z_AXIS) {
    control.orientation.w = 0.71;
    control.orientation.x = 0;
    control.orientation.y = 0.71;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
  } else if (movable_axis & Y_AXIS) {
    control.orientation.w = 0.71;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 0.71;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &process_feedback);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "animation_control");
  ros::NodeHandle nh;

  pub = nh.advertise<sensor_msgs::JointState>("/motion/controller/joint_state/in_joints_ref", 1);

  tf_listener.reset(new tf2_ros::TransformListener(tf_buffer));
  server.reset(new interactive_markers::InteractiveMarkerServer("hmi","",false));

  ros::Duration(0.1).sleep();

  std::string robot_description;
  if (!nh.getParam("/robot_description", robot_description)) {
    ROS_ERROR_STREAM("joint_state_marker_publisher: cannot load URDF string from /robot_description");
    exit(1);
  }
  if (!model.initString(robot_description)) {
    ROS_ERROR_STREAM("joint_state_marker_publisher: failed to parse URDF model");
    exit(1);
  }

  for (int i = 0; i < 4; i++) {
    makeJointControlMarker(joint_frames[0 + i*5], joint_parent_frames[0 + i*5], X_AXIS);
    makeJointControlMarker(joint_frames[1 + i*5], joint_parent_frames[1 + i*5], Y_AXIS);
    makeJointControlMarker(joint_frames[2 + i*5], joint_parent_frames[2 + i*5], Y_AXIS);
    makeJointControlMarker(joint_frames[3 + i*5], joint_parent_frames[3 + i*5], Y_AXIS);
    makeJointControlMarker(joint_frames[4 + i*5], joint_parent_frames[4 + i*5], X_AXIS);
  }

  makeJointControlMarker(joint_frames[20], joint_parent_frames[20], Y_AXIS);
  makeJointControlMarker(joint_frames[21], joint_parent_frames[21], Z_AXIS);
  makeJointControlMarker(joint_frames[22], joint_parent_frames[22], Y_AXIS);
  makeJointControlMarker(joint_frames[23], joint_parent_frames[23], X_AXIS);

  // @Note: That sleep is IMPORTANT! Markers need time to be created
  ros::Duration(0.5).sleep();

  // @Note: Moving markers ONLY after sleep above
  move_all_to_home();

  server->applyChanges();

  ros::spin();

  server.reset();
}
