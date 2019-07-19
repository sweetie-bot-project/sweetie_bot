#include "pose_marker.hpp"

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

using namespace visualization_msgs;
using namespace interactive_markers;

namespace sweetie_bot {
namespace hmi {

PoseMarker::~PoseMarker() {}

void PoseMarker::processEnable6DOF( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT)
  {
    ROS_INFO_STREAM("Feedback from marker '" << feedback->marker_name << "' "
                    << " / control '" << feedback->control_name << "': menu \"Enable 6-DOF\"");

    MenuHandler::CheckState check;
    menu_handler.getCheckState(feedback->menu_entry_id, check);
    switch (check) {
    case MenuHandler::CHECKED:
      menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::UNCHECKED);
      updateInteractiveMarker(false);
      break;
    case MenuHandler::UNCHECKED:
      menu_handler.setCheckState(feedback->menu_entry_id, MenuHandler::CHECKED);
      updateInteractiveMarker(true);
      break;
    }
  }
}

void PoseMarker::processNormalize( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback, const ros::Publisher& pose_pub, bool pose_publish )
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT)
  {
    ROS_INFO_STREAM("Feedback from marker '" << feedback->marker_name << "' "
                    << " / control '" << feedback->control_name << "': menu \"Normalize pose\"");

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = feedback->header;
    pose_stamped.pose = feedback->pose;
    // normalize pose
    normalize(pose_stamped);
    // publish new pose
    if (pose_publish) pose_pub.publish(pose_stamped);

    menu_handler.reApply(*server);
    server->applyChanges();
  }
}

void PoseMarker::processMoveToFrame( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback )
{
	if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
		ROS_INFO_STREAM( "Feedback from marker '" << feedback->marker_name << "' "
                     << " / control '" << feedback->control_name << "': menu \"move to\" select, entry id = " << feedback->menu_entry_id );

		std::string frame_id;
		if (menu_handler.getTitle(feedback->menu_entry_id, frame_id)) {
			moveToFrame(frame_id);

			menu_handler.reApply(*server);
			server->applyChanges();
		}
	}
}

void PoseMarker::updateInteractiveMarker(bool is6DOF)
{
  InteractiveMarker int_marker;
  server->get(name, int_marker);

  if (is6DOF) {
    InteractiveMarkerControl control;
    std::vector<InteractiveMarkerControl> controls;
    // Create controls with old axises orientations
    for ( const auto& old_ctrl : int_marker.controls) {
      if (old_ctrl.name.substr(0, 4) == "move") {
        control.name = "rotate" + old_ctrl.name.substr(4);
        control.orientation = old_ctrl.orientation;
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        controls.push_back(control);
      }
    }
    int_marker.controls.insert(int_marker.controls.end(), controls.begin(), controls.end());
  }
  else {
    auto& controls = int_marker.controls;
    controls.erase(std::remove_if(controls.begin(), controls.end(),
                                  [](const InteractiveMarkerControl& control)
                                  { return control.interaction_mode == InteractiveMarkerControl::ROTATE_AXIS; }), controls.end());
  }

  // remove old marker from server
  server->erase(name);

	// add new marker to server
	server->insert(int_marker);

  menu_handler.reApply(*server);
  server->applyChanges();
}

void PoseMarker::makeInteractiveMarker(Marker (*makeMarkerBody)(double scale), const MenuHandler::FeedbackCallback& processFeedback, bool is6DOF)
{
	InteractiveMarker int_marker;
	//header setup
	int_marker.header.frame_id = "odom_combined";
	//int_marker.pose.position = ...;
	int_marker.scale = 0.15*std::min(scale, 1.0);
	int_marker.name = name;
	int_marker.description = name;

	// insert base_link model
	{
		InteractiveMarkerControl control;
		// TODO: with base mesh
		control.always_visible = true;
		control.markers.push_back( makeMarkerBody(scale) );
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
		int_marker.controls.push_back( control );
	}

	// add iteractive controls
	{
		InteractiveMarkerControl control;
		tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
		orien.normalize();
		tf::quaternionTFToMsg(orien, control.orientation);
    if (is6DOF) {
      control.name = "rotate_x";
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
    }
		control.name = "move_x";
		control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back(control);

		orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
		orien.normalize();
		tf::quaternionTFToMsg(orien, control.orientation);
    if (is6DOF) {
      control.name = "rotate_z";
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
    }
		control.name = "move_z";
		control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back(control);

		orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
		orien.normalize();
		tf::quaternionTFToMsg(orien, control.orientation);
    if (is6DOF) {
      control.name = "rotate_y";
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
    }
		control.name = "move_y";
		control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back(control);
	}

	// add marker to server
	server->insert(int_marker);
	server->setCallback(int_marker.name, processFeedback);
	menu_handler.apply( *server, int_marker.name );
}

void PoseMarker::moveToFrame(const std::string& frame)
{
  try {
    // get transform
    geometry_msgs::TransformStamped T;
    T = tf_buffer.lookupTransform("odom_combined", frame, ros::Time(0));
    // convert to pose
    geometry_msgs::Pose pose;
    pose.position.x = T.transform.translation.x;
    pose.position.y = T.transform.translation.y;
    pose.position.z = T.transform.translation.z;
    pose.orientation = T.transform.rotation;
    // set pose
    server->setPose(name, pose);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("lookupTransform: %s", ex.what());
	}
}

void PoseMarker::normalize(geometry_msgs::PoseStamped pose_stamped)
{
  // normilize pose
  pose_stamped.pose.position.z = normalized_z_level;
  tf::Quaternion orien(0.0, 0.0, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w);
  orien.normalize();
  tf::quaternionTFToMsg(orien, pose_stamped.pose.orientation);
  // set pose of marker
  server->setPose(name, pose_stamped.pose);
}

} // namespace hmi
} // namespace sweetie_bot

