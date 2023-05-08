#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>

#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <tf_conversions/tf_kdl.h>

#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sweetie_bot_kinematics_msgs/JointStateAccel.h>
#include <sweetie_bot_kinematics_msgs/SupportState.h>
#include <sweetie_bot_kinematics_msgs/RigidBodyState.h>
#include <sweetie_bot_kinematics_msgs/BalanceState.h>

#include <sweetie_bot_orocos_misc/joint_state_check.hpp>
#include <sweetie_bot_orocos_misc/message_checks.hpp>

struct ColorRGBAInit : public std_msgs::ColorRGBA
{
	public:
		ColorRGBAInit() : std_msgs::ColorRGBA() {}
		//ColorRGBAInit(const ColorRGBAInit& color) : ColorRGBA(color) {}
		ColorRGBAInit(_r_type _r, _g_type _g, _b_type _b, _a_type _a = 1.0) { r = _r; b = _b; g = _g; a = _a; }
};


class DynamicsVisualizer 
{
	protected:
		typedef sensor_msgs::JointState JointState;
		typedef sweetie_bot_kinematics_msgs::JointStateAccel JointStateAccel;
		typedef sweetie_bot_kinematics_msgs::SupportState SupportState;
		typedef sweetie_bot_kinematics_msgs::RigidBodyState RigidBodyState;
		typedef sweetie_bot_kinematics_msgs::BalanceState BalanceState;
		typedef visualization_msgs::MarkerArray MarkerArray;
		typedef visualization_msgs::Marker Marker;

	protected:
		static const ColorRGBAInit RED;
		static const ColorRGBAInit GREEN;
		static const ColorRGBAInit MAGENTA;
		static const ColorRGBAInit BLUE;
		static const ColorRGBAInit LIGHT_BLUE;
		static const ColorRGBAInit YELLOW;

	protected:
		// NODE INTERFACE
		// NodeHandler
		ros::NodeHandle node_handler;
		// publisers
		ros::Publisher joints_pub;
		ros::Publisher markers_pub;
		// subscribers
		ros::Subscriber joints_accel_sub;
		ros::Subscriber supports_sub;
		ros::Subscriber wrenches_sub;
		ros::Subscriber base_sub;
		ros::Subscriber balance_sub;
		// tf
		tf::TransformListener tf_listener;
		// timer
		ros::Timer timer;

		// PARAMETERS
		std::string robot_model_ns_param;
		double point_size_param;
		double torque_scale_param;
		double force_scale_param;
		double velocity_angular_scale_param;
		bool display_twist;

		// robot_model parameters cache
		std::map<std::string, KDL::Vector> contact_points_cache;

		// BUFFERS
		SupportState supports;
		RigidBodyState wrenches;
		RigidBodyState base;
		BalanceState balance;

	public:
		DynamicsVisualizer()
		{
			// input
			joints_accel_sub = node_handler.subscribe<JointStateAccel>("joint_state_accel", 1, &DynamicsVisualizer::callbackJointsAccelSub, this);
			base_sub = node_handler.subscribe<RigidBodyState>("base", 1, &DynamicsVisualizer::callbackBaseSub, this);
			wrenches_sub = node_handler.subscribe<RigidBodyState>("wrenches", 1, &DynamicsVisualizer::callbackWrenchesSub, this);
			supports_sub = node_handler.subscribe<SupportState>("supports", 1, &DynamicsVisualizer::callbackSupportsSub, this);
			balance_sub = node_handler.subscribe<BalanceState>("balance", 1, &DynamicsVisualizer::callbackBalanceSub, this);
			// output
			markers_pub = node_handler.advertise<MarkerArray>("marker_array", 3);
			joints_pub = node_handler.advertise<JointState>("joint_efforts", 1);	

			// get node parameters
			if (!ros::param::get("~robot_model_namespace", robot_model_ns_param)) {
				robot_model_ns_param = "";
			}
			if (!ros::param::get("~point_size", point_size_param)) {
				point_size_param = 0.005;
			}
			if (!ros::param::get("~torque_scale", torque_scale_param)) {
				torque_scale_param = 0.01;
			}
			if (!ros::param::get("~force_scale", force_scale_param)) {
				force_scale_param = 0.01;
			}
			if (!ros::param::get("~display_twist", display_twist)) {
				display_twist = true;
			}
			if (!ros::param::get("~velocity_angular_scale", velocity_angular_scale_param)) {
				velocity_angular_scale_param = 1.0/(2.0*M_PI);
			}

			// timer
			timer = node_handler.createTimer(ros::Duration(0.05), &DynamicsVisualizer::loop, this);
			timer.start();

			ROS_INFO("SweeterBot dynamics visualizer started!");
		}


		std::string getContactFrame(const std::string& name) {
			std::string frame;
			if (node_handler.getParamCached(robot_model_ns_param + "/robot_model/chains/" + name + "/last_link", frame)) {
				return frame;
			}
			else throw ros::Exception("Unable to determine last_link frame name of " + name + " kinematic chain. Check if robot_model is loaded onto Parameter Service in namespace '" + robot_model_ns_param + "'.");
		}

		KDL::Vector getContactPoint(const std::string& name) {
			// try to find parameter in cache
			auto it = contact_points_cache.find(name);
			if (it != contact_points_cache.end()) return it->second;
			// contact not found. Request it from robot model
			KDL::Vector point;
			bool success = true;
			success = success && node_handler.getParam(robot_model_ns_param + "/robot_model/contacts/" + name + "/points/Element0/X", point[0]);
			success = success && node_handler.getParam(robot_model_ns_param + "/robot_model/contacts/" + name + "/points/Element0/Y", point[1]);
			success = success && node_handler.getParam(robot_model_ns_param + "/robot_model/contacts/" + name + "/points/Element0/Z", point[2]);
			if (success) {
				contact_points_cache[name] = point;
				return point;
			}
			else throw ros::Exception("Unable to get contact point " + name + ". Check if robot_model is loaded into Parameter Service and contact point exists.");
		}

		void callbackJointsAccelSub(const JointStateAccel::ConstPtr& msg)
		{
			// copy to joints state 
			JointState joints;
			joints.header = msg->header;
			joints.name = msg->name;
			joints.position = msg->position;
			joints.velocity = msg->velocity;
			joints.effort = msg->effort;
			//TODO typestamp? 
			// publish
			joints_pub.publish(joints);
		}

		void callbackSupportsSub(const SupportState::ConstPtr& msg) 
		{
			if (sweetie_bot::isValidSupportStateNameSuppCont(*msg)) {
				// buffer message for following vizualization
				supports = *msg;
			}
		}

		void callbackWrenchesSub(const RigidBodyState::ConstPtr& msg) 
		{
			if (sweetie_bot::isValidRigidBodyStateNameFrame(*msg)) {
				// buffer message for following vizualization
				wrenches = *msg;
			}
		}

		void callbackBaseSub(const RigidBodyState::ConstPtr& msg) 
		{
			if (sweetie_bot::isValidRigidBodyStateNameFrame(*msg, 1)) {
				// buffer message for following vizualization
				base = *msg;
			}
		}

		void callbackBalanceSub(const BalanceState::ConstPtr& msg) 
		{
			balance = *msg;
		}

		void loop(const ros::TimerEvent&) 
		{
			if (supports.name.size() > 0) visualizeSupports();
			if (wrenches.name.size() > 0) visualizeWrenches();
			visualizeBalance();
		}

		void visualizeSupports() 
		{
			// prepare Markers message
			MarkerArray marker_array;

			// add contact points
			marker_array.markers.resize(1);
			Marker& marker_points = marker_array.markers[0];
			// message with points
			marker_points.header.frame_id = "odom_combined"; // contact points are displayed in world frame
			marker_points.header.stamp = ros::Time::now();
			marker_points.ns = "contacts";
			marker_points.id = 0;
			marker_points.type = visualization_msgs::Marker::POINTS;
			marker_points.action = 0; // add/modify 
			marker_points.pose.orientation.w = 1.0; // other elements are zeros
			marker_points.scale.x = point_size_param; marker_points.scale.y = point_size_param; marker_points.scale.z = 0.0;
			marker_points.color = RED;
			marker_points.lifetime = ros::Duration(1.0); // one second
			marker_points.frame_locked = true; // odom_combined is fixed frame

			for(int k = 0; k < supports.name.size(); k++) {
				if (supports.contact[k] == "") continue;
				try {
					geometry_msgs::PointStamped point_stamped;
					// get point information
					point_stamped.header.frame_id = getContactFrame(supports.name[k]);
					tf::pointKDLToMsg(getContactPoint(supports.contact[k]), point_stamped.point);
					// convert to world frame
					tf_listener.transformPoint("odom_combined", point_stamped, point_stamped); 
					// add point marker
					marker_points.points.push_back(point_stamped.point);
					// check if point support 
					if (supports.support[k] > 0.0) {
						// set color: point is in contact
						marker_points.colors.push_back(RED);
					}
					else {
						// set color: point is not in contact
						marker_points.colors.push_back(GREEN);
					}
				}
				catch (tf::TransformException& e) {
					ROS_ERROR("tf error: %s", e.what());
					return;
				}
				catch (ros::Exception& e) { // getContactFrame or getContactPoint failed
					ROS_ERROR("ROS error: %s", e.what());
				}
			}
			// publish resulting message
			markers_pub.publish(marker_array);
		}

		void visualizeWrenches() 
		{
			// prepare Markers message
			MarkerArray marker_array;
			// add contact points
			marker_array.markers.resize(1);
			Marker& marker = marker_array.markers[0];
			// message contact forces vizualization
			marker.header.frame_id = "odom_combined"; // forces are supplied in base_link frame
			marker.header.stamp = ros::Time::now();
			marker.ns = "limbs_external_forces_and_twists";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::LINE_LIST;
			marker.action = 0; // add/modify 
			marker.pose.orientation.w = 1.0; // other elements are zeros
			marker.scale.x = point_size_param/2; marker.scale.y = 0.0; marker.scale.z = 0.0;
			marker.color = RED;
			marker.lifetime = ros::Duration(1.0); // one second
			marker.frame_locked = true; // odom_combined is fixed frame

			// RigidBodyState messge contains all objects in world frame coordinates
			for(int k = 0; k < wrenches.wrench.size(); k++) {
					// get wrench and bring it to limb tip
					KDL::Wrench wrench = wrenches.wrench[k];
					KDL::Vector point = wrenches.frame[k].p;
					wrench = wrench.RefPoint(point);
					// wrench visualization
					KDL::Vector point_force = point + wrench.force*force_scale_param; // force visualization
					KDL::Vector point_torque = point + wrench.torque*torque_scale_param; // torque visualization
					// add to line list
					marker.points.emplace_back(); tf::pointKDLToMsg(point, marker.points.back()); marker.colors.push_back(RED);
					marker.points.emplace_back(); tf::pointKDLToMsg(point_force, marker.points.back()); marker.colors.push_back(RED);
					marker.points.emplace_back(); tf::pointKDLToMsg(point, marker.points.back()); marker.colors.push_back(BLUE);
					marker.points.emplace_back(); tf::pointKDLToMsg(point_torque, marker.points.back()); marker.colors.push_back(BLUE);
			}
			if (display_twist) {
				// display limbs twists
				for(int k = 0; k < wrenches.twist.size(); k++) {
					// get velocity and bring it to limb tip
					KDL::Twist twist = wrenches.twist[k];
					KDL::Vector point = wrenches.frame[k].p;
					twist = twist.RefPoint(point);
					// velocity visualization
					KDL::Vector point_vel = point + twist.vel; // linear vel visualization
					KDL::Vector point_rot = point + twist.rot*velocity_angular_scale_param; // angular vel visualization
					// add to line list
					marker.points.emplace_back(); tf::pointKDLToMsg(point, marker.points.back()); marker.colors.push_back(GREEN);
					marker.points.emplace_back(); tf::pointKDLToMsg(point_vel, marker.points.back()); marker.colors.push_back(GREEN);
					marker.points.emplace_back(); tf::pointKDLToMsg(point, marker.points.back()); marker.colors.push_back(YELLOW);
					marker.points.emplace_back(); tf::pointKDLToMsg(point_rot, marker.points.back()); marker.colors.push_back(YELLOW);
				}
				// display base twist
				if (base.name.size() == 1) {
					KDL::Twist twist = base.twist[0];
					KDL::Vector point = base.frame[0].p;
					twist = twist.RefPoint(point);
					// velocity visualization
					KDL::Vector point_vel = point + twist.vel; // linear vel visualization
					KDL::Vector point_rot = point + twist.rot*velocity_angular_scale_param; // angular vel visualization
					// add to line list
					marker.points.emplace_back(); tf::pointKDLToMsg(point, marker.points.back()); marker.colors.push_back(GREEN);
					marker.points.emplace_back(); tf::pointKDLToMsg(point_vel, marker.points.back()); marker.colors.push_back(GREEN);
					marker.points.emplace_back(); tf::pointKDLToMsg(point, marker.points.back()); marker.colors.push_back(YELLOW);
					marker.points.emplace_back(); tf::pointKDLToMsg(point_rot, marker.points.back()); marker.colors.push_back(YELLOW);
				}
			}
			// publish resulting message
			markers_pub.publish(marker_array);
		}

		void visualizeBalance() {
			// prepare Markers message
			MarkerArray marker_array;
			marker_array.markers.resize(2);
			Marker& marker_zmp = marker_array.markers[0];
			Marker& marker_lines = marker_array.markers[1];
			// message with CoP an point
			marker_zmp.header.frame_id = "odom_combined"; // we can calculate ZMP only on world frame
			marker_zmp.header.stamp = ros::Time::now();
			marker_zmp.ns = "balance";
			marker_zmp.id = 0;
			marker_zmp.type = visualization_msgs::Marker::POINTS;
			marker_zmp.action = 0; // add/modify 
			marker_zmp.pose.orientation.w = 1.0; // other elements are zeros
			marker_zmp.scale.x = point_size_param; marker_zmp.scale.y = point_size_param; marker_zmp.scale.z = 0.0;
			marker_zmp.color = GREEN;
			marker_zmp.lifetime = ros::Duration(1.0); // one second
			marker_zmp.frame_locked = true; // odom_combined is fixed frame
			// message with support polygone
			marker_lines = marker_zmp;
			marker_lines.id = 1;
			marker_lines.type = visualization_msgs::Marker::LINE_STRIP;
			marker_lines.scale.x = point_size_param/2; marker_lines.scale.y = 0.0; marker_lines.scale.z = 0.0;
			marker_lines.color = GREEN;
			
			// use balance message to display CoP and CoM
			// contact are assumed to be positioned in z = 0 plane
			// CoP (z = 0)
			marker_zmp.points.emplace_back(); 
			tf::pointKDLToMsg(balance.CoP, marker_zmp.points.back());
			marker_zmp.colors.push_back(MAGENTA);
			// CoM projection (z = 0)
			marker_zmp.points.emplace_back(); 
			tf::pointKDLToMsg(balance.CoM, marker_zmp.points.back());
			marker_zmp.points.back().z = 0.0;
			marker_zmp.colors.push_back(LIGHT_BLUE);
			// CoM 
			marker_zmp.points.emplace_back(); 
			tf::pointKDLToMsg(balance.CoM, marker_zmp.points.back());
			marker_zmp.colors.push_back(LIGHT_BLUE);

			// now add support polygone
			for(int k = 0; k < balance.support_points.size(); k++) {
				geometry_msgs::Point point;
				tf::pointKDLToMsg(balance.support_points[k], point);
				marker_lines.points.push_back(point);
			}
			// close support polygon
			if (marker_lines.points.size() >= 2) {
				marker_lines.points.push_back(marker_lines.points.front());
			}

			// publish resulting message
			markers_pub.publish(marker_array);
		}
};


const ColorRGBAInit DynamicsVisualizer::RED = ColorRGBAInit(1 ,0, 0);
const ColorRGBAInit DynamicsVisualizer::GREEN = ColorRGBAInit(0, 1, 0);
const ColorRGBAInit DynamicsVisualizer::MAGENTA = ColorRGBAInit(1, 0, 1);
const ColorRGBAInit DynamicsVisualizer::BLUE = ColorRGBAInit(0, 0, 1);
const ColorRGBAInit DynamicsVisualizer::LIGHT_BLUE = ColorRGBAInit(0, 1, 1);
const ColorRGBAInit DynamicsVisualizer::YELLOW = ColorRGBAInit(1, 1, 0);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sweetie_dynamics_visualizer");
	ROS_INFO("SweetieBot Dynamics Visualizer main.");

	std::unique_ptr<DynamicsVisualizer> visualizer(new DynamicsVisualizer());

	ros::spin();

	return 0;
}

