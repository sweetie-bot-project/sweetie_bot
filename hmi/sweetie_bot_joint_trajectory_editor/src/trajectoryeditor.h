#ifndef TRAJECTORYEDITOR_H
#define TRAJECTORYEDITOR_H

#include <QMainWindow>
#include <QTimer>
#include <QtMath>
#include <QTableWidgetItem>

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <sweetie_bot_herkulex_msgs/ServoCommands.h>
#include <sweetie_bot_herkulex_msgs/HerkulexState.h>
#include <sweetie_bot_herkulex_msgs/HerkulexJointState.h>

#include "ui_Editor.h"

#include "joint_trajectory_data.h"
#include "joint_list_table_view.h"
#include "joint_trajectory_point_table_view.h"
#include "param_msg_loader.h"

namespace Ui {
class TrajectoryEditor;
}

class TrajectoryEditor : public QMainWindow
{
	Q_OBJECT
	
	public:
		typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;

	public:
		explicit TrajectoryEditor(int argc, char *argv[], ros::NodeHandle node, QWidget *parent = 0);
		void bootstrap();
		~TrajectoryEditor();

	private:
		// ROS node
		ros::NodeHandle node_;

		// pub/sub interface
		ros::Subscriber sub_joints_; 
		ros::Publisher pub_joints_set_;
		ros::Publisher pub_joints_marker_set_;
		ros::Publisher pub_servo_commands_;
		ros::Subscriber sub_servo_states_;
		ros::Subscriber sub_servo_joint_states_;
		// services
		ros::ServiceClient torque_main_switch_;
		// parameteres buffers
		std::string trajectories_param_name;
		// messages buffers
		sensor_msgs::JointState joint_state_;
		// actionlib clients
		ActionClient * action_execute_trajectory_;
		// internals
		Ui::TrajectoryEditor ui;
		sweetie_bot::hmi::JointTrajectoryData joint_trajectory_data_;
		sweetie_bot::hmi::JointListTableModel joint_list_table_model_;
		sweetie_bot::hmi::JointTrajectoryPointTableModel joint_trajectory_point_table_model_;
		QTimer * timer;

	private:
		// ROS callbacks
		void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg);
		void servoJointsCallback(const sweetie_bot_herkulex_msgs::HerkulexJointState::ConstPtr& msg);

		enum JointTorqueState : uint8_t {
			UNINITIALIZED,
			TORQUE_ON,
			TORQUE_OFF
		};
		std::unordered_map<std::string, JointTorqueState> is_joint_torque_on;

		void servoStateCallback(const sweetie_bot_herkulex_msgs::HerkulexState::ConstPtr& msg);
		void executeActionCallback(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryActionResultConstPtr& result);

		// parameters managment
		sweetie_bot::tools::ParamMsgLoader<control_msgs::FollowJointTrajectoryGoal> loader_;
		void updateParamList();
	
		// helper functions
		void setServoTorqueOn(bool value);

		std::array<bool, 5> is_limb_on;
		std::array<std::vector<std::string>, 5> limb_joint_names;
		void handleLimbButtonToggle(int limb_index, QPushButton *button);
		void setLimbServoTorqueOn(int limb_index, bool set_on);

		void updateSelectedPose(bool update_only_disabled = false);

	private slots:
		void rosSpin();
		// servo control
		void on_turnAllServoOnButton_clicked();
		void on_turnAllServoOffButton_clicked();
		// legs servo control
		void on_leg1ToggleButton_clicked();
		void on_leg2ToggleButton_clicked();
		void on_leg3ToggleButton_clicked();
		void on_leg4ToggleButton_clicked();
		void on_headToggleButton_clicked();
		// trajectory managment
		void on_loadTrajectoryButton_clicked();
		void on_saveTrajectoryButton_clicked();
		void on_delTrajectoryButton_clicked();
		// pose managment
		void on_addRobotPoseButton_clicked();
		void on_deletePoseButton_clicked();
		void on_dublicatePoseButton_clicked();
		void on_updateRobotPoseButton_clicked();
		// robot control
		void on_setRobotPoseButton_clicked();
		void on_executeButton_clicked();
		// time scale parameters
		void on_scaleSpinBox_valueChanged(double value);
		void on_scaleSlider_sliderMoved(int value);
		void on_applyScaleButton_clicked();
		// joints managment
		void on_addJointButton_clicked();
		void on_delJointButton_clicked();
		// tolerances
		void on_resetTolerancesButton_clicked();
		void on_goalTimeToleranceSpinBox_valueChanged(double arg1);
		// table interactions 
		void on_jointsTableView_clicked(const QModelIndex &index);
		void on_pointsTableView_clicked(const QModelIndex &index);
		void on_pointsTableView_doubleClicked(const QModelIndex &index);
};

#endif // TRAJECTORYEDITOR_H
