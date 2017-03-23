#ifndef TRAJECTORYEDITOR_H
#define TRAJECTORYEDITOR_H

#include <QMainWindow>
#include <QTimer>
#include <QtMath>
#include <QTableWidgetItem>


// ROS
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "sensor_msgs/JointState.h"

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
    explicit TrajectoryEditor(int argc, char *argv[], ros::NodeHandle node, QWidget *parent = 0);
  ~TrajectoryEditor();
  void bootstrap();
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;
private:
   ros::NodeHandle node_;
   ros::Subscriber sub_real_;
   ros::Subscriber sub_virtual_;
   ros::Publisher pub_joints_virtual_set;
   ros::Publisher pub_joints_marker_set;

   ros::ServiceClient torque_main_switch_;

   sensor_msgs::JointState joint_state_real_;
   sensor_msgs::JointState joint_state_virtual_;

   const std::string trajectories_param_name = "/sweetie_bot_joint_trajectory_editor/trajectories";

   Client *client_virtual;
   Client *client_real;

   Ui::TrajectoryEditor *ui;
   sweetie_bot::interface::JointTrajectoryData *joint_trajectory_data_;

   sweetie_bot::interface::JointListTableView *joint_list_table_view_;
   sweetie_bot::interface::JointTrajectoryPointTableView *joint_trajectory_point_table_view_;

   sweetie_bot::tools::ParamMsgLoader<control_msgs::FollowJointTrajectoryGoal>* loader_;
   void jointsRealCallback(const sensor_msgs::JointState::ConstPtr& msg);
   void jointsVirtualCallback(const sensor_msgs::JointState::ConstPtr& msg);
   void executeActionCallback(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryActionResultConstPtr& result);
   void updateParamList();

private slots:
    void rosSpin();
    void on_turnAllTrajectoryServosButton_clicked();
    void on_turnAllServoOnButton_clicked();
    void on_loadTrajectoryButton_clicked();
    void on_turnAllSelectedServosButton_clicked();
    void on_addRealPoseButton_clicked();
    void on_saveTrajectoryButton_clicked();
    void on_deletePoseButton_clicked();
    void on_addVirtualPoseButton_clicked();
    void on_executeButton_clicked();
    void on_jointsTableView_clicked(const QModelIndex &index);
    void on_addButton_clicked();
    void on_applyButton_clicked();
    void on_delButton_clicked();
    void on_delTrajectoryButton_clicked();
    void on_goalTimeToleranceSpinBox_valueChanged(double arg1);
    void on_setVirtualPoseButton_clicked();
    void on_pointsTableView_clicked(const QModelIndex &index);
    void on_pointsTableView_doubleClicked(const QModelIndex &index);
    void on_virtualCheckBox_clicked(bool checked);
};

#endif // TRAJECTORYEDITOR_H
