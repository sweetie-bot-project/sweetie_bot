#ifndef TRAJECTORYEDITOR_H
#define TRAJECTORYEDITOR_H

#include <QMainWindow>
#include <QTimer>

// ROS
#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryGoal.h"
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
private:
   ros::NodeHandle node_;
   ros::Subscriber sub_real_;
   ros::Subscriber sub_virtual_;
   sensor_msgs::JointState joint_state_real_;
   sensor_msgs::JointState joint_state_virtual_;

   Ui::TrajectoryEditor *ui;
   sweetie_bot::interface::JointTrajectoryData *joint_trajectory_data_;

   sweetie_bot::interface::JointListTableView *joint_list_table_view_;
   sweetie_bot::interface::JointTrajectoryPointTableView *joint_trajectory_point_table_view_;

   sweetie_bot::tools::ParamMsgLoader<control_msgs::FollowJointTrajectoryGoal>* loader_;
   void jointsRealCallback(const sensor_msgs::JointState::ConstPtr& msg);
   void jointsVirtualCallback(const sensor_msgs::JointState::ConstPtr& msg);
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
};

#endif // TRAJECTORYEDITOR_H
