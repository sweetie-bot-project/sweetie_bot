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

#include <sweetie_bot_joint_trajectory_editor/param_msg_loader.h>

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
   Ui::TrajectoryEditor *ui;
   sweetie_bot::interface::JointTrajectoryData *joint_trajectory_data_;

   sweetie_bot::interface::JointListTableView *joint_list_table_view_;
   sweetie_bot::interface::JointTrajectoryPointTableView *joint_trajectory_point_table_view_;

   sweetie_bot::tools::ParamMsgLoader<control_msgs::FollowJointTrajectoryGoal>* loader_;
private slots:
    void rosSpin();

    void on_pushButton_4_clicked();
    void on_pushButton_5_clicked();
    void on_pushButton_6_clicked();
    void on_pushButton_10_clicked();
};

#endif // TRAJECTORYEDITOR_H
