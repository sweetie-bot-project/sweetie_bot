#ifndef TRAJECTORYEDITOR_H
#define TRAJECTORYEDITOR_H

#include <QMainWindow>
#include <QTimer>

// ROS
#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryGoal.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

// file io
#include <fstream> 

namespace Ui {
class TrajectoryEditor;
}

class TrajectoryEditor : public QMainWindow
{
    Q_OBJECT

public:
    explicit TrajectoryEditor(int argc, char *argv[], QWidget *parent = 0);
    ~TrajectoryEditor();
private:
    void jointsRealCallback(const sensor_msgs::JointState::ConstPtr& msg);

   Ui::TrajectoryEditor *ui;
private slots:
    void rosSpin();

};

#endif // TRAJECTORYEDITOR_H
