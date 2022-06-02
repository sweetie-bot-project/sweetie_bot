#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QTimer>
#include <QOpenGLWidget>

#include <QHBoxLayout>
#include <QApplication>

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/JointState.h>
#include <sweetie_bot_text_msgs/TextCommand.h>

#include "eyewindow.h"

class MainWindow : public QOpenGLWidget {
    Q_OBJECT

private:
    bool m_publishPixmap = false;

    EyeWindow *m_leftEye;
    EyeWindow *m_rightEye;
    QHBoxLayout *m_uiLayout;

    QTimer *m_moveTimer;
    QTimer *m_blinkTimer;
    QTimer *m_updateTimer;

    // ROS
    ros::NodeHandle node_;
    ros::Subscriber sub_joint_state_;
    ros::Subscriber sub_control_;

    void controlCallback(const sweetie_bot_text_msgs::TextCommand::ConstPtr& msg);
    void moveCallback(const sensor_msgs::JointState::ConstPtr& msg);

public:
    MainWindow(QWidget *parent = 0);

    void moveBothEyes(MoveFlags flags, int ms, EyeState targetStateLeft, EyeState targetStateRight, bool moveWithBlink = false);
    void blinkBothEyes(int ms);

    EyeState generateEmotion(const EyeState &baseState, const char *emotionName);

private slots:

    void Update();

    // ROS
    void rosSpin();
};

#endif // MAINWINDOW_H
