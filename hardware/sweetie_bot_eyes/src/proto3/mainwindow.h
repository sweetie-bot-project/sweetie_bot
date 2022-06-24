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

#include <boost/math/distributions/lognormal.hpp>
#include <random>

#include <unordered_map>

class MainWindow : public QOpenGLWidget {
    Q_OBJECT

private:
    bool m_publishPixmap = false;

    int m_autoBlinkDelay;

    EyeWindow *m_leftEye;
    EyeWindow *m_rightEye;
    QHBoxLayout *m_uiLayout;

    QTimer *m_moveTimer;
    QTimer *m_blinkTimer;
    QTimer *m_randomMovementTimer;

    EyeState m_leftSavedState;
    EyeState m_rightSavedState;

    EyesAnimation *m_playingAnimation;

    int  m_animationStage = 0; // Zero means animation is not running

    bool m_restartAutoblink = false;
    bool m_delayedBlinkWaiting = false;
    bool m_delayedMoveWaiting  = false;

    // Natural random blinkng generation
    bool m_generationEnabled = false;
    int m_blinkGenerationTime;
    int m_msBlinkGenerationInterval;
    int m_timeUntilNextBlink;

    boost::math::lognormal_distribution<> m_blinkTimeDistribution;
    std::uniform_real_distribution<double> m_uniformDist;
    std::random_device m_rd;
    std::default_random_engine m_noiseGenerator;

    std::unordered_map<std::string, EyesAnimation *> m_animations;

    // ROS
    ros::NodeHandle node_;
    ros::Subscriber sub_joint_state_;
    ros::Subscriber sub_control_;

    void controlCallback(const sweetie_bot_text_msgs::TextCommand::ConstPtr& msg);
    void moveCallback(const sensor_msgs::JointState::ConstPtr& msg);

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void initAnimations();
    void playAnimation(EyesAnimation *animation);
    void animationUpdate();

    void moveBothEyes(MoveFlags flags, EyeAnimation *targetSequenceLeft, EyeAnimation *targetSequenceRight);
    void moveBothEyes(MoveFlags flags, int ms, EyeState targetStateLeft, EyeState targetStateRight, bool moveWithBlink = false);
    void blinkBothEyes(int ms);

    EyeState generateEmotion(const EyeState &baseState, const char *emotionName);

private slots:
    void randomMovementGeneration();

    void updateBlink();
    void updateMove();

    // ROS
    void rosSpin();
};

#endif // MAINWINDOW_H
