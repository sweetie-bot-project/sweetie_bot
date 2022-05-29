#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <QOpenGLWidget>
#include <QOpenGLFramebufferObject>
#include <QOpenGLPaintDevice>

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sweetie_bot_text_msgs/TextCommand.h>

#include "eye_state.h"

class MainWindow : public QOpenGLWidget {
    Q_OBJECT

private:
    bool m_isLeftEye;
    bool m_publishPixmap;
    bool m_debug_mode_enabled = false;

    QOpenGLFramebufferObject *m_fbo;

    EyeState m_state;
    EyeState m_startAnimationState;
    EyeState m_endAnimationState;

    //octagon points
    QVector<QPointF>m_Pin;
    QVector<QPointF>m_Pout;

    //blinking
    int m_blinkDefaultDuration;
    int m_blinkDuration;
    int m_blinkDelay;
    int m_currentBlinkingTime;
    bool m_isBlinking;
    bool m_isGoingDown;
    bool m_isMoveWithBlink;

    //delete in next versions
    int m_msBetweenMovement;
    QTimer* m_randomMoveTimer;

    QPainterPath m_topEyelidPath;
    QPainterPath m_bottomEyelidPath;
    QTransform m_topEyelidTransform;
    QTransform m_bottomEyelidTransform;

    QTimer* m_blinkTimer;
    QTimer* m_moveTimer;

    enum EyePath {
        GreenEllipse,
        BlackOctagonAndLines,
        WhiteArea,

        EyePathCount = 3
    };
    QVector<QPainterPath> m_eyePaths;
    QTransform m_eyeTransform;

    QVector<QPainterPath> m_shinesPaths;

    MoveFlags m_moveFlags;

    //movement
    int m_currentMovingTime;   //how long is eye moving now
    int m_movingTime;          //how long the eye should travel
    int m_msUpdateMove;

    QString path_;
    QImage *overlay_;

    // ROS
    ros::NodeHandle node_;
    ros::Subscriber sub_joint_state_;
    ros::Subscriber sub_control_;
    ros::Publisher  pub_eye_image_;

    void controlCallback(const sweetie_bot_text_msgs::TextCommand::ConstPtr& msg);
    void moveCallback(const sensor_msgs::JointState::ConstPtr& msg);

public:
    MainWindow(bool isLeftEye = true, QWidget *parent = 0);
    ~MainWindow();

    void PublishImage();

    QPointF rotatePoint(QPointF point, QPointF center, float angle);

    void computeFrame();
    void computeEye();
    void computeEyeTransform();
    void computeEyelidTransform();
    void computeShines();
    void computeEyelid();
    QPainterPath computeShinePath(int dx, int dy, int r1, int r2, int angle);

    void move(MoveFlags flags, int ms, EyeState targetState);
    void blink(int ms);

    void keyPressEvent(QKeyEvent *e);

    void initializeGL();
    void paintGL();

private slots:
    void updateMovingState();
    void updateBlinkState();

    //delete in next versions
    void computeRandomMove();

    // ROS
    void rosSpin();
};

#endif // MAINWINDOW_H
