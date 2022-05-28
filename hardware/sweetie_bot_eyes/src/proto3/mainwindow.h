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


#include "consts.h"

struct EyeState {
    QPointF center = QPointF(WIDTH/2.,HEIGHT/2. + 10); // +10 for new screen center correction

    // Eye configuraton
    float radius       = 287.5;       // Eye's first radius
    float radius2      = radius * radiusRatio;  // Eye's second radius (it is elliptic)
    float pupilRadius  = 0.6;         // Pupil contraction radius
    float pupilAngle   = 0;           // Pupil rotation
    float angle        = 40.83*0.9;   // Eye rotation angle
    float radiusRatio  = 0.8;         // Radius1/Radius2

    // Eyelids
    float topEyelidAngle       = -5;
    float topEyelidY           = 135;
    float bottomEyelidAngle    = -5;
    float bottomEyelidY        = 725;

    // Colors
    QColor eyeColor            = Qt::green;
    QColor eyelidColor         = QColor(143, 210, 143);
    QColor eyelidOutlineColor  = QColor(116, 169, 116);
    QColor whiteAreaColor      = Qt::white;

    inline void resetConfiguration() {
        radiusRatio       = 0.8;
        radius            = 287.5;
        radius2           = radius * radiusRatio;
        pupilRadius       = 0.6;
        pupilAngle        = 0;
        angle             = 40.83*0.9;

        topEyelidAngle    = -5;
        topEyelidY        = 135;
        bottomEyelidAngle = -5;
        bottomEyelidY     = 725;
    }

    inline void resetColors() {
        eyeColor           = Qt::green;
        eyelidColor        = QColor(143, 210, 143);
        eyelidOutlineColor = QColor(116, 169, 116);
        whiteAreaColor     = Qt::white;
    }
};


class MainWindow : public QOpenGLWidget {
    Q_OBJECT

private:
    bool m_isLeftEye;
    bool m_publishPixmap;

    QOpenGLFramebufferObject *m_fbo;

    EyeState m_state;

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

    // For blinking
    float m_startBottomEyelidY;
    float m_startBottomEyelidRotation;
    float m_startTopEyelidY;
    float m_startTopEyelidRotation;
    float m_startApertureContraction;

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

    enum MoveFlags {
        EyePosition           =  0x001,
        EyeRotation           =  0x002,
        EyeSize               =  0x004,
        EyeColor              =  0x008,
        PupilSize             =  0x010,
        PupilRotation         =  0x020,
        TopEyelidHeight       =  0x040,
        TopEyelidRotation     =  0x080,
        BottomEyelidHeight    =  0x100,
        BottomEyelidRotation  =  0x200,
    };


    //movement
    int m_currentMovingTime;   //how long is eye moving now
    int m_movingTime;          //how long the eye should travel
    int m_msUpdateMove;

    MoveFlags m_moveFlags;

    QPointF m_endEyePosition;  //where eye is moving to
    QPointF m_stepEyePosition; //step

    float m_endEyeRotation;
    float m_stepEyeRotation;

    float m_endEyeRadius;
    float m_stepEyeRadius;

    float m_endEyeRadiusScale;
    float m_stepEyeRadiusScale;

    float m_endPupilRelativeSize;
    float m_stepPupilRelativeSize;

    float m_endPupilRotation;
    float m_stepPupilRotation;

    float m_endTopEyelidY;
    float m_stepTopEyelidHeight;

    float m_endBottomEyelidY;
    float m_stepBottomEyelidHeight;

    float m_endTopEyelidRotation;
    float m_stepTopEyelidRotation;

    float m_endBottomEyelidRotation;
    float m_stepBottomEyelidRotation;

    float m_endApertureContraction;

    QString path_;
    QImage *overlay_;
    bool m_debug_mode_enabled = false;

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

    void move(MoveFlags flags, int ms,
              float eyeToX, float eyeToY, float eyeRotation,
              float eyeRadius, float eyeRadiusScale,
              int eyeColorR, int eyeColorG, int eyeColorB,
              float pupilRelativeSize,
              float pupilRotation,
              float topEyelidHeight,
              float topEyelidRotation,
              float bottomEyelidHeight,
              float bottomEyelidRotation);
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
