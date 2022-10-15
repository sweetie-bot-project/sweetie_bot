#ifndef EYEWINDOW_H
#define EYEWINDOW_H

#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <QOpenGLWidget>
#include <QOpenGLFramebufferObject>
#include <QOpenGLPaintDevice>

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>

#include "eye_state.h"

class EyeWindow : public QOpenGLWidget {
    Q_OBJECT

private:
    bool m_isLeftEye;
    bool m_publishPixmap;
    bool m_debug_mode_enabled = false;
    bool m_mouseEnabled;

    QOpenGLFramebufferObject *m_fbo;

    EyeState m_state;

    EyeAnimation  m_tempAnimation;
    EyeAnimation *m_playingAnimation;
    int m_currentAnimationStateId;

    EyeState m_lastAnimationState;

    EyeState m_startBlinkAnimationState;
    EyeState m_endBlinkAnimationState;

    //octagon points
    QVector<QPointF>m_Pin;
    QVector<QPointF>m_Pout;

    //blinking
    int m_blinkDefaultDuration;
    int m_blinkDuration;
    int m_blinkPause;
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

    QTimer *m_blinkTimer;
    QTimer *m_moveTimer;

    bool m_isMoveDryRunning;
    bool m_isBlinkDryRunning;

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
    bool m_isMoving;

    QString path_;
    QImage *overlay_;

    // ROS
    ros::NodeHandle node_;
    ros::Publisher  pub_eye_image_;

public:
    EyeWindow(bool isLeftEye = true, QWidget *parent = 0);
    ~EyeWindow();

    void PublishImage();

    QPointF rotatePoint(QPointF point, QPointF center, float angle);

    void computeFrame();
    void computeEye();
    void computeEyeTransform();
    void computeEyelidTransform();
    void computeShines();
    void computeEyelid();
    QPainterPath computeShinePath(int dx, int dy, int r1, int r2, int angle);

    void connectMoveTimer(QTimer *timer);
    void connectBlinkTimer(QTimer *timer);

    void move(MoveFlags flags, int ms, EyeState targetState, bool moveWithBlink = false, bool dryRun = false);
    void move(MoveFlags flags, EyeAnimation *targetSequence, bool moveWithBlink = false, bool dryRun = false);
    void blink(int ms, bool dryRun = false, int ms_pause = 50);

    void keyPressEvent(QKeyEvent *e);
    void mouseMoveEvent(QMouseEvent *e);

    bool isMoving() const { return m_isMoving; }
    bool isBlinking() const { return m_isBlinking; }
    bool isMouseEnabled() const { return m_mouseEnabled; }

    EyeState & getState() { return m_state; }
    void resetState();

    void initializeGL();
    void paintGL();

private slots:
    void updateMovingState();
    void updateBlinkState();

    //delete in next versions
    void computeRandomMove();
};

#endif // EYEWINDOW_H
