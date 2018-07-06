#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QPainter>
#include <QTimer>

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sweetie_bot_text_msgs/TextCommand.h>

class MainWindow : public QWidget {
    Q_OBJECT

private:
    bool m_isLeftEye;
    bool m_publishPixmap;

    QPointF m_c;     //Center of eye
    float m_R;       //Radius1 of eye
    float m_R2;      //Radius2 of eye (it is elliptic)
    float m_relR8;   //Pupil radius
    float m_alpha;   //Pupil rotation
    float m_rot;     //Rotation of eye
    float m_scale;   //Radius1/Radius2

    //octagon points
    QVector<QPointF>m_Pin;
    QVector<QPointF>m_Pout;

    //blinking
    int m_blinkLength;
    int m_blinkingTime;
    int m_currentBlinkingTime;
    bool m_isBlinking;
    bool m_isGoingDown;
    bool m_isMoveWithBlink;

    //delete in next versions
    int m_msBetweenMovement;
    QTimer* m_randomMoveTimer;

    //eyelid
    float m_topEyelidRotation;
    float m_topEyelidY;
    float m_oldTopEyelidY;          //for blinking
    QPainterPath m_topEyelidPath;
    QPainterPath m_bottomEyelidPath; //for blinking
    QTransform m_eyelidTransform;

    QTimer* m_blinkTimer;
    QTimer* m_moveTimer;

    enum EyePath {
        GreenEllipse,
        BlackOctagonAndLines,
        WhiteArea,

        EyePathCount
    };
    QVector<QPainterPath> m_eyePaths;
    QTransform m_eyeTransform;

    QVector<QPainterPath> m_shinesPaths;

    enum MoveFlags {
        EyePosition = 1,
        EyeRotation = 2,
        EyeSize = 4,
        EyeColor = 8,
        PupilSize = 16,
        PupilRotation = 32,
        EyelidHeight = 64,
        EyelidRotation = 128
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

    QColor m_eyeColor;
    QColor m_eyelidColor;
    QColor m_eyelidOutlineColor;

    float m_endPupilRelativeSize;
    float m_stepPupilRelativeSize;

    float m_endPupilRotation;
    float m_stepPupilRotation;

    float m_endTopEyelidHeight;
    float m_stepTopEyelidHeight;

    float m_endEyelidRotation;
    float m_stepEyelidRotation;

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

    void countFrame();
    void countEye();
    void countEyeTransform();
    void countEyelidTransform();
    void countShines();
    void countEyelid();
    QPainterPath countShinePath(int dx, int dy, int r1, int r2, int angle);

    void move(MoveFlags flags, int ms,
              float eyeToX, float eyeToY, float eyeRotation,
              float eyeRadius, float eyeRadiusScale,
              int eyeColorR, int eyeColorG, int eyeColorB,
              float pupilRelativeSize,
              float pupilRotation,
              float eyelidHeight,
              float eyelidRotation);
    void setEndPositions();
    void blink(int ms);

    void paintEvent(QPaintEvent *);
    void keyPressEvent(QKeyEvent *e);


private slots:
    void updateMovingState();
    void updateBlinkState();

    //delete in next versions
    void countMove();

    // ROS
    void rosSpin();
};

#endif // MAINWINDOW_H
