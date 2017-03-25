#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ros/ros.h"

#include <sensor_msgs/JointState.h>
#include <sweetie_bot_text_msgs/TextCommand.h>

#include <ros/package.h>

#include <QWidget>
#include <QTimer>

class MainWindow : public QWidget {
    Q_OBJECT

private:
    bool m_isLeftEye;

    QPointF m_c;         	//Center of eye
    float m_R;           	//Radius1 of eye
    float m_R2;          	//Radius2 of eye (it is elliptic)
    float m_relR8;       	//Pupil radius
    float m_pupilRotation;   //Pupil rotation
    float m_rot;         	//Rotation of eye
    float m_scale;       	//Radius1/Radius2

    //octagon points
    QVector<QPointF>m_Pin;
    QVector<QPointF>m_Pout;

    bool m_gradientsOn;
    QVector<QLinearGradient> m_eyeGradients;
    QVector<QPainterPath> m_eyeGradientPaths;
    float m_lineWidth;
    float m_gradientWidth;

    //blinking
    float m_blinkHeight;
    int m_blinkLength;
    int m_blinkingTime;
    int m_currentBlinkingTime;
    bool m_isBlinking;
    bool m_isGoingDown;
    bool m_isMoveWithBlink;

    //delete in next versions
    int m_msBetweenMovement;
    QTimer* m_randomMoveTimer;

    QColor m_eyeColor;
    QColor m_pupilColor;
    QColor m_shinesColor;
    QColor m_outAreaColor;
    QColor m_eyelidColor;
    QColor m_eyelidOutlineColor;

    //shines
    QVector<QImage> m_shinesImages;
    QVector<QPointF> m_imagePositions;
    QVector<QPainterPath> m_shinesPaths;
    QPointF m_shinesOffset;
    float m_shinesScale;

    //eyelidss
    bool m_topEyelidVisible;
    bool m_bottomEyelidVisible;

    float m_topEyelidRotation;
    float m_oldTopEyelidRotation;
    float m_bottomEyelidRotation;
    float m_oldBottomEyelidRotation;

    float m_topEyelidY;
    float m_oldTopEyelidY;
    float m_bottomEyelidY;
    float m_oldBottomEyelidY;

    float m_topEyelidBend;
    float m_oldTopEyelidBend;
    float m_bottomEyelidBend;
    float m_oldBottomEyelidBend;

    QPainterPath m_topEyelidPath;
    QPainterPath m_bottomEyelidPath;
    QTransform m_topEyelidTransform;
    QTransform m_bottomEyelidTransform;

    QTimer* m_blinkTimer;
    QTimer* m_moveTimer;

    enum EyePath {
        GreenEllipse,
        OctagonAndLines,
        OutArea,

        EyePathCount
    };
    QVector<QPainterPath> m_eyePaths;
    QTransform m_eyeTransform;
    QTransform m_eyeScaleTransform;

    enum MoveFlags {
        EyePosition = 1,
        EyeRotation = 2,
        EyeSize = 4,
        EyeColor = 8,
        PupilSize = 16,
        PupilRotation = 32,
        TopEyelidHeight = 64,
        TopEyelidRotation = 128
    };

    QTransform m_transform45;


    //movement
    int m_currentMovingTime;   //how long is eye moving now
    int m_movingTime;      	//how long the eye should travel
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
    float m_stepTopEyelidY;

    float m_endBottomEyelidY;
    float m_stepBottomEyelidY;

    float m_endTopEyelidBend;
    float m_stepTopEyelidBend;

    float m_endBottomEyelidBend;
    float m_stepBottomEyelidBend;

    float m_endTopEyelidRotation;
    float m_stepTopEyelidRotation;

    float m_endBottomEyelidRotation;
    float m_stepBottomEyelidRotation;

	QString path_;
	QImage *overlay_;

	// ROS
	ros::NodeHandle node_;
	ros::Subscriber sub_joint_state_;
	ros::Subscriber sub_blink_;
	ros::Subscriber sub_color_;

	void controlCallback(const sweetie_bot_text_msgs::TextCommand::ConstPtr& msg);
	void moveCallback(const sensor_msgs::JointState::ConstPtr& msg);

public:
    MainWindow(bool isLeftEye = true, QWidget *parent = 0);
    ~MainWindow();

    QPointF rotatePoint(const QPointF& point, const QPointF& center, float angle);
    float distance(const QPointF& p1, const QPointF& p2);

    void countFrame();

    void countTransform45();

    void countEye();
    void countEyeGradients();
    void countEyeTransform();
    void countEyeScaleTransform();

    void countEyelids();
    void countTopEyelid();
    void countBottomEyelid();
    void countEyelidTransforms();
    void countTopEyelidTransform();
    void countBottomEyelidTransform();

    void countShines();
    void addShineImage(int dx, int dy, int r1, int r2, float angle);
    void addShinePath(int dx, int dy, int r1, int r2, int angle);
    //	void countShineGradient(float power);

    void move(MoveFlags flags, int ms,
              float eyeToX, float eyeToY, float eyeRotation,
              float eyeRadius, float eyeRadiusScale,
              int eyeColorR, int eyeColorG, int eyeColorB,
              float pupilRelativeSize,
              float pupilRotation,
              float topEyelidY,
              float topEyelidRotation);
    void setEndPositions();
    void blink(int ms);

    void drawFrame(float eyeX, float eyeY,
                   float eyeRotation,
                   float eyeRadius, float eyeRadiusScale,
                   int eyeColorR, int eyeColorG, int eyeColorB,
                   float pupilRelativeSize, float pupilRotation,
                   bool topEyelidVisible, bool bottomEyelidVisible,
                   float topEyelidY, float bottomEyelidY,
                   float topEyelidBend, float bottomEyelidBend,
                   float topEyelidRotation, float bottomEyelidRotation,
                   int eyelidColorR, int eyelidColorG, int eyelidColorB,
                   int eyelidOutlineColorR, int eyelidOutlineColorG, int eyelidOutlineColorB,
                   float shinesScale, float shinesOffsetX, float shinesOffsetY,
                   int shinesColorR, int shinesColorG, int shinesColorB,
                   int outAreaColorR, int outAreaColorG, int outAreaColorB);

    void paintEvent(QPaintEvent *);
    void keyPressEvent(QKeyEvent *e);
    void mousePressEvent(QMouseEvent* e);

    float toRad(float deg);
    float toDeg(float rad);

private slots:
    void updateMovingState();
    void updateBlinkState();
	void rosSpin();

    //delete in next versions
    void countMove();
};

#endif // MAINWINDOW_H
