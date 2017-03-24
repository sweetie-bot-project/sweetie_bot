#include "mainwindow.h"

#include "consts.h"

#include <qmath.h>
#include <QPainter>
#include <QKeyEvent>

#include <QDebug>


MainWindow::MainWindow(int argc, char *argv[], bool isLeftEye, QWidget *parent) : QWidget(parent),
    m_isLeftEye(isLeftEye),

    m_c(QPointF(160,120)),
    m_R(125.0),
    m_relR8(0.6),
    m_pupilRotation(0.0),
    m_rot(0.0),
    m_scale(0.8),

    m_gradientsOn(false),
    m_lineWidth(3.0),
    m_gradientWidth(15.0),

    m_blinkHeight(180.0),
    m_blinkLength(200),
    m_blinkingTime(0),
    m_currentBlinkingTime(0),
    m_isBlinking(false),
    m_isGoingDown(false),
    m_isMoveWithBlink(false),

    m_msBetweenMovement(3000),
    m_randomMoveTimer(new QTimer(this)),

    m_eyeColor(QColor(Qt::green)),
    m_pupilColor(QColor(Qt::black)),
    m_shinesColor(QColor(Qt::white)),
    m_outAreaColor(QColor(Qt::white)),
    m_eyelidColor(QColor(143,210,143)),
    m_eyelidOutlineColor(QColor(116,169,116)),

    m_shinesOffset(QPointF(0,0)),
    m_shinesScale(1.0),

    m_topEyelidVisible(true),
    m_bottomEyelidVisible(true),
    m_topEyelidRotation(10.0),
    m_oldTopEyelidRotation(0.0),
    m_bottomEyelidRotation(-10.0),
    m_oldBottomEyelidRotation(0.0),
    m_topEyelidY(60.0),
    m_oldTopEyelidY(0.0),
    m_bottomEyelidY(60.0),
    m_oldBottomEyelidY(0.0),
    m_topEyelidBend(-30.0),
    m_oldTopEyelidBend(0.0),
    m_bottomEyelidBend(-30.0),
    m_oldBottomEyelidBend(0.0),

    m_blinkTimer(new QTimer(this)),
    m_moveTimer(new QTimer(this)),

    m_currentMovingTime(0),
    m_movingTime(0),
    m_msUpdateMove(20),

    m_moveFlags((MoveFlags)0),

    m_endEyePosition(QPointF(0,0)),
    m_stepEyePosition(QPointF(0,0)),
    m_endEyeRotation(0.0),
    m_stepEyeRotation(0.0),
    m_endEyeRadius(0.0),
    m_stepEyeRadius(0.0),
    m_endEyeRadiusScale(0.0),
    m_stepEyeRadiusScale(0.0),
    m_endPupilRelativeSize(0.0),
    m_stepPupilRelativeSize(0.0),
    m_endPupilRotation(0.0),
    m_stepPupilRotation(0.0),
    m_endTopEyelidY(0.0),
    m_stepTopEyelidY(0.0),
    m_endBottomEyelidY(0.0),
    m_stepBottomEyelidY(0.0),
    m_endTopEyelidBend(0.0),
    m_stepTopEyelidBend(0.0),
    m_endBottomEyelidBend(0.0),
    m_stepBottomEyelidBend(0.0),
    m_endTopEyelidRotation(0.0),
    m_stepTopEyelidRotation(0.0),
    m_endBottomEyelidRotation(0.0),
    m_stepBottomEyelidRotation(0.0) {

    setWindowFlags(Qt::FramelessWindowHint);

    m_Pin.fill(QPointF(), SIDES + 1);
    m_Pout.fill(QPointF(), SIDES);
    m_eyeGradients.fill(QLinearGradient(), SIDES);
    m_eyeGradientPaths.fill(QPainterPath(), SIDES);
    m_eyePaths.fill(QPainterPath(), EyePathCount);

    setFixedSize(WIDTH, HEIGHT);

    countTransform45();
    countEyeTransform();
    countEyeScaleTransform();
    countEyelidTransforms();
    countFrame();

    m_blinkTimer->setInterval(m_msUpdateMove);
    m_moveTimer->setInterval(m_msUpdateMove);
    connect(m_moveTimer, SIGNAL(timeout()), this, SLOT(updateMovingState()));
    connect(m_blinkTimer, SIGNAL(timeout()), this, SLOT(updateBlinkState()));
    connect(m_randomMoveTimer, SIGNAL(timeout()), this, SLOT(countMove()));

    setGeometry(400,400,320,240);

    // ROS
    ros::init(argc, argv, "eye");
    node = new ros::NodeHandle();
	path = QString::fromStdString( ros::package::getPath("sweetie_bot_eyes") );

    sub = node->subscribe<sensor_msgs::JointState>("/sweetie_bot/joint_states", 1000, &MainWindow::controlCallback, this);

    if(m_isLeftEye) {
        overlay = new QImage(path + "/overlays/leftEyeOverlay.png");
    }
    else {
        overlay = new QImage(path + "/overlays/rightEyeOverlay.png");
    }

    QTimer *timer = new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(rosSpin()));
    timer->start(50);
}

MainWindow::~MainWindow() {
}

void MainWindow::controlCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  double x = 0, y = 0;

  auto pos = std::find(msg->name.begin(), msg->name.end(), "joint55");
  if(pos != msg->name.end()) {
	int n = std::distance(msg->name.begin(), pos);
    if(msg->position.size() > n)
		y = msg->position[n];
  }

  pos = std::find(msg->name.begin(), msg->name.end(), "joint56");
  if(pos != msg->name.end()) {
	int n = std::distance(msg->name.begin(), pos);
    if(msg->position.size() > n)
		x = msg->position[n];
  }
 
  float eyeToX = 160 - (160 * x) / 2;
  float eyeToY = 120 - (120 * y) / 2;

  //ROS_INFO_STREAM(m_isLeftEye << "eye: ("<< x << "," << y << ") " << eyeToX << " " << eyeToY);

  m_isMoveWithBlink = false;

  move((MoveFlags)1, 30,
         eyeToX, eyeToY, 0, 0, 0,
         0, 0, 0,
         0, 0,
         0, 0);

}

void MainWindow::rosSpin()
{
    ros::spinOnce();
}

QPointF MainWindow::rotatePoint(const QPointF& point, const QPointF& center, float angle) {
    float x = point.x() - center.x();
    float y = point.y() - center.y();
    float newx = x * cos(angle) - y * sin(angle);
    float newy = x * sin(angle) + y * cos(angle);
    newx = newx + center.x();
    newy = newy + center.y();
    return QPointF(newx,newy);
}

float MainWindow::distance(const QPointF& p1, const QPointF& p2) {
    return qSqrt((p1.x() - p2.x()) * (p1.x() - p2.x()) +
                 (p1.y() - p2.y()) * (p1.y() - p2.y()));
}

void MainWindow::paintEvent(QPaintEvent *) {
    QPainter painter(this);
    painter.setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);

    painter.setPen(QPen(m_eyeColor));
    painter.setBrush(m_eyeColor);
    painter.drawPath(m_eyePaths[GreenEllipse]);

    if(m_gradientsOn) {
        painter.setPen(Qt::transparent);
        for(int i = 0; i < m_eyeGradientPaths.count(); i++) {
            painter.setBrush(m_eyeGradients[i]);
            painter.drawPath(m_eyeGradientPaths[i]);
        }
    }
    else {
        painter.setPen(QPen(m_pupilColor, m_lineWidth));
    }

    painter.setBrush(QColor(m_pupilColor));
    painter.drawPath(m_eyePaths[OctagonAndLines]);

    painter.setPen(Qt::transparent);
    painter.setBrush(m_outAreaColor);
    painter.drawPath(m_eyePaths[OutArea]);

    if(m_gradientsOn) {
        painter.setTransform(m_eyeTransform);
        painter.setTransform(m_eyeScaleTransform, true);
        for(int i = 0; i < m_shinesImages.size(); i++) {
            painter.drawImage(m_imagePositions[i], m_shinesImages[i]);
        }
        painter.resetTransform();
    }
    else {
        painter.setBrush(m_shinesColor);
        for(int i = 0; i < m_shinesPaths.count(); i++) {
            painter.drawPath(m_shinesPaths.at(i));
        }
    }

    painter.setPen(QPen(m_eyelidOutlineColor, 2));
    painter.setBrush(m_eyelidColor);

    if(m_topEyelidVisible) {
        painter.drawPath(m_topEyelidPath);
    }

    if(m_bottomEyelidVisible) {
        painter.drawPath(m_bottomEyelidPath);
    }

    painter.drawImage(0, 0, *overlay);
}

void MainWindow::keyPressEvent(QKeyEvent *e) {
    if(e->key() == Qt::Key_Escape) {
        close();
    }
    else if(e->key() == Qt::Key_D) {
        m_c.setX(m_c.x() + 1);
        countTransform45();
        countEyeScaleTransform();
        countEyeTransform();
        countFrame();
    }
    else if(e->key() == Qt::Key_A) {
        m_c.setX(m_c.x() - 1);
        countTransform45();
        countEyeScaleTransform();
        countEyeTransform();
        countFrame();
    }
    else if(e->key() == Qt::Key_S) {
        m_c.setY(m_c.y() + 1);
        countTransform45();
        countEyeScaleTransform();
        countEyeTransform();
        countFrame();
    }
    else if(e->key() == Qt::Key_W) {
        m_c.setY(m_c.y() - 1);
        countTransform45();
        countEyeScaleTransform();
        countEyeTransform();
        countFrame();
    }
    else if(e->key() == Qt::Key_Q) {
        m_rot -= 1;
        if(m_rot == -1)
            m_rot = 359;
        countEyeTransform();
        countFrame();
    }
    else if(e->key() == Qt::Key_E) {
        m_rot += 1;
        if(m_rot == 360)
            m_rot = 0;
        countEyeTransform();
        countFrame();
    }
    else if(e->key() == Qt::Key_T) {
        m_pupilRotation += 1;
        if(m_pupilRotation == 360)
            m_pupilRotation = 0;
        countFrame();
    }
    else if(e->key() == Qt::Key_R) {
        m_pupilRotation -= 1;
        if(m_pupilRotation == -1)
            m_pupilRotation = 359;
        countFrame();
    }
    else if(e->key() == Qt::Key_F) {
        if(m_scale > 0) {
            m_scale -= 0.01;
            countEyeScaleTransform();
            countFrame();
        }
    }
    else if(e->key() == Qt::Key_G) {
        if(m_scale < 1) {
            m_scale += 0.01;
            countEyeScaleTransform();
            countFrame();
        }
    }
    else if(e->key() == Qt::Key_Z) {
        if(m_R > 1) {
            m_R -= 1;
            countFrame();
        }
    }
    else if(e->key() == Qt::Key_X) {
        m_R += 1;
        countFrame();
    }
    else if(e->key() == Qt::Key_C) {
        if(m_relR8 > 0) {
            m_relR8 -= 0.01;
            countFrame();
        }
    }
    else if(e->key() == Qt::Key_V) {
        if(m_relR8 < 1) {
            m_relR8 += 0.01;
            countFrame();
        }
    }
    else if(e->key() == Qt::Key_B) {
        blink(m_blinkLength);
    }
    else if(e->key() == Qt::Key_Y) {
        if(m_msBetweenMovement >= 1000) {
            m_msBetweenMovement -= 500;
            m_randomMoveTimer->setInterval(m_msBetweenMovement);
        }
    }
    else if(e->key() == Qt::Key_U) {
        m_msBetweenMovement += 500;
    }
    else if(e->key() == Qt::Key_H) {
        if(m_topEyelidRotation > -30) {
            m_topEyelidRotation--;
            countTopEyelidTransform();
            countFrame();
        }
    }
    else if(e->key() == Qt::Key_J) {
        if(m_topEyelidRotation < 30) {
            m_topEyelidRotation++;
            countTopEyelidTransform();
            countFrame();
        }
    }
    else if(e->key() == Qt::Key_N) {
        if(m_topEyelidY > 0) {
            m_topEyelidY--;
            countFrame();
        }
    }
    else if(e->key() == Qt::Key_M) {
        if(m_topEyelidY < 120) {
            m_topEyelidY++;
            countFrame();
        }
    }
    else if(e->key() == Qt::Key_P) {
        if(!m_randomMoveTimer->isActive()) {
            m_randomMoveTimer->start();
            countMove();
        }
        else {
            m_randomMoveTimer->stop();
            m_isMoveWithBlink = false;
        }
    }
    else if(e->key() == Qt::Key_K) {
        m_gradientsOn = !m_gradientsOn;
        countFrame();
    }
}

void MainWindow::mousePressEvent(QMouseEvent* e) {
    qDebug()<<e->pos();
}

void MainWindow::countMove() {
    int ms           	=  qrand() % 301 + 100;      	//from 100 to 400
    float eyeToX     	=  qrand() % 161 + 80;       	//from 80  to 240
    float eyeToY     	=  qrand() % 81  + 100;      	//from 100 to 180
    float eyeRotation	=  qrand() % 41  - 20;       	//from -20 to 20
    float eyeRadius  	=  qrand() % 51  + 100;      	//from 100 to 150
    float eyeScale   	= (qrand() % 51  + 50) / 100.0;  //from 0.5 to 1
    int eyeColorR    	=  qrand() % 256;            	//from 0   to 255
    int eyeColorG    	=  qrand() % 256;            	//from 0   to 255
    int eyeColorB    	=  qrand() % 256;            	//from 0   to 255
    float pupilRelSize   = (qrand() % 61  + 20) / 100.0;  //from 0.2 to 0.8
    float pupilRotation  =  qrand() % 360;            	//from 0   to 359
    float eyelidHeight   =  qrand() % 101;            	//from 0   to 100
    float eyelidRotation =  qrand() % 61  - 30;       	//from -30 to 30

    m_isMoveWithBlink = qrand()%2;

    move((MoveFlags)(1), ms,
         eyeToX, eyeToY, eyeRotation, eyeRadius, eyeScale,
         eyeColorR, eyeColorG, eyeColorB,
         pupilRelSize, pupilRotation,
         eyelidHeight, eyelidRotation);

    m_randomMoveTimer->setInterval(m_msBetweenMovement);
}

void MainWindow::countShines() {
    m_shinesImages.clear();
    m_imagePositions.clear();
    m_shinesPaths.clear();

    if(m_gradientsOn) {
        addShineImage(-40, -30, 60, 30, 105);
        addShineImage(-12, 25, 15, 7, 120);
    }
    else {
        addShinePath(-40, -30, 60, 30, 105);
        addShinePath(-12, 25, 15, 7, 120);
    }
}

void MainWindow::countEyelids() {
    countTopEyelid();
    countBottomEyelid();
}

void MainWindow::countTopEyelid() {
    m_topEyelidPath = QPainterPath();
    m_topEyelidPath.moveTo(xLeft, yUp);
    m_topEyelidPath.lineTo(xLeft, yCenter - m_topEyelidY);
    m_topEyelidPath.quadTo(xCenter, yCenter - m_topEyelidY + 2 * m_topEyelidBend,
                           xRight, yCenter - m_topEyelidY);
    m_topEyelidPath.lineTo(xRight, yUp);
    m_topEyelidPath = m_topEyelidTransform.map(m_topEyelidPath);
}

void MainWindow::countBottomEyelid() {
    m_bottomEyelidPath = QPainterPath();
    m_bottomEyelidPath.moveTo(xLeft, yDown);
    m_bottomEyelidPath.lineTo(xLeft, yCenter + m_bottomEyelidY);
    m_bottomEyelidPath.quadTo(xCenter, yCenter + m_bottomEyelidY - 2 * m_bottomEyelidBend,
                              xRight, yCenter + m_bottomEyelidY);
    m_bottomEyelidPath.lineTo(xRight, yDown);
    m_bottomEyelidPath = m_bottomEyelidTransform.map(m_bottomEyelidPath);
}

void MainWindow::addShineImage(int dx, int dy, int r1, int r2, float angle) {
    float r100 = m_R/100;

    float scaledR1 = r1 * m_shinesScale * r100;
    float scaledR2 = r2 * m_shinesScale * r100;
    QPointF center(scaledR1, scaledR1);

    QImage img(scaledR1 * 2, scaledR1 * 2, QImage::Format_ARGB32);
    img.fill(Qt::transparent);

    QPainter pixmapPainter(&img);
    pixmapPainter.setRenderHint(QPainter::Antialiasing);
    pixmapPainter.setPen(Qt::transparent);

    QRadialGradient shineGradient(center, scaledR1);
    QColor tempColor = m_shinesColor;
    for(float x = 0; x < 1; x += 1.0 / SHINE_GRADIENT_SAMPLES) {
        float alpha = 255 * (-x*x + 1);
        tempColor.setAlpha(alpha);
        shineGradient.setColorAt(x, tempColor);
    }

    shineGradient.setColorAt(1.0, Qt::transparent);
    pixmapPainter.setBrush(shineGradient);
    pixmapPainter.drawEllipse(center, scaledR1, scaledR1);

    float radAngle = toRad(angle);
    float sinAngleAbs = qAbs(qSin(radAngle));
    float cosAngleAbs = qAbs(qCos(radAngle));
    float transformedWidth2  = scaledR1 * cosAngleAbs + scaledR2 * sinAngleAbs;
    float transformedHeight2 = scaledR1 * sinAngleAbs + scaledR2 * cosAngleAbs;

    QPointF position(m_c.x() - transformedWidth2 + (dx + m_shinesOffset.x()) * r100,
                     m_c.y() - transformedHeight2 + (dy + m_shinesOffset.y()) * r100);
    m_imagePositions.append(position);

    QTransform t;
    t.rotate(angle);
    t.scale(1, scaledR2/scaledR1);

    m_shinesImages.append(img.transformed(t, Qt::SmoothTransformation));
}

void MainWindow::addShinePath(int dx, int dy, int r1, int r2, int angle) {
    QPainterPath path;

    float r100 = m_R/100;
    QPointF center(m_c.x() + (dx + m_shinesOffset.x()) * r100,
                   m_c.y() + (dy + m_shinesOffset.y()) * r100);

    QTransform t = m_eyeScaleTransform;
    t.translate(center.x(), center.y());
    t.rotate(angle);
    t.translate(-center.x(), -center.y());

    path.addEllipse(center, r1 * r100 * m_shinesScale, r2 * r100  * m_shinesScale);
    path = t.map(path);

    m_shinesPaths.append(m_eyeTransform.map(path));
}

void MainWindow::move(MoveFlags flags, int ms,
                      float eyeToX, float eyeToY, float eyeRotation,
                      float eyeRadius, float eyeRadiusScale,
                      int eyeColorR, int eyeColorG, int eyeColorB,
                      float pupilRelativeSize,
                      float pupilRotation,
                      float topEyelidY,
                      float topEyelidRotation) {
    m_currentMovingTime = 0;
    m_movingTime = ms;
    m_moveFlags = flags;

    float frac = m_msUpdateMove/(float)m_movingTime;

    if(m_moveFlags & EyePosition) {
        m_endEyePosition = QPointF(eyeToX, eyeToY);
        m_stepEyePosition = (m_endEyePosition - m_c) * frac;
    }
    if(m_moveFlags & EyeRotation && !m_isMoveWithBlink) {
        m_endEyeRotation = eyeRotation;
        m_stepEyeRotation = (m_endEyeRotation - m_rot) * frac;
    }
    if(m_moveFlags & EyeSize) {
        m_endEyeRadius = eyeRadius;
        m_stepEyeRadius = (m_endEyeRadius - m_R) * frac;

        m_endEyeRadiusScale = eyeRadiusScale;
        m_stepEyeRadiusScale = (m_endEyeRadiusScale - m_scale) * frac;
    }
    if(m_moveFlags & EyeColor) {
        m_eyeColor = QColor(eyeColorR, eyeColorG, eyeColorB);
    }
    if(m_moveFlags & PupilSize) {
        m_endPupilRelativeSize = pupilRelativeSize;
        m_stepPupilRelativeSize = (m_endPupilRelativeSize - m_relR8) * frac;
    }
    if(m_moveFlags & PupilRotation) {
        m_endPupilRotation = pupilRotation;
        m_stepPupilRotation = (m_endPupilRotation - m_pupilRotation) * frac;
    }
    if(m_moveFlags & TopEyelidHeight && !m_isBlinking) {
        m_endTopEyelidY= topEyelidY;
        m_stepTopEyelidY = (m_endTopEyelidY- m_topEyelidY) * frac;
    }
    if(m_moveFlags & TopEyelidRotation && !m_isMoveWithBlink) {
        m_endTopEyelidRotation = topEyelidRotation;
        m_stepTopEyelidRotation = (m_endTopEyelidRotation - m_topEyelidRotation) * frac;
    }

    if(!m_isMoveWithBlink) {
        m_moveTimer->start();
    }
    else {
        blink(m_blinkLength);
    }
}

void MainWindow::updateMovingState() {
    m_currentMovingTime += m_msUpdateMove;

    if(m_currentMovingTime > m_movingTime) {
        if(m_moveFlags & EyePosition) {
            m_c = m_endEyePosition;
        }
        if(m_moveFlags & EyeRotation) {
            m_rot = m_endEyeRotation;
        }
        if(m_moveFlags & EyeSize) {
            m_R = m_endEyeRadius;
            m_scale = m_endEyeRadiusScale;
        }
        if(m_moveFlags & PupilSize) {
            m_relR8 = m_endPupilRelativeSize;
        }
        if(m_moveFlags & PupilRotation) {
            m_pupilRotation = m_endPupilRotation;
        }
        if(m_moveFlags & TopEyelidHeight && !m_isBlinking) {
            m_topEyelidY = m_endTopEyelidY;
        }
        if(m_moveFlags & TopEyelidRotation) {
            m_topEyelidRotation = m_endTopEyelidRotation;
        }

        m_currentMovingTime = 0;
        m_moveTimer->stop();
    }
    else {
        if(m_moveFlags & EyePosition) {
            m_c += m_stepEyePosition;
        }
        if(m_moveFlags & EyeRotation) {
            m_rot += m_stepEyeRotation;
        }
        if(m_moveFlags & EyeSize) {
            m_R += m_stepEyeRadius;
            m_scale += m_stepEyeRadiusScale;
        }
        if(m_moveFlags & PupilSize) {
            m_relR8 += m_stepPupilRelativeSize;
        }
        if(m_moveFlags & PupilRotation) {
            m_pupilRotation += m_stepPupilRotation;
        }
        if(m_moveFlags & TopEyelidHeight && !m_isBlinking) {
            m_topEyelidY += m_stepTopEyelidY;
        }
        if(m_moveFlags & TopEyelidRotation) {
            m_topEyelidRotation += m_stepTopEyelidRotation;
        }
    }

    if(m_moveFlags & EyePosition) {
        countTransform45();
    }
    if(m_moveFlags & EyePosition || m_moveFlags & EyeRotation) {
        countEyeTransform();
    }
    if(m_moveFlags & EyePosition || m_moveFlags & EyeSize) {
        countEyeScaleTransform();
    }
    if(m_moveFlags & TopEyelidRotation) {
        countTopEyelidTransform();
    }

    countFrame();
}

void MainWindow::blink(int ms) {
    if(m_isBlinking) {
        return;
    }

    m_isBlinking = true;
    m_isGoingDown = true;
    m_blinkingTime = ms;

    m_oldTopEyelidY = m_topEyelidY;
    m_oldBottomEyelidY = m_bottomEyelidY;
    m_oldTopEyelidBend = m_topEyelidBend;
    m_oldBottomEyelidBend = m_bottomEyelidBend;
    m_oldTopEyelidRotation = m_topEyelidRotation;
    m_oldBottomEyelidRotation = m_bottomEyelidRotation;

    float frac = m_msUpdateMove/(float)m_blinkingTime;

    m_endTopEyelidY = yCenter - m_blinkHeight;
    m_endBottomEyelidY = m_blinkHeight - yCenter;
    m_stepTopEyelidY = (m_endTopEyelidY - m_topEyelidY) * frac;
    m_stepBottomEyelidY = (m_endBottomEyelidY - m_bottomEyelidY) * frac;

    m_endTopEyelidBend = 0.0;
    m_endBottomEyelidBend = 0.0;
    m_stepTopEyelidBend = (m_endTopEyelidBend - m_topEyelidBend) * frac;
    m_stepBottomEyelidBend = (m_endBottomEyelidBend - m_bottomEyelidBend) * frac;

    m_endTopEyelidRotation = 0.0;
    m_endBottomEyelidRotation = 0.0;
    m_stepTopEyelidRotation = (m_endTopEyelidRotation - m_topEyelidRotation) * frac;
    m_stepBottomEyelidRotation = (m_endBottomEyelidRotation - m_bottomEyelidRotation) * frac;

    m_blinkTimer->start();
}

void MainWindow::drawFrame(float eyeX, float eyeY,
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
                           int outAreaColorR, int outAreaColorG, int outAreaColorB) {
    m_c = QPointF(eyeX, eyeY);
    m_rot = eyeRotation;
    m_R = eyeRadius;
    m_scale = eyeRadiusScale;
    m_eyeColor = QColor(eyeColorR, eyeColorG, eyeColorB);
    m_relR8 = pupilRelativeSize;
    m_pupilRotation = pupilRotation;
    m_topEyelidVisible = topEyelidVisible;
    m_bottomEyelidVisible = bottomEyelidVisible;
    m_topEyelidY = topEyelidY;
    m_bottomEyelidY = bottomEyelidY;
    m_topEyelidBend = topEyelidBend;
    m_bottomEyelidBend = bottomEyelidBend;
    m_topEyelidRotation = topEyelidRotation;
    m_bottomEyelidRotation = bottomEyelidRotation;
    m_eyelidColor = QColor(eyelidColorR, eyelidColorG, eyelidColorB);
    m_eyelidOutlineColor = QColor(eyelidOutlineColorR, eyelidOutlineColorG, eyelidOutlineColorB);
    m_shinesScale = shinesScale;
    m_shinesOffset = QPointF(shinesOffsetX, shinesOffsetY);
    m_shinesColor = QColor(shinesColorR, shinesColorG, shinesColorB);
    m_outAreaColor = QColor(outAreaColorR, outAreaColorG, outAreaColorB);

    countEyeTransform();
    countEyelidTransforms();
    countFrame();
}

void MainWindow::updateBlinkState() {
    m_currentBlinkingTime += m_msUpdateMove;

    if(m_isGoingDown && m_currentBlinkingTime > m_blinkingTime) {
        m_isGoingDown = false;

        m_topEyelidY = m_endTopEyelidY;
        m_bottomEyelidY = m_endBottomEyelidY;
        m_topEyelidBend = m_endTopEyelidBend;
        m_bottomEyelidBend = m_endBottomEyelidBend;
        m_topEyelidRotation = m_endTopEyelidRotation;
        m_bottomEyelidRotation = m_endBottomEyelidRotation;

        float frac = m_msUpdateMove/(float)m_blinkingTime;
        m_endTopEyelidY = m_oldTopEyelidY;
        m_endBottomEyelidY = m_oldBottomEyelidY;
        m_endTopEyelidBend = m_oldTopEyelidBend;
        m_endBottomEyelidBend = m_oldBottomEyelidBend;
        m_endTopEyelidRotation = m_oldTopEyelidRotation;
        m_endBottomEyelidRotation = m_oldBottomEyelidRotation;

        m_stepTopEyelidY = (m_endTopEyelidY - m_topEyelidY) * frac;
        m_stepBottomEyelidY = (m_endBottomEyelidY - m_bottomEyelidY) * frac;
        m_stepTopEyelidBend = (m_endTopEyelidBend - m_topEyelidBend) * frac;
        m_stepBottomEyelidBend = (m_endBottomEyelidBend - m_bottomEyelidBend) * frac;
        m_stepTopEyelidRotation = (m_endTopEyelidRotation - m_topEyelidRotation) * frac;
        m_stepBottomEyelidRotation = (m_endBottomEyelidRotation - m_bottomEyelidRotation) * frac;

        if(m_isMoveWithBlink) {
            if(m_moveFlags & EyePosition) {
                m_c = m_endEyePosition;
            }
            if(m_moveFlags & EyeRotation) {
                m_rot += m_endEyeRotation;
            }
            if(m_moveFlags & EyeSize) {
                m_R = m_endEyeRadius;
                m_scale = m_endEyeRadiusScale;
            }
            if(m_moveFlags & PupilSize) {
                m_relR8 = m_endPupilRelativeSize;
            }
            if(m_moveFlags & PupilRotation) {
                m_pupilRotation = m_endPupilRotation;
            }

            if(m_moveFlags & EyePosition) {
                countTransform45();
            }
            if(m_moveFlags & EyePosition || m_moveFlags & EyeRotation) {
                countEyeTransform();
            }
            if(m_moveFlags & EyePosition || m_moveFlags & EyeSize) {
                countEyeScaleTransform();
            }

            countEye();
            countShines();
        }
    }
    else if(!m_isGoingDown && m_currentBlinkingTime > m_blinkingTime * 2) {
        m_topEyelidY = m_endTopEyelidY;
        m_bottomEyelidY = m_endBottomEyelidY;
        m_topEyelidBend = m_endTopEyelidBend;
        m_bottomEyelidBend = m_endBottomEyelidBend;
        m_topEyelidRotation = m_endTopEyelidRotation;
        m_bottomEyelidRotation = m_endBottomEyelidRotation;

        m_currentBlinkingTime = 0;
        m_isBlinking = false;
        m_blinkTimer->stop();
    }
    else {
        m_topEyelidY += m_stepTopEyelidY;
        m_bottomEyelidY += m_stepBottomEyelidY;
        m_topEyelidBend += m_stepTopEyelidBend;
        m_bottomEyelidBend += m_stepBottomEyelidBend;
        m_topEyelidRotation += m_stepTopEyelidRotation;
        m_bottomEyelidRotation += m_stepBottomEyelidRotation;
    }

    countEyelidTransforms();
    countEyelids();
    update();
}

void MainWindow::countFrame() {
    countEye();
    countShines();
    countEyelids();
    update();
}

void MainWindow::countTransform45() {
    m_transform45.reset();
    m_transform45.translate(m_c.x(), m_c.y());
    m_transform45.rotate(360 / SIDES);
    m_transform45.translate(-m_c.x(), -m_c.y());
}

void MainWindow::countEye() {
    float alpha = toRad(m_pupilRotation);
    float absR8 = m_R * m_relR8;
    QPointF V = QPointF(m_c.x(), m_c.y() - absR8);

    m_Pin[0] = V;
    for(int i = 1; i < SIDES; i++) {
        m_Pin[i] = m_transform45.map(m_Pin[i - 1]);
    }
    m_Pin[SIDES] = m_Pin[0];

    float beta = M_PI/SIDES;
    float gamma = qAsin(absR8*sin(5*M_PI/SIDES)/m_R);
    float delta = 3*M_PI/SIDES - gamma;
    float l = qSqrt(m_R*m_R + absR8*absR8 - 2*m_R*absR8*cos(delta));
    if(m_gradientsOn) {
        l += m_gradientWidth;
    }

    float dx = l*qCos(beta);
    float dy = l*qSin(beta);

    m_Pout[0] = QPointF(V.x() - dx, V.y() - dy);
    for(int i = 1; i < SIDES; i++) {
        m_Pout[i] = m_transform45.map(m_Pout[i - 1]);
    }

    if(m_pupilRotation != 0) {
        for(int i = 0; i < SIDES; i++) {
            m_Pin[i] = rotatePoint(m_Pin[i], m_c, alpha);
            m_Pout[i] = rotatePoint(m_Pout[i], m_c, alpha);
        }
        m_Pin[SIDES] = rotatePoint(m_Pin[SIDES], m_c, alpha);
    }

    m_R2 = m_R * m_scale;

    //countPath
    for(int i = 0; i < EyePathCount; i++) {
        m_eyePaths[i] = QPainterPath();
    }
    m_eyePaths[GreenEllipse].addEllipse(m_c, m_R2, m_R);

    if(m_gradientsOn) {
        countEyeGradients();
    }
    else {
        for(int i = 0; i < SIDES; i++) {
            m_eyePaths[OctagonAndLines].moveTo(m_Pin[i]);
            m_eyePaths[OctagonAndLines].lineTo(m_Pout[i]);
        }
    }

    m_eyePaths[OctagonAndLines].addPolygon(QPolygonF(m_Pin));
    m_eyePaths[OctagonAndLines] = m_eyeScaleTransform.map(m_eyePaths[OctagonAndLines]);

    m_eyePaths[OutArea].addEllipse(m_c, m_R2, m_R);
    if(m_rot != 0) {
        for(int i = 0; i < EyePathCount; i++) {
            m_eyePaths[i] = m_eyeTransform.map(m_eyePaths[i]);
        }
    }
    m_eyePaths[OutArea].addRect(0, 0, WIDTH, HEIGHT);
}

void MainWindow::countEyeGradients() {
    QPointF topRight[SIDES], topLeft[SIDES];
    QPointF bottomLeft[SIDES], bottomRight[SIDES];

    for(int i = 0; i < SIDES; i++) {
        m_eyeGradientPaths[i] = QPainterPath();

        topRight[i] = m_eyeScaleTransform.map(m_Pin[i]);
        topLeft[i] = m_eyeScaleTransform.map(m_Pout[i]);
        topRight[i] = m_eyeTransform.map(topRight[i]);
        topLeft[i] = m_eyeTransform.map(topLeft[i]);
    }

    float absDelta = m_gradientWidth * m_scale;
    for(int i = 0; i < SIDES; i++) {
        float delta;
        if(topRight[i].y() < topLeft[i].y()) {
            delta = - absDelta;
        }
        else {
            delta = absDelta;
        }

        float k = (topRight[i].x() - topLeft[i].x()) / (topRight[i].y() - topLeft[i].y());
        float atanK = qAtan(k);
        float dx = qCos(atanK) * delta;
        float dy = qSin(atanK) * delta;

        bottomLeft[i] = QPointF(topLeft[i].x() - dx, topLeft[i].y() + dy);
        bottomRight[i] = QPointF(topRight[i].x() - dx, topRight[i].y() + dy);

        m_eyeGradientPaths[i].moveTo(topRight[i]);
        m_eyeGradientPaths[i].lineTo(topLeft[i]);
        m_eyeGradientPaths[i].lineTo(bottomLeft[i]);
        m_eyeGradientPaths[i].lineTo(bottomRight[i]);

        m_eyeGradients[i].setStart(topLeft[i]);
        m_eyeGradients[i].setFinalStop(bottomLeft[i]);
        m_eyeGradients[i].setColorAt(0.0, m_pupilColor);
        m_eyeGradients[i].setColorAt(1.0, QColor(Qt::transparent));
    }
}

void MainWindow::countEyeTransform() {
    m_eyeTransform.reset();
    m_eyeTransform.translate(m_c.x(), m_c.y());
    m_eyeTransform.rotate(m_rot);
    m_eyeTransform.translate(-m_c.x(), -m_c.y());
}

void MainWindow::countEyeScaleTransform() {
    m_eyeScaleTransform.reset();
    m_eyeScaleTransform.translate(m_c.x(), m_c.y());
    m_eyeScaleTransform.scale(m_scale, 1.0);
    m_eyeScaleTransform.translate(-m_c.x(), -m_c.y());
}

void MainWindow::countEyelidTransforms() {
    countTopEyelidTransform();
    countBottomEyelidTransform();
}

void MainWindow::countTopEyelidTransform() {
    m_topEyelidTransform.reset();
    m_topEyelidTransform.translate(xCenter, yCenter);
    if(m_isLeftEye) {
        m_topEyelidTransform.rotate(m_topEyelidRotation);
    }
    else {
        m_topEyelidTransform.rotate(-m_topEyelidRotation);
    }
    m_topEyelidTransform.translate(-xCenter, -yCenter);
}

void MainWindow::countBottomEyelidTransform() {
    m_bottomEyelidTransform.reset();
    m_bottomEyelidTransform.translate(xCenter, yCenter);
    if(m_isLeftEye) {
        m_bottomEyelidTransform.rotate(m_bottomEyelidRotation);
    }
    else {
        m_bottomEyelidTransform.rotate(-m_bottomEyelidRotation);
    }
    m_bottomEyelidTransform.translate(-xCenter, -yCenter);
}


float MainWindow::toRad(float deg) {
    return deg * M_PI / 180.0;
}


float MainWindow::toDeg(float rad) {
    return rad * 180.0 / M_PI;
}
