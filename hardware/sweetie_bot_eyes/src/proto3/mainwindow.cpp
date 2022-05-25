#include "mainwindow.h"
#include "qmath.h"
#include <QPainter>
#include <QKeyEvent>
#include <QApplication>

#include "consts.h"

// Convertion proportions for new eye resolution
// 320x240 -> 800x800
// 2.5x3.3333333333333335

MainWindow::MainWindow(bool isLeftEye, QWidget *parent) : QOpenGLWidget(parent),
    m_isLeftEye(isLeftEye),
    m_publishPixmap(false),

    m_c(QPointF(WIDTH/2.,HEIGHT/2. + 10)), // +10 for new screen center correction
    m_R(115.0*2.5),
    m_relR8(0.6),
    m_alpha(0.0),
    m_rot(40.83*0.9),
    m_scale(0.8),

    m_blinkLength(150),
    m_blinkingTime(0),
    m_currentBlinkingTime(0),
    m_isBlinking(false),
    m_isGoingDown(false),
    m_isMoveWithBlink(false),

    m_msBetweenMovement(3000),
    m_randomMoveTimer(new QTimer(this)),

    m_topEyelidRotation(-5.0),
    m_topEyelidY(135.0),
    m_oldTopEyelidY(0.0),

    m_blinkTimer(new QTimer(this)),
    m_moveTimer(new QTimer(this)),

    m_currentMovingTime(0),
    m_movingTime(0),
    m_msUpdateMove(16),

    m_moveFlags((MoveFlags)0),

    m_endEyePosition(QPointF(0,0)),
    m_stepEyePosition(QPointF(0,0)),
    m_endEyeRotation(0.0),
    m_stepEyeRotation(0.0),
    m_endEyeRadius(0.0),
    m_stepEyeRadius(0.0),
    m_endEyeRadiusScale(0.0),
    m_stepEyeRadiusScale(0.0),
    m_eyeColor(QColor(0, 255, 0)),

    m_eyelidColor(QColor(143,210,143)),
    m_eyelidOutlineColor(QColor(116,169,116)),

    m_endPupilRelativeSize(0.0),
    m_stepPupilRelativeSize(0.0),
    m_endPupilRotation(0.0),
    m_stepPupilRotation(0.0),
    m_endTopEyelidHeight(0.0),
    m_stepTopEyelidHeight(0.0),
    m_endEyelidRotation(0.0),
    m_stepEyelidRotation(0.0)

    // ROS
    //node_(new ros::NodeHandle)
{
    setAutoFillBackground(false);

    QSurfaceFormat format;
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    format.setSwapInterval(10);
    format.setSamples(8);
    setFormat(format);

    setWindowFlags(Qt::FramelessWindowHint);

    if (!m_isLeftEye) {
        m_rot = -m_rot;
    }

    m_Pin.fill(QPointF(), SIDES + 1);
    m_Pout.fill(QPointF(), SIDES);
    m_eyePaths.fill(QPainterPath(), EyePathCount);

    setFixedSize(WIDTH, HEIGHT);

    countEyeTransform();
    countEyelidTransform();
    countFrame();

    // Should be set on the widget to accept keyPressEvent as a child
    // or there's no way to set focus on the individual eye
    setFocusPolicy(Qt::StrongFocus);

    m_blinkTimer->setInterval(m_msUpdateMove);
    m_moveTimer->setInterval(m_msUpdateMove);
    connect(m_moveTimer, SIGNAL(timeout()), this, SLOT(updateMovingState()));
    connect(m_blinkTimer, SIGNAL(timeout()), this, SLOT(updateBlinkState()));
    connect(m_randomMoveTimer, SIGNAL(timeout()), this, SLOT(countMove()));

    QTimer *timer = new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(rosSpin()));
    timer->start(10);

    // NODE INTERFACE 
    // parameteres
    ros::param::get("~publish_pixmap", m_publishPixmap);
    // subscribers
    sub_control_ = node_.subscribe<sweetie_bot_text_msgs::TextCommand>("control", 1, &MainWindow::controlCallback, this, ros::TransportHints().tcpNoDelay());
    sub_joint_state_ = node_.subscribe<sensor_msgs::JointState>("joint_states", 1, &MainWindow::moveCallback, this, ros::TransportHints().tcpNoDelay());
    // publishers
    std::string image_eye_topic_name = (m_isLeftEye) ? "eye_image_left" : "eye_image_right";
    pub_eye_image_ = node_.advertise<sensor_msgs::Image>(image_eye_topic_name, 1);

    path_ =  QString::fromStdString( ros::package::getPath("sweetie_bot_eyes") );

    // Set debug mode
    auto args = QApplication::arguments();
    m_debug_mode_enabled = args.contains("-debug");
    
    if (m_debug_mode_enabled) {
        if(m_isLeftEye) {
            overlay_ = new QImage(path_ + "/overlays/proto3_leftEyeOverlay.png");
        }
        else {
            overlay_ = new QImage(path_ + "/overlays/proto3_rightEyeOverlay.png");
        }
    }
}

MainWindow::~MainWindow() {
    delete m_fbo;
}

void MainWindow::rosSpin()
{
    if(!ros::ok()) QApplication::quit();
    ros::spinOnce();
}

void MainWindow::initializeGL() {
    QOpenGLFramebufferObjectFormat format;
    format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);

    m_fbo = new QOpenGLFramebufferObject(width(), height(), format);
}

void MainWindow::paintGL() {
    QOpenGLPaintDevice fboPaintDev(width(), height());
    QPainter painter(&fboPaintDev);
    painter.setRenderHints(QPainter::Antialiasing);

    painter.setPen(QPen(m_eyeColor));
    painter.setBrush(QColor(m_eyeColor));
    painter.drawPath(m_eyePaths[GreenEllipse]);

    painter.setPen(QPen(Qt::black, 3));
    painter.setBrush(QColor(Qt::black));
    painter.drawPath(m_eyePaths[BlackOctagonAndLines]);

    painter.setPen(QPen(Qt::white, 0));
    painter.setBrush(QColor(Qt::white));
    painter.drawPath(m_eyePaths[WhiteArea]);

    for(int i = 0; i < m_shinesPaths.size(); i++) {
        painter.drawPath(m_shinesPaths.at(i));
    }

    painter.setPen(QPen(m_eyelidOutlineColor, 2));
    painter.setBrush(m_eyelidColor);

    painter.drawPath(m_topEyelidPath);
    if(m_isBlinking) {
        painter.drawPath(m_bottomEyelidPath);
    }

    if (m_debug_mode_enabled) {
        painter.drawImage(0, 0, *overlay_);

        QString eye_info = QString("eye :: x,y: %1, %2; ang: %3; x_scale: %4; radius: %5").arg(QString::number(m_c.x()), QString::number(m_c.y()), QString::number(m_rot), QString::number(m_scale), QString::number(m_R));
        QString aperture_info = QString("aperture :: ang: %1; contraction: %3\n").arg(QString::number(m_alpha), QString::number(m_relR8));
        QString top_eyelid_info = QString("top_eyelid :: y: %1; ang: %3\n").arg(QString::number(m_topEyelidY), QString::number(m_topEyelidRotation));
        QString bottom_eyelid_info = QString("bottom_eyelid :: y: %1; ang: %3\n").arg(QString::number(m_topEyelidY), QString::number(m_topEyelidRotation));
    
        painter.drawText(10, 10, eye_info);
        painter.drawText(10, 20, aperture_info);
        painter.drawText(10, 30, top_eyelid_info);
        painter.drawText(10, 40, bottom_eyelid_info);
    }

    painter.end();
}

void MainWindow::PublishImage()
{
    makeCurrent();
    m_fbo->bind();

    paintGL();

    m_fbo->release();

    QImage fboImage(m_fbo->toImage().convertToFormat(QImage::Format_RGBA8888));
    QImage image(fboImage.constBits(), fboImage.width(), fboImage.height(), QImage::Format_RGBA8888);

    doneCurrent();

    sensor_msgs::Image img;
    img.header.stamp = ros::Time::now();
    img.width = image.width();
    img.height = image.height();
    img.encoding = "rgba8";
    img.step = image.bytesPerLine();
    //ROS_INFO("format=%d bytesPerLine=%d", int(image.format()), image.bytesPerLine());
    img.data = std::vector<unsigned char>(image.bits(), image.bits() + image.byteCount());
    pub_eye_image_.publish(img);
}

constexpr unsigned int str2hash(const char* str, int h = 0)
{
    return !str[h] ? 5381 : (str2hash(str, h+1)*33) ^ str[h];
}

void MainWindow::controlCallback(const sweetie_bot_text_msgs::TextCommand::ConstPtr& msg)
{
	//ROS_INFO_STREAM("\n" << *msg);
	switch(str2hash(msg->type.c_str()))
	{
		case str2hash("eyes/action"):
			switch(str2hash(msg->command.c_str())){
				case str2hash("blink"):
									//ROS_INFO("blink");
									blink(100);
									break;
				case str2hash("slow_blink"):
									//ROS_INFO("slow_blink");
									blink(1000);
									break;

			}				
			break;

		case str2hash("eyes/emotion"):
			switch(str2hash(msg->command.c_str())){
				case str2hash("normal"):
									//ROS_INFO("normal");
									m_topEyelidRotation = -5;
									m_topEyelidY = 135;

									m_eyeColor = QColor(Qt::green);
									m_eyelidColor = QColor(143,210,143);
									m_eyelidOutlineColor = QColor(116,169,116);
									break;
				case str2hash("red_eyes"):
									//ROS_INFO("red_eyes");
									m_eyeColor = QColor(Qt::red);
									m_eyelidColor = QColor(166,32,55);
									m_eyelidOutlineColor = QColor(0,0,0);
									break;
				case str2hash("sad_look"):
									//ROS_INFO("sad_look");
									m_topEyelidRotation = 15;
									m_topEyelidY = 150;
									break;
				case str2hash("evil_look"):
									//ROS_INFO("evil_look");
									m_topEyelidRotation = -30;
									m_topEyelidY = 200;
									break;
			}				
			break;

	}
	countEyelidTransform();
	if (m_publishPixmap) PublishImage();
}


void MainWindow::moveCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  double x = 0, y = 0;

  auto pos = std::find(msg->name.begin(), msg->name.end(), "eyes_pitch");
  if(pos != msg->name.end()) {
        int n = std::distance(msg->name.begin(), pos);
    if(msg->position.size() > n)
                y = msg->position[n];
  }

  pos = std::find(msg->name.begin(), msg->name.end(), "eyes_yaw");
  if(pos != msg->name.end()) {
        int n = std::distance(msg->name.begin(), pos);
    if(msg->position.size() > n)
                x = msg->position[n];
  }

  float eyeToX = WIDTH/2 - (WIDTH/2 * x) / 2;
  float eyeToY = HEIGHT/2 - (HEIGHT/2 * y) / 2;

  //ROS_INFO_STREAM(m_isLeftEye << "eye: ("<< x << "," << y << ") " << eyeToX << " " << eyeToY);

  m_isMoveWithBlink = false;

  move((MoveFlags)1, 30,
         eyeToX, eyeToY, 0, 0, 0,
         0, 0, 0,
         0, 0,
         0, 0);

  if (m_publishPixmap) PublishImage();
}

QPointF MainWindow::rotatePoint(QPointF point, QPointF center, float angle) {
    float x = point.x() - center.x();
    float y = point.y() - center.y();
    float newx = x * cos(angle) - y * sin(angle);
    float newy = x * sin(angle) + y * cos(angle);
    newx = newx + center.x();
    newy = newy + center.y();
    return QPointF(newx,newy);
}

void MainWindow::keyPressEvent(QKeyEvent *e) {
    if(e->key() == Qt::Key_Escape) {
        QApplication::quit();
    }
    else if(e->key() == Qt::Key_D) {
        m_c.setX(m_c.x() + 1);
        countFrame();
    }
    else if(e->key() == Qt::Key_A) {
        m_c.setX(m_c.x() - 1);
        countFrame();
    }
    else if(e->key() == Qt::Key_S) {
        m_c.setY(m_c.y() + 1);
        countFrame();
    }
    else if(e->key() == Qt::Key_W) {
        m_c.setY(m_c.y() - 1);
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
        m_alpha += 1;
        if(m_alpha == 360)
            m_alpha = 0;
        countFrame();
    }
    else if(e->key() == Qt::Key_R) {
        m_alpha -= 1;
        if(m_alpha == -1)
            m_alpha = 359;
        countFrame();
    }
    else if(e->key() == Qt::Key_F) {
        if(m_scale > 0) {
            m_scale -= 0.01;
            countFrame();
        }
    }
    else if(e->key() == Qt::Key_G) {
        if(m_scale < 1) {
            m_scale += 0.01;
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
            countEyelidTransform();
            countFrame();
        }
    }
    else if(e->key() == Qt::Key_J) {
        if(m_topEyelidRotation < 30) {
            m_topEyelidRotation++;
            countEyelidTransform();
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
        if(m_topEyelidY < HEIGHT/2) {
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
}

void MainWindow::countMove() {
    int ms = qrand()%301 + 100;                       //from 100 to 400
    float eyeToX = qrand()%161 + 80;                  //from 80  to 240
    float eyeToY = qrand()%81 + 100;                  //from 100 to 180
    float eyeRotation = qrand()%41 - 20;              //from -20 to 20
    float eyeRadius = (qrand()%51 + 100);             //from 100 to 150
    float eyeScale = (qrand()%51 + 50) / 100.0;       //from 0.5 to 1
    int eyeColorR = qrand()%256;                      //from 0   to 255
    int eyeColorG = qrand()%256;                      //from 0   to 255
    int eyeColorB = qrand()%256;                      //from 0   to 255
    float pupilRelSize = (qrand()%61 + 20) / 100.0;   //from 0.2 to 0.8
    float pupilRotation = qrand()%360;                //from 0   to 359
    float eyelidHeight = qrand()%101;                 //from 0   to 100
    float eyelidRotation = qrand()%61 - 30;           //from -30 to 30

    m_isMoveWithBlink = qrand()%2;

    move((MoveFlags)1, ms,
         eyeToX, eyeToY, eyeRotation, eyeRadius, eyeScale,
         eyeColorR, eyeColorG, eyeColorB,
         pupilRelSize, pupilRotation,
         eyelidHeight, eyelidRotation);

    m_randomMoveTimer->setInterval(m_msBetweenMovement);
}

void MainWindow::countShines() {
    m_shinesPaths.clear();
    m_shinesPaths.append(countShinePath(-40, -30, 50, 25, 105));
    m_shinesPaths.append(countShinePath(-12, 25, 15, 7, 120));
    for(int i = 0; i < m_shinesPaths.size(); i++) {
        m_shinesPaths[i] = m_eyeTransform.map(m_shinesPaths[i]);
    }
}

void MainWindow::countEyelid() {
    m_topEyelidPath = QPainterPath();
    m_topEyelidPath.moveTo(xLeft, yUp);
    m_topEyelidPath.lineTo(xLeft, m_topEyelidY);
    m_topEyelidPath.lineTo(xRight, m_topEyelidY);
    m_topEyelidPath.lineTo(xRight, yUp);
    m_topEyelidPath = m_eyelidTransform.map(m_topEyelidPath);
    m_topEyelidPath = m_eyeTransform.map(m_topEyelidPath);

    m_bottomEyelidPath = QPainterPath();
    m_bottomEyelidPath.moveTo(xLeft, yDown);
    m_bottomEyelidPath.lineTo(xLeft, 2 * BLINK_HEIGHT - m_topEyelidY);
    m_bottomEyelidPath.lineTo(xRight, 2 * BLINK_HEIGHT - m_topEyelidY);
    m_bottomEyelidPath.lineTo(xRight, yDown);
    m_bottomEyelidPath = m_eyelidTransform.map(m_bottomEyelidPath);
    m_bottomEyelidPath = m_eyeTransform.map(m_bottomEyelidPath);
}

QPainterPath MainWindow::countShinePath(int dx, int dy, int r1, int r2, int angle) {
    QPainterPath path;
    float r100 = m_R/100;
    QPointF center(m_c.x() + dx * r100, m_c.y() + dy * r100);

    QTransform t;
    t.translate(m_c.x(), m_c.y());
    t.scale(m_scale, 1);
    t.translate(-m_c.x(), -m_c.y());
    t.translate(center.x(), center.y());
    t.rotate(angle);
    t.translate(-center.x(), -center.y());
    path.addEllipse(center, r1 * r100, r2 * r100);
    path = t.map(path);

    return path;
}

void MainWindow::move(MoveFlags flags, int ms,
                      float eyeToX, float eyeToY, float eyeRotation,
                      float eyeRadius, float eyeRadiusScale,
                      int eyeColorR, int eyeColorG, int eyeColorB,
                      float pupilRelativeSize,
                      float pupilRotation,
                      float eyelidHeight,
                      float eyelidRotation) {
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
        m_stepPupilRotation = (m_endPupilRotation - m_alpha) * frac;
    }
    if(m_moveFlags & EyelidHeight && !m_isBlinking) {
        m_endTopEyelidHeight = eyelidHeight;
        m_stepTopEyelidHeight = (m_endTopEyelidHeight - m_topEyelidY) * frac;
    }
    if(m_moveFlags & EyelidRotation && !m_isMoveWithBlink) {
        m_endEyelidRotation = eyelidRotation;
        m_stepEyelidRotation = (m_endEyelidRotation - m_topEyelidRotation) * frac;
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

    bool recountEyeTransform = false;
    bool recountEyelidTransform = false;

    if(m_currentMovingTime > m_movingTime) {
        if(m_moveFlags & EyePosition) {
            m_c = m_endEyePosition;
        }
        if(m_moveFlags & EyeRotation) {
            m_rot = m_endEyeRotation;
            recountEyeTransform = true;
        }
        if(m_moveFlags & EyeSize) {
            m_R = m_endEyeRadius;
            m_scale = m_endEyeRadiusScale;
        }
        if(m_moveFlags & PupilSize) {
            m_relR8 = m_endPupilRelativeSize;
        }
        if(m_moveFlags & PupilRotation) {
            m_alpha = m_endPupilRotation;
        }
        if(m_moveFlags & EyelidHeight && !m_isBlinking) {
            m_topEyelidY = m_endTopEyelidHeight;
        }
        if(m_moveFlags & EyelidRotation) {
            m_topEyelidRotation = m_endEyelidRotation;
            recountEyelidTransform = true;
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
            recountEyeTransform = true;
        }
        if(m_moveFlags & EyeSize) {
            m_R += m_stepEyeRadius;
            m_scale += m_stepEyeRadiusScale;
        }
        if(m_moveFlags & PupilSize) {
            m_relR8 += m_stepPupilRelativeSize;
        }
        if(m_moveFlags & PupilRotation) {
            m_alpha += m_stepPupilRotation;
        }
        if(m_moveFlags & EyelidHeight && !m_isBlinking) {
            m_topEyelidY += m_stepTopEyelidHeight;
        }
        if(m_moveFlags & EyelidRotation) {
            m_topEyelidRotation += m_stepEyelidRotation;
            recountEyelidTransform = true;
        }
    }

    if(recountEyeTransform) {
        countEyeTransform();
    }
    if(recountEyelidTransform) {
        countEyelidTransform();
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

    float frac = m_msUpdateMove/(float)m_blinkingTime;
    m_endTopEyelidHeight = BLINK_HEIGHT;
    m_stepTopEyelidHeight = (m_endTopEyelidHeight - m_topEyelidY) * frac;

    m_blinkTimer->start();
}


void MainWindow::updateBlinkState() {
    m_currentBlinkingTime += m_msUpdateMove;

    if(m_isGoingDown && m_currentBlinkingTime > m_blinkingTime) {
        m_isGoingDown = false;
        m_topEyelidY = m_endTopEyelidHeight;

        float frac = m_msUpdateMove/(float)m_blinkingTime;
        m_endTopEyelidHeight = m_oldTopEyelidY;
        m_stepTopEyelidHeight = (m_endTopEyelidHeight - m_topEyelidY) * frac;

        if(m_isMoveWithBlink) {
            if(m_moveFlags & EyePosition) {
                m_c = m_endEyePosition;
            }
            if(m_moveFlags & EyeSize) {
                m_R = m_endEyeRadius;
                m_scale = m_endEyeRadiusScale;
            }
            if(m_moveFlags & PupilSize) {
                m_relR8 = m_endPupilRelativeSize;
            }
            if(m_moveFlags & PupilRotation) {
                m_alpha = m_endPupilRotation;
            }
            countFrame();
        }
    }
    else if(!m_isGoingDown && m_currentBlinkingTime > m_blinkingTime * 2) {
        m_topEyelidY = m_endTopEyelidHeight;
        m_currentBlinkingTime = 0;
        m_isBlinking = false;
        m_blinkTimer->stop();
    }
    else {
        m_topEyelidY += m_stepTopEyelidHeight;
    }

    countEyelid();
    update();
}

void MainWindow::countFrame() {
    countEye();
    countShines();
    countEyelid();
    update();
}

void MainWindow::countEye() {
    float alpha = m_alpha * PI/180;
    float absR8 = m_R * m_relR8;
    QPointF V = QPointF(m_c.x(), m_c.y() - absR8);

    m_Pin[0] = V;
    for(int i = 1; i < SIDES; i++) {
        m_Pin[i] = rotatePoint(m_Pin[i - 1], m_c, PI/4);
    }

    float betta = PI/8;
    float gamma = asin(absR8*sin(5*PI/8)/m_R);
    float delta = 3*PI/8 - gamma;
    float l = sqrt(m_R*m_R + absR8*absR8 - 2*m_R*absR8*cos(delta));
    float dx = l*cos(betta);
    float dy = l*sin(betta);

    m_Pout[0] = QPointF(V.x() - dx, V.y() - dy);
    for(int i = 1; i < SIDES; i++) {
        m_Pout[i] = rotatePoint(m_Pout[i - 1], m_c, PI/4);
    }

    if(m_alpha != 0) {
        for(int i = 0; i < SIDES; i++) {
            m_Pin[i] = rotatePoint(m_Pin[i], m_c, alpha);
            m_Pout[i] = rotatePoint(m_Pout[i], m_c, alpha);
        }
    }

    if(m_scale != 1.0) {
        for(int i = 0; i < SIDES; i++) {
            float newXin = m_Pin[i].x();
            newXin -= m_c.x();
            newXin *= m_scale;
            newXin += m_c.x();
            float newXout = m_Pout[i].x();
            newXout -= m_c.x();
            newXout *= m_scale;
            newXout += m_c.x();
            m_Pin[i].setX(newXin);
            m_Pout[i].setX(newXout);
        }
        m_R2 = m_R * m_scale;
    }
    else {
        m_R2 = m_R;
    }

    //countPath
    for(int i = 0; i < EyePathCount; i++) {
        m_eyePaths[i] = QPainterPath();
    }
    m_eyePaths[GreenEllipse].addEllipse(m_c, m_R2, m_R);

    m_Pin[SIDES] = m_Pin[0];
    m_eyePaths[BlackOctagonAndLines].addPolygon(QPolygonF(m_Pin));
    for(int i = 0; i < SIDES; i++) {
        m_eyePaths[BlackOctagonAndLines].moveTo(m_Pout[i]);
        m_eyePaths[BlackOctagonAndLines].lineTo(m_Pin[i]);
    }

    m_eyePaths[WhiteArea].addEllipse(m_c, m_R2, m_R);
    if(m_rot != 0) {
        for(int i = 0; i < EyePathCount; i++) {
            m_eyePaths[i] = m_eyeTransform.map(m_eyePaths[i]);
        }
    }
    m_eyePaths[WhiteArea].addRect(0, 0, WIDTH, HEIGHT);
}

void MainWindow::countEyeTransform() {
    m_eyeTransform.reset();
    m_eyeTransform.translate(m_c.x(), m_c.y());
    m_eyeTransform.rotate(m_rot);
    m_eyeTransform.translate(-m_c.x(), -m_c.y());
}


void MainWindow::countEyelidTransform() {
    m_eyelidTransform.reset();
    m_eyelidTransform.translate(WIDTH/2, m_topEyelidY);
    if(m_isLeftEye) {
        m_eyelidTransform.rotate(m_topEyelidRotation);
    }
    else {
        m_eyelidTransform.rotate(-m_topEyelidRotation);
    }
    m_eyelidTransform.translate(-WIDTH/2, -m_topEyelidY);
}

