#include "mainwindow.h"
#include "qmath.h"
#include <QPainter>
#include <QKeyEvent>
#include <QApplication>

// Convertion proportions for new eye resolution
// 320x240 -> 800x800
// 2.5x3.3333333333333335

MainWindow::MainWindow(bool isLeftEye, QWidget *parent) : QOpenGLWidget(parent),
    m_isLeftEye(isLeftEye),
    m_publishPixmap(false),

    m_blinkDefaultDuration(150),
    m_blinkDuration(0),
    m_blinkDelay(50),
    m_currentBlinkingTime(0),
    m_isBlinking(false),
    m_isGoingDown(false),
    m_isMoveWithBlink(false),

    m_msBetweenMovement(3000),
    m_randomMoveTimer(new QTimer(this)),

    m_blinkTimer(new QTimer(this)),
    m_moveTimer(new QTimer(this)),

    m_currentMovingTime(0),
    m_movingTime(0),
    m_msUpdateMove(16),

    m_moveFlags((MoveFlags)0)
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
        m_state.angle = -m_state.angle;
    }

    m_Pin.fill(QPointF(), SIDES + 1);
    m_Pout.fill(QPointF(), SIDES);
    m_eyePaths.fill(QPainterPath(), EyePathCount);

    setFixedSize(WIDTH, HEIGHT);

    computeEyeTransform();
    computeEyelidTransform();
    computeFrame();

    // Should be set on the widget to accept keyPressEvent as a child
    // or there's no way to set focus on the individual eye
    setFocusPolicy(Qt::StrongFocus);

    m_blinkTimer->setInterval(m_msUpdateMove);
    m_moveTimer->setInterval(m_msUpdateMove);
    connect(m_moveTimer, SIGNAL(timeout()), this, SLOT(updateMovingState()));
    connect(m_blinkTimer, SIGNAL(timeout()), this, SLOT(updateBlinkState()));
    connect(m_randomMoveTimer, SIGNAL(timeout()), this, SLOT(computeRandomMove()));

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

    painter.setPen(QPen(m_state.eyeColor));
    painter.setBrush(QColor(m_state.eyeColor));
    painter.drawPath(m_eyePaths[GreenEllipse]);

    painter.setPen(QPen(Qt::black, 3));
    painter.setBrush(QColor(Qt::black));
    painter.drawPath(m_eyePaths[BlackOctagonAndLines]);

    painter.setPen(QPen(m_state.whiteAreaColor));
    painter.setBrush(m_state.whiteAreaColor);
    painter.drawPath(m_eyePaths[WhiteArea]);

    painter.setPen(QPen(Qt::white));
    painter.setBrush(Qt::white);
    for(int i = 0; i < m_shinesPaths.size(); i++) {
        painter.drawPath(m_shinesPaths.at(i));
    }

    painter.setPen(QPen(m_state.eyelidOutlineColor, 2));
    painter.setBrush(m_state.eyelidColor);

    painter.drawPath(m_topEyelidPath);
    painter.drawPath(m_bottomEyelidPath);

    if (m_debug_mode_enabled) {
        painter.drawImage(0, 0, *overlay_);

        QString eye_info = QString("eye :: x,y: %1, %2; ang: %3; x_scale: %4; radius: %5").arg(QString::number(m_state.center.x()), QString::number(m_state.center.y()), QString::number(m_state.angle), QString::number(m_state.radiusRatio), QString::number(m_state.radius));
        QString aperture_info = QString("aperture :: ang: %1; contraction: %3\n").arg(QString::number(m_state.pupilAngle), QString::number(m_state.pupilRadius));
        QString top_eyelid_info = QString("top_eyelid :: y: %1; ang: %3\n").arg(QString::number(m_state.topEyelidY), QString::number(m_state.topEyelidAngle));
        QString bottom_eyelid_info = QString("bottom_eyelid :: y: %1; ang: %3\n").arg(QString::number(m_state.bottomEyelidY), QString::number(m_state.bottomEyelidAngle));
    
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
	switch(str2hash(msg->type.c_str())) {
    case str2hash("eyes/action"):
        switch(str2hash(msg->command.c_str())){
        case str2hash("blink"):
            blink(100);
            break;
        case str2hash("slow_blink"):
            blink(1000);
            break;
            // TODO: Add eye roll
            // TODO: Add eyes close

        }				
        break;

    case str2hash("eyes/emotion"):
        auto &s = m_state;

        switch(str2hash(msg->command.c_str())){
        case str2hash("normal"):
            s.resetColors();
            s.resetConfiguration();
            break;

        case str2hash("green_eyes"):
            s.resetColors();
            s.eyeColor = QColor(Qt::green);
            s.eyelidColor = QColor(143,210,143);
            s.eyelidOutlineColor = QColor(116,169,116);
            s.whiteAreaColor = QColor(Qt::white);
            break;

        case str2hash("red_eyes"):
            s.resetColors();
            s.eyeColor = QColor(Qt::red);
            s.eyelidColor = QColor(166,32,55);
            s.eyelidOutlineColor = QColor(0,0,0);
            s.whiteAreaColor = QColor(Qt::white);
            break;

        case str2hash("sad_look"):
            s.resetConfiguration();
            s.topEyelidAngle = 17;
            s.topEyelidY = 150;
            break;

        case str2hash("unamused_look"):
            s.resetConfiguration();
            s.topEyelidAngle = -2;
            s.topEyelidY = 290;
            break;

        case str2hash("surprised_look"):
            s.resetConfiguration();
            s.topEyelidY = 114;
            s.radius = 254.5;
            break;

        case str2hash("pleasure_look"):
            s.resetConfiguration();
            s.topEyelidY = 114;
            s.pupilRadius = 0.78;
            break;

        case str2hash("happy_look"):
            s.resetConfiguration();
            s.bottomEyelidY = 555;
            break;

        case str2hash("tender_look"):
            s.resetConfiguration();
            s.topEyelidAngle = 4;
            s.topEyelidY = 170;
            s.bottomEyelidY = 600;
            break;

        case str2hash("high_look"):
            s.resetConfiguration();
            s.topEyelidAngle = 6;
            s.topEyelidY = 310;
            s.radius = 290.5;
            s.pupilRadius = 0.87;

            s.resetColors();
            s.whiteAreaColor = QColor(255,183,195);
            break;

        case str2hash("scared_look"):
            s.resetConfiguration();
            s.topEyelidAngle = 5;
            s.bottomEyelidY = 670;
            s.bottomEyelidAngle = -5;
            s.radius = 200.5;
            s.pupilRadius = 0.46;
            break;

        case str2hash("very_scared_look"):
            s.resetConfiguration();
            s.topEyelidAngle = 5;
            s.radius = 122.5;
            s.pupilRadius = 0.4;
            break;

        case str2hash("raised_right_eyebrow_look"):
            s.resetConfiguration();
            if (m_isLeftEye)  s.topEyelidY = 119;
            else              s.topEyelidY = 263;
            break;

        case str2hash("raised_left_eyebrow_look"):
            s.resetConfiguration();
            if (!m_isLeftEye)  s.topEyelidY = 119;
            else               s.topEyelidY = 263;
            break;

        case str2hash("evil_look"):
            s.resetConfiguration();
            s.topEyelidAngle = -30;
            s.topEyelidY = 200;
            break;
        }
        break;
    }

	computeEyelidTransform();
	computeFrame();
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

  move(EyePosition, 30,
         eyeToX, eyeToY, 0, 0, 0,
         0, 0, 0,
         0, 0,
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
    auto &s = m_state;

    switch (e->key()) {
    case Qt::Key_Escape:
        QApplication::quit();
    	break;

    // Eye position control
    case Qt::Key_W:
        s.center.setY(s.center.y() - 1);
        computeEyeTransform();
        computeFrame();
    	break;

    case Qt::Key_A:
        s.center.setX(s.center.x() - 1);
        computeFrame();
    	break;

    case Qt::Key_S:
        s.center.setY(s.center.y() + 1);
        computeFrame();
    	break;

    case Qt::Key_D:
        s.center.setX(s.center.x() + 1);
        computeFrame();
    	break;


    // Whole eye rotation control (including eyelid)
    case Qt::Key_Q:
        s.angle -= 1;
        if(s.angle == -1)
            s.angle = 359;
        computeEyeTransform();
        computeFrame();
    	break;

    case Qt::Key_E:
        s.angle += 1;
        if(s.angle == 360)
            s.angle = 0;
        computeEyeTransform();
        computeFrame();
    	break;


    // Aperture rotation control
    case Qt::Key_T:
        s.pupilAngle += 1;
        if(s.pupilAngle == 360)
            s.pupilAngle = 0;
        computeFrame();
    	break;

    case Qt::Key_R:
        s.pupilAngle -= 1;
        if(s.pupilAngle == -1)
            s.pupilAngle = 359;
        computeFrame();
    	break;

    // Aperture contraction control
    case Qt::Key_C:
        if(s.pupilRadius > 0) {
            s.pupilRadius -= 0.01;
            computeFrame();
        }
    	break;

    case Qt::Key_V:
        if(s.pupilRadius < 1) {
            s.pupilRadius += 0.01;
            computeFrame();
        }
    	break;


    // Eye horizontal radius control
    case Qt::Key_F:
        if(s.radiusRatio > 0) {
            s.radiusRatio -= 0.01;
            computeFrame();
        }
    	break;

    case Qt::Key_G:
        if(s.radiusRatio < 1) {
            s.radiusRatio += 0.01;
            computeFrame();
        }
    	break;

    // Eye scale control
    case Qt::Key_Z:
        if(s.radius > 1) {
            s.radius -= 1;
            computeFrame();
        }
    	break;

    case Qt::Key_X:
        s.radius += 1;
        computeFrame();
    	break;

    // Top eyelid control
    case Qt::Key_H:
        if(s.topEyelidAngle > -30) {
            s.topEyelidAngle--;
            computeEyelidTransform();
            computeFrame();
        }
    	break;

    case Qt::Key_J:
        if(s.topEyelidAngle < 30) {
            s.topEyelidAngle++;
            computeEyelidTransform();
            computeFrame();
        }
    	break;

    case Qt::Key_N:
        if(s.topEyelidY > 0) {
            s.topEyelidY--;
            computeFrame();
        }
    	break;

    case Qt::Key_M:
        if(s.topEyelidY < HEIGHT/2) {
            s.topEyelidY++;
            computeFrame();
        }
    	break;


    // Bottom eyelid control
    case Qt::Key_K:
        if(s.bottomEyelidAngle > -30) {
            s.bottomEyelidAngle--;
            computeEyelidTransform();
            computeFrame();
        }
    	break;

    case Qt::Key_L:
        if(s.bottomEyelidAngle < 30) {
            s.bottomEyelidAngle++;
            computeEyelidTransform();
            computeFrame();
        }
    	break;

    case Qt::Key_Comma:
        if(s.bottomEyelidY > HEIGHT/2) {
            s.bottomEyelidY--;
            computeFrame();
        }
    	break;

    case Qt::Key_Period:
        if(s.bottomEyelidY < HEIGHT) {
            s.bottomEyelidY++;
            computeFrame();
        }
    	break;

    // Single blink initiation
    case Qt::Key_B:
        blink(m_blinkDefaultDuration);
    	break;

    // Random movements control
    case Qt::Key_Y:
        if(m_msBetweenMovement >= 1000) {
            m_msBetweenMovement -= 500;
            m_randomMoveTimer->setInterval(m_msBetweenMovement);
        }
    	break;

    case Qt::Key_U:
        m_msBetweenMovement += 500;
    	break;

    case Qt::Key_P:
        if(!m_randomMoveTimer->isActive()) {
            m_randomMoveTimer->start();
            computeRandomMove();
        }
        else {
            m_randomMoveTimer->stop();
            m_isMoveWithBlink = false;
        }
    	break;
    }
}

void MainWindow::computeRandomMove() {
    int ms = qrand()%301 + 100;                       //from 100 to 400
    float eyeToX = qrand()%400 + 200;                 //from 200 to 600
    float eyeToY = qrand()%200 + 300;                 //from 300 to 500
    float eyeRotation = qrand()%41 - 20;              //from -20 to 20
    float eyeRadius = (qrand()%101 + 290);            //from 190 to 290
    float eyeScale = (qrand()%51 + 50) / 100.0;       //from 0.5 to 1
    int eyeColorR = qrand()%256;                      //from 0   to 255
    int eyeColorG = qrand()%256;                      //from 0   to 255
    int eyeColorB = qrand()%256;                      //from 0   to 255
    float pupilRelSize = (qrand()%61 + 20) / 100.0;   //from 0.2 to 0.8
    float pupilRotation = qrand()%360;                //from 0   to 359
    float topEyelidHeight = qrand()%101 + 100;        //from 100 to 200
    float topEyelidRotation = qrand()%21 - 10;        //from -10 to 10
    float bottomEyelidHeight = qrand()%101 + 630;     //from 630 to 730 
    float bottomEyelidRotation = qrand()%21 - 10;     //from -10 to 10

    m_isMoveWithBlink = qrand()%2;

    move((MoveFlags)EyePosition, ms,
         eyeToX, eyeToY, eyeRotation, eyeRadius, eyeScale,
         eyeColorR, eyeColorG, eyeColorB,
         pupilRelSize, pupilRotation,
         topEyelidHeight, topEyelidRotation,
         bottomEyelidHeight, bottomEyelidRotation);

    m_randomMoveTimer->setInterval(m_msBetweenMovement);
}

void MainWindow::computeShines() {
    m_shinesPaths.clear();
    m_shinesPaths.append(computeShinePath(-40, -30, 50, 25, 105));
    m_shinesPaths.append(computeShinePath(-12, 25, 15, 7, 120));
    for(int i = 0; i < m_shinesPaths.size(); i++) {
        m_shinesPaths[i] = m_eyeTransform.map(m_shinesPaths[i]);
    }
}

void MainWindow::computeEyelid() {
    m_topEyelidPath = QPainterPath();
    m_topEyelidPath.moveTo(xLeft, yUp);
    m_topEyelidPath.lineTo(xLeft, m_state.topEyelidY);
    m_topEyelidPath.lineTo(xRight, m_state.topEyelidY);
    m_topEyelidPath.lineTo(xRight, yUp);
    m_topEyelidPath = m_topEyelidTransform.map(m_topEyelidPath);
    m_topEyelidPath = m_eyeTransform.map(m_topEyelidPath);

    m_bottomEyelidPath = QPainterPath();
    m_bottomEyelidPath.moveTo(xLeft, yDown);
    m_bottomEyelidPath.lineTo(xLeft, m_state.bottomEyelidY);
    m_bottomEyelidPath.lineTo(xRight, m_state.bottomEyelidY);
    m_bottomEyelidPath.lineTo(xRight, yDown);
    m_bottomEyelidPath = m_bottomEyelidTransform.map(m_bottomEyelidPath);
    m_bottomEyelidPath = m_eyeTransform.map(m_bottomEyelidPath);
}

QPainterPath MainWindow::computeShinePath(int dx, int dy, int r1, int r2, int angle) {
    QPainterPath path;
    float r100 = m_state.radius/100;
    QPointF center(m_state.center.x() + dx * r100, m_state.center.y() + dy * r100);

    QTransform t;
    t.translate(m_state.center.x(), m_state.center.y());
    t.scale(m_state.radiusRatio, 1);
    t.translate(-m_state.center.x(), -m_state.center.y());
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
                      float topEyelidHeight,
                      float topEyelidRotation,
                      float bottomEyelidHeight,
                      float bottomEyelidRotation) {
    m_currentMovingTime = 0;
    m_movingTime = ms;
    m_moveFlags = flags;

    float frac = m_msUpdateMove/(float)m_movingTime;

    auto &s = m_state;

    if(m_moveFlags & EyePosition) {
        m_endEyePosition = QPointF(eyeToX, eyeToY);
        m_stepEyePosition = (m_endEyePosition - s.center) * frac;
    }
    if(m_moveFlags & EyeRotation && !m_isMoveWithBlink) {
        m_endEyeRotation = eyeRotation;
        m_stepEyeRotation = (m_endEyeRotation - s.angle) * frac;
    }
    if(m_moveFlags & EyeSize) {
        m_endEyeRadius = eyeRadius;
        m_stepEyeRadius = (m_endEyeRadius - s.radius) * frac;

        m_endEyeRadiusScale = eyeRadiusScale;
        m_stepEyeRadiusScale = (m_endEyeRadiusScale - s.radiusRatio) * frac;
    }
    if(m_moveFlags & EyeColor) {
        s.eyeColor = QColor(eyeColorR, eyeColorG, eyeColorB);
    }
    if(m_moveFlags & PupilSize) {
        m_endPupilRelativeSize = pupilRelativeSize;
        m_stepPupilRelativeSize = (m_endPupilRelativeSize - s.pupilRadius) * frac;
    }
    if(m_moveFlags & PupilRotation) {
        m_endPupilRotation = pupilRotation;
        m_stepPupilRotation = (m_endPupilRotation - s.pupilAngle) * frac;
    }

    if(m_moveFlags & TopEyelidHeight && !m_isBlinking) {
        m_endTopEyelidY = topEyelidHeight;
        m_stepTopEyelidHeight = (m_endTopEyelidY - s.topEyelidY) * frac;
    }
    if(m_moveFlags & TopEyelidRotation && !m_isMoveWithBlink) {
        m_endTopEyelidRotation = topEyelidRotation;
        m_stepTopEyelidRotation = (m_endTopEyelidRotation - s.topEyelidAngle) * frac;
    }
    if(m_moveFlags & BottomEyelidHeight && !m_isBlinking) {
        m_endBottomEyelidY = bottomEyelidHeight;
        m_stepBottomEyelidHeight = (m_endBottomEyelidY - s.bottomEyelidY) * frac;
    }
    if(m_moveFlags & BottomEyelidRotation && !m_isMoveWithBlink) {
        m_endBottomEyelidRotation = bottomEyelidRotation;
        m_stepBottomEyelidRotation = (m_endBottomEyelidRotation - s.bottomEyelidAngle) * frac;
    }

    if(!m_isMoveWithBlink) {
        m_moveTimer->start();
    }
    else {
        blink(m_blinkDefaultDuration);
    }
}

void MainWindow::updateMovingState() {
    m_currentMovingTime += m_msUpdateMove;

    bool recomputeEyeTransform = false;
    bool recomputeEyelidTransform = false;

    auto &s = m_state;

    if(m_currentMovingTime > m_movingTime) {
        if(m_moveFlags & EyePosition) {
            s.center = m_endEyePosition;
        }
        if(m_moveFlags & EyeRotation) {
            s.angle = m_endEyeRotation;
            recomputeEyeTransform = true;
        }
        if(m_moveFlags & EyeSize) {
            s.radius = m_endEyeRadius;
            s.radiusRatio = m_endEyeRadiusScale;
        }
        if(m_moveFlags & PupilSize) {
            s.pupilRadius = m_endPupilRelativeSize;
        }
        if(m_moveFlags & PupilRotation) {
            s.pupilAngle = m_endPupilRotation;
        }

        if(m_moveFlags & TopEyelidHeight && !m_isBlinking) {
            s.topEyelidY = m_endTopEyelidY;
        }
        if(m_moveFlags & TopEyelidRotation) {
            s.topEyelidAngle = m_endTopEyelidRotation;
            recomputeEyelidTransform = true;
        }
        if(m_moveFlags & BottomEyelidHeight && !m_isBlinking) {
            s.bottomEyelidY = m_endBottomEyelidY;
        }
        if(m_moveFlags & BottomEyelidRotation) {
            s.bottomEyelidAngle = m_endBottomEyelidRotation;
            recomputeEyelidTransform = true;
        }

        m_currentMovingTime = 0;
        m_moveTimer->stop();
    }
    else {
        if(m_moveFlags & EyePosition) {
            s.center += m_stepEyePosition;
        }
        if(m_moveFlags & EyeRotation) {
            s.angle += m_stepEyeRotation;
            recomputeEyeTransform = true;
        }
        if(m_moveFlags & EyeSize) {
            s.radius += m_stepEyeRadius;
            s.radiusRatio += m_stepEyeRadiusScale;
        }
        if(m_moveFlags & PupilSize) {
            s.pupilRadius += m_stepPupilRelativeSize;
        }
        if(m_moveFlags & PupilRotation) {
            s.pupilAngle += m_stepPupilRotation;
        }

        if(m_moveFlags & TopEyelidHeight && !m_isBlinking) {
            s.topEyelidY += m_stepTopEyelidHeight;
        }
        if(m_moveFlags & TopEyelidRotation) {
            s.topEyelidAngle += m_stepTopEyelidRotation;
            recomputeEyelidTransform = true;
        }
        if(m_moveFlags & BottomEyelidHeight && !m_isBlinking) {
            s.bottomEyelidY += m_stepBottomEyelidHeight;
        }
        if(m_moveFlags & BottomEyelidRotation) {
            s.bottomEyelidAngle += m_stepBottomEyelidRotation;
            recomputeEyelidTransform = true;
        }
    }

    if(recomputeEyeTransform) {
        computeEyeTransform();
    }
    if(recomputeEyelidTransform) {
        computeEyelidTransform();
    }

    computeFrame();
}

inline float lerp(float first_value, float second_value, float t) {
    return first_value * (1 - t) + second_value * t;
}

inline float bezier_1d_cubic(float u0, float u1, float t) {
    float square = t * t;
    float cube = square * t;

    float inv = 1 - t;
    float inv_square = inv * (1 - t);

    return 3*inv_square*t*u0 + 3*inv*square*u1 + cube;
}

void MainWindow::blink(int duration_ms) {
    if (m_isBlinking)  return;

    m_isGoingDown = true;
    m_blinkDuration = 2 * duration_ms; // Moving forward and backward is 2 times longer

    // Save current eyelids state
    m_startTopEyelidY = m_state.topEyelidY;
    m_startBottomEyelidY = m_state.bottomEyelidY;
    m_startTopEyelidRotation = m_state.topEyelidAngle;
    m_startBottomEyelidRotation = m_state.bottomEyelidAngle;
    m_startApertureContraction = m_state.pupilRadius;

    // Compute target eyelids state
    float eyelidsTouchHightRatio = 0.8;
    float eyelidsTouchHight = lerp(m_state.topEyelidY, m_state.bottomEyelidY, eyelidsTouchHightRatio);
    float eyelidsTouchRotation = (m_state.topEyelidAngle + m_state.bottomEyelidAngle) * .5;

    m_endTopEyelidY = eyelidsTouchHight;
    m_endBottomEyelidY = eyelidsTouchHight;
    m_endTopEyelidRotation = eyelidsTouchRotation;
    m_endBottomEyelidRotation = eyelidsTouchRotation;
    m_endApertureContraction = 0.0;

    m_blinkTimer->start();
}

void MainWindow::updateBlinkState() {
    m_currentBlinkingTime += m_msUpdateMove;

    if(m_isGoingDown && m_currentBlinkingTime > (m_blinkDuration * 0.5 + m_blinkDelay)) {
        // Change direction of eyelid movement
        m_isGoingDown = false;

        // @Cleanup
        if(m_isMoveWithBlink) {
            if(m_moveFlags & EyePosition) {
                m_state.center = m_endEyePosition;
            }
            if(m_moveFlags & EyeSize) {
                m_state.radius = m_endEyeRadius;
                m_state.radiusRatio = m_endEyeRadiusScale;
            }
            if(m_moveFlags & PupilSize) {
                m_state.pupilRadius = m_endPupilRelativeSize;
            }
            if(m_moveFlags & PupilRotation) {
                m_state.pupilAngle = m_endPupilRotation;
            }
            computeFrame();
        }
    } else if(!m_isGoingDown && m_currentBlinkingTime > (m_blinkDuration + m_blinkDelay)) {
        // Stop movement
        m_state.topEyelidY = m_startTopEyelidY;
        m_state.bottomEyelidY = m_startBottomEyelidY;
        m_state.bottomEyelidAngle = m_startBottomEyelidRotation;
        m_state.topEyelidAngle = m_startTopEyelidRotation;
        m_state.pupilRadius = m_startApertureContraction;

        m_currentBlinkingTime = 0;
        m_isBlinking = false;
        m_blinkTimer->stop();
    } else {
        auto relativeBlinkingTime = 2 * m_currentBlinkingTime / (float)m_blinkDuration; // defined on [0;2] range

        // Incorporating delay between up and down motions
        if (m_currentBlinkingTime > (m_blinkDuration * 0.5 + m_blinkDelay)) {
            relativeBlinkingTime -= 2 * m_blinkDelay / (float)m_blinkDuration;
        } else if (m_currentBlinkingTime > m_blinkDuration * 0.5) {
            relativeBlinkingTime = 1.0;
        }

        // Reversing interpolation, after [0;1] range overflow
        if (relativeBlinkingTime > 1.0) {
            relativeBlinkingTime = 2.0 - relativeBlinkingTime;
        }

        m_state.topEyelidY    = lerp(m_startTopEyelidY, m_endTopEyelidY, relativeBlinkingTime);
        m_state.bottomEyelidY = lerp(m_startBottomEyelidY, m_endBottomEyelidY, relativeBlinkingTime);

        relativeBlinkingTime   = bezier_1d_cubic(0.1, -0.25, relativeBlinkingTime);
        m_state.topEyelidAngle    = lerp(m_startTopEyelidRotation, m_endTopEyelidRotation, relativeBlinkingTime);
        m_state.bottomEyelidAngle = lerp(m_startBottomEyelidRotation, m_endBottomEyelidRotation, relativeBlinkingTime);

        // Contract aperture while blinking
        auto relativeContractionTime = 6 * relativeBlinkingTime;
        m_state.pupilRadius = lerp(m_startApertureContraction, m_endApertureContraction, relativeContractionTime);
        m_state.pupilRadius = std::max(m_state.pupilRadius, m_endApertureContraction);
    }

    computeEyelidTransform();
    computeEyelid();
    computeEye();
    update();
}

void MainWindow::computeFrame() {
    computeEye();
    computeShines();
    computeEyelid();
    update();
}

void MainWindow::computeEye() {
    float alpha = m_state.pupilAngle * PI/180;
    float absR8 = m_state.radius * m_state.pupilRadius;
    QPointF V = QPointF(m_state.center.x(), m_state.center.y() - absR8);

    m_Pin[0] = V;
    for(int i = 1; i < SIDES; i++) {
        m_Pin[i] = rotatePoint(m_Pin[i - 1], m_state.center, PI/4);
    }

    float betta = PI/8;
    float gamma = asin(absR8*sin(5*PI/8)/m_state.radius);
    float delta = 3*PI/8 - gamma;
    float l = sqrt(m_state.radius*m_state.radius + absR8*absR8 - 2*m_state.radius*absR8*cos(delta));
    float dx = l*cos(betta);
    float dy = l*sin(betta);

    m_Pout[0] = QPointF(V.x() - dx, V.y() - dy);
    for(int i = 1; i < SIDES; i++) {
        m_Pout[i] = rotatePoint(m_Pout[i - 1], m_state.center, PI/4);
    }

    if(m_state.pupilAngle != 0) {
        for(int i = 0; i < SIDES; i++) {
            m_Pin[i] = rotatePoint(m_Pin[i], m_state.center, alpha);
            m_Pout[i] = rotatePoint(m_Pout[i], m_state.center, alpha);
        }
    }

    if(m_state.radiusRatio != 1.0) {
        for(int i = 0; i < SIDES; i++) {
            float newXin = m_Pin[i].x();
            newXin -= m_state.center.x();
            newXin *= m_state.radiusRatio;
            newXin += m_state.center.x();
            float newXout = m_Pout[i].x();
            newXout -= m_state.center.x();
            newXout *= m_state.radiusRatio;
            newXout += m_state.center.x();
            m_Pin[i].setX(newXin);
            m_Pout[i].setX(newXout);
        }
        m_state.radius2 = m_state.radius * m_state.radiusRatio;
    }
    else {
        m_state.radius2 = m_state.radius;
    }

    //computePath
    for(int i = 0; i < EyePathCount; i++) {
        m_eyePaths[i] = QPainterPath();
    }
    m_eyePaths[GreenEllipse].addEllipse(m_state.center, m_state.radius2, m_state.radius);

    m_Pin[SIDES] = m_Pin[0];
    m_eyePaths[BlackOctagonAndLines].addPolygon(QPolygonF(m_Pin));
    for(int i = 0; i < SIDES; i++) {
        m_eyePaths[BlackOctagonAndLines].moveTo(m_Pout[i]);
        m_eyePaths[BlackOctagonAndLines].lineTo(m_Pin[i]);
    }

    m_eyePaths[WhiteArea].addEllipse(m_state.center, m_state.radius2, m_state.radius);
    if(m_state.angle != 0) {
        for(int i = 0; i < EyePathCount; i++) {
            m_eyePaths[i] = m_eyeTransform.map(m_eyePaths[i]);
        }
    }
    m_eyePaths[WhiteArea].addRect(0, 0, WIDTH, HEIGHT);
}

void MainWindow::computeEyeTransform() {
    m_eyeTransform.reset();
    m_eyeTransform.translate(m_state.center.x(), m_state.center.y());
    m_eyeTransform.rotate(m_state.angle);
    m_eyeTransform.translate(-m_state.center.x(), -m_state.center.y());
}


void MainWindow::computeEyelidTransform() {
    m_topEyelidTransform.reset();
    m_topEyelidTransform.translate(WIDTH/2, m_state.topEyelidY);
    if(m_isLeftEye) {
        m_topEyelidTransform.rotate(m_state.topEyelidAngle);
    }
    else {
        m_topEyelidTransform.rotate(-m_state.topEyelidAngle);
    }
    m_topEyelidTransform.translate(-WIDTH/2, -m_state.topEyelidY);

    m_bottomEyelidTransform.reset();
    m_bottomEyelidTransform.translate(WIDTH/2, m_state.bottomEyelidY);
    if(m_isLeftEye) {
        m_bottomEyelidTransform.rotate(m_state.bottomEyelidAngle);
    }
    else {
        m_bottomEyelidTransform.rotate(-m_state.bottomEyelidAngle);
    }
    m_bottomEyelidTransform.translate(-WIDTH/2, -m_state.bottomEyelidY);
}

