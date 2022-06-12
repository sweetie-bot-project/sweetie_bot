#include "mainwindow.h"
#include "qmath.h"
#include <QPainter>
#include <QKeyEvent>
#include <QApplication>

// Convertion proportions for new eye resolution
// 320x240 -> 800x800
// 2.5x3.3333333333333335

EyeWindow::EyeWindow(bool isLeftEye, QWidget *parent) :
    QOpenGLWidget(parent),

    m_publishPixmap(false),

    m_state(isLeftEye),

    m_blinkDefaultDuration(150),
    m_blinkDuration(0),
    m_blinkDelay(50),
    m_currentBlinkingTime(0),
    m_isBlinking(false),
    m_isGoingDown(false),
    m_isMoveWithBlink(false),

    m_blinkTimer(new QTimer(this)),
    m_moveTimer(new QTimer(this)),

    m_msBetweenMovement(3000),
    m_randomMoveTimer(new QTimer(this)),

    m_isMoveDryRunning(false),
    m_isBlinkDryRunning(false),

    m_currentMovingTime(0),
    m_movingTime(0),
    m_msUpdateMove(16),
    m_isMoving(false),

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

    m_Pin.fill(QPointF(), APERTURE_EDGE_COUNT + 1);
    m_Pout.fill(QPointF(), APERTURE_EDGE_COUNT);
    m_eyePaths.fill(QPainterPath(), EyePathCount);

    setFixedSize(WIDTH, HEIGHT);

    computeEyeTransform();
    computeEyelidTransform();
    computeFrame();

    // Should be set on the widget to accept keyPressEvent as a child
    // or there's no way to set focus on the individual eye
    setFocusPolicy(Qt::StrongFocus);

    connect(m_moveTimer, SIGNAL(timeout()), this, SLOT(updateMovingState()));
    connect(m_blinkTimer, SIGNAL(timeout()), this, SLOT(updateBlinkState()));
    connect(m_randomMoveTimer, SIGNAL(timeout()), this, SLOT(computeRandomMove()));

    m_moveTimer->setInterval(m_msUpdateMove);
    m_blinkTimer->setInterval(m_msUpdateMove);

    // NODE INTERFACE 
    // parameteres
    ros::param::get("~publish_pixmap", m_publishPixmap);
    // publishers
    std::string image_eye_topic_name;
    switch (m_state.side) {
    case LEFT:  image_eye_topic_name = "eye_image_left";  break;
    case RIGHT: image_eye_topic_name = "eye_image_right"; break;
    }
    pub_eye_image_ = node_.advertise<sensor_msgs::Image>(image_eye_topic_name, 1);

    path_ =  QString::fromStdString( ros::package::getPath("sweetie_bot_eyes") );

    // Set debug mode
    auto args = QApplication::arguments();
    m_debug_mode_enabled = args.contains("-debug");

    if (m_debug_mode_enabled) {
        switch (m_state.side) {
        case LEFT:
            overlay_ = new QImage(path_ + "/overlays/proto3_leftEyeOverlay.png");
            break;
        case RIGHT:
            overlay_ = new QImage(path_ + "/overlays/proto3_rightEyeOverlay.png");
            break;
        }
    }
}

EyeWindow::~EyeWindow() {
    delete m_fbo;
}

void EyeWindow::connectMoveTimer(QTimer *timer) {
    connect(timer, SIGNAL(timeout()), this, SLOT(updateMovingState()));
}

void EyeWindow::connectBlinkTimer(QTimer *timer) {
    connect(timer, SIGNAL(timeout()), this, SLOT(updateBlinkState()));
}

void EyeWindow::initializeGL() {
    QOpenGLFramebufferObjectFormat format;
    format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);

    m_fbo = new QOpenGLFramebufferObject(width(), height(), format);
}

void EyeWindow::paintGL() {
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

void EyeWindow::PublishImage()
{
    // @Speed: It's slow to render offscreen buffer in current thread
    //         But it only happens on host machine, so we can fix this later.
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

QPointF EyeWindow::rotatePoint(QPointF point, QPointF center, float angle) {
    float x = point.x() - center.x();
    float y = point.y() - center.y();
    float newx = x * cos(angle) - y * sin(angle);
    float newy = x * sin(angle) + y * cos(angle);
    newx = newx + center.x();
    newy = newy + center.y();
    return QPointF(newx,newy);
}

void EyeWindow::keyPressEvent(QKeyEvent *e) {
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

void EyeWindow::computeRandomMove() {
    int ms = qrand()%301 + 100;                                   //from 100 to 400

    EyeState randomState;
    randomState.center            = QPointF(qrand()%200 + 300,    //from 300 to 500
                                            qrand()%400 + 200);   //from 200 to 600

    randomState.angle             = qrand()%41 - 20;              //from -20 to 20
    randomState.radius            = qrand()%201 + 90;             //from 90 to 290
    randomState.radiusRatio       = (qrand()%51 + 50) / 100.0;    //from 0.5 to 1

    randomState.eyeColor          = QColor(qrand()%256,           // from 0 to 255
                                           qrand()%256,           // from 0 to 255
                                           qrand()%256);          // from 0 to 255

    randomState.pupilRadius       = (qrand()%61 + 20) / 100.0;    //from 0.2 to 0.8
    randomState.pupilAngle        = qrand()%360;                  //from 0   to 359

    randomState.topEyelidY        = qrand()%101 + 100;            //from 100 to 200
    randomState.topEyelidAngle    = qrand()%21 - 10;              //from -10 to 10
    randomState.bottomEyelidY     = qrand()%101 + 630;            //from 630 to 730 
    randomState.bottomEyelidAngle = qrand()%21 - 10;              //from -10 to 10

    m_isMoveWithBlink = qrand()%2;

    move((MoveFlags)(PupilSize | PupilRotation | TopEyelidHeight | TopEyelidRotation | BottomEyelidHeight | BottomEyelidRotation), ms, randomState);

    m_randomMoveTimer->setInterval(m_msBetweenMovement);
}

void EyeWindow::computeShines() {
    m_shinesPaths.clear();
    m_shinesPaths.append(computeShinePath(-40, -30, 50, 25, 105));
    m_shinesPaths.append(computeShinePath(-12, 25, 15, 7, 120));
    for(int i = 0; i < m_shinesPaths.size(); i++) {
        m_shinesPaths[i] = m_eyeTransform.map(m_shinesPaths[i]);
    }
}

void EyeWindow::computeEyelid() {
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

QPainterPath EyeWindow::computeShinePath(int dx, int dy, int r1, int r2, int angle) {
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

void EyeWindow::move(MoveFlags flags, int ms, EyeState targetState, bool moveWithBlink, bool dryRun) {
    // @Note: Right now we don't allow several simultanious motions to execute.
    //        But in the long run, it will be preferable to enable asyncronious
    //        movements, as it'll more smoothly integrate with other parts of the robot system.
    if (m_isMoving || m_isMoveDryRunning)  return;

    m_isMoveDryRunning = dryRun;

    m_movingTime = ms;
    m_moveFlags = flags;
    m_currentMovingTime = 0;

    m_isMoveWithBlink = moveWithBlink;

    // Disabling certain movement to not race with blink motion
    if (!dryRun) {
        if (m_isMoveWithBlink)  flags = (MoveFlags)(flags & ~EyeRotation);
        if (m_isBlinking)       flags = (MoveFlags)(flags & ~TopEyelidHeight);
        if (m_isMoveWithBlink)  flags = (MoveFlags)(flags & ~TopEyelidRotation);
        if (m_isBlinking)       flags = (MoveFlags)(flags & ~BottomEyelidHeight);
        if (m_isMoveWithBlink)  flags = (MoveFlags)(flags & ~BottomEyelidRotation);
    }

    m_endAnimationState.assignSelectively(targetState, flags);

    // Assign color immediately to the current state, without animating
    if (flags & EyeColors) {
        m_state.assignSelectively(targetState, (MoveFlags)EyeColors);
    }

    if (!m_isMoveWithBlink) {
        if (!dryRun)  m_moveTimer->start();
    } else {
        blink(m_blinkDefaultDuration, dryRun);
    }
}

void EyeWindow::updateMovingState() {
    // Disabling certain movement to not race with blink motion
    auto &flags = m_moveFlags;
    if (!m_isMoveDryRunning) {
        if (m_isMoveWithBlink)  flags = (MoveFlags)(flags & ~EyeRotation);
        if (m_isBlinking)       flags = (MoveFlags)(flags & ~TopEyelidHeight);
        if (m_isMoveWithBlink)  flags = (MoveFlags)(flags & ~TopEyelidRotation);
        if (m_isBlinking)       flags = (MoveFlags)(flags & ~BottomEyelidHeight);
        if (m_isMoveWithBlink)  flags = (MoveFlags)(flags & ~BottomEyelidRotation);
    }


    // Assign start animation state at first iteration, so it would be up to date in the case of delayed move
    if (m_currentMovingTime == 0) {
        m_isMoving = true;

        m_startAnimationState.assignSelectively(m_state, flags);
    }

    m_currentMovingTime += m_msUpdateMove;

    if (m_currentMovingTime > m_movingTime) {
        // Making sure current state is in the end position
        m_state.assignSelectively(m_endAnimationState, flags);

        m_currentMovingTime = 0;
        m_isMoving = false;

        if (!m_isMoveDryRunning)  m_moveTimer->stop();
        m_isMoveDryRunning = false;
    } else {
        auto relativeMovingTime = m_currentMovingTime / (float)m_movingTime;
        auto interpolatedState = lerp(m_startAnimationState, m_endAnimationState, relativeMovingTime);

        m_state.assignSelectively(interpolatedState, flags);
    }

    if (m_moveFlags & EyeRotation)  computeEyeTransform();
    if (m_moveFlags & (TopEyelidRotation | BottomEyelidRotation))  computeEyelidTransform();

    computeFrame();
}

void EyeWindow::blink(int duration_ms, bool dryRun) {
    if (m_isBlinking || m_isBlinkDryRunning)  return;

    m_isBlinkDryRunning = dryRun;

    m_isGoingDown = true;
    m_blinkDuration = 2 * duration_ms; // Moving forward and backward takes 2 times longer
    m_currentBlinkingTime = 0;

    if (!dryRun)  m_blinkTimer->start();
}

void EyeWindow::updateBlinkState() {
    // Assign start animation state at first iteration as well as dependent end animation state,
    // so they would be up to date in the calse of delayed move
    if (m_currentBlinkingTime == 0) {
        m_isBlinking = true;

        auto initState = m_state;
        m_startBlinkAnimationState.assignOnlyBlinkState(initState);

        // Compute target eyelids state
        float eyelidsTouchHightRatio = 0.8;
        float eyelidsTouchHight = lerp(initState.topEyelidY, initState.bottomEyelidY, eyelidsTouchHightRatio);
        float eyelidsTouchAngle = (initState.topEyelidAngle + initState.bottomEyelidAngle) * .5;

        m_endBlinkAnimationState.topEyelidY = eyelidsTouchHight;
        m_endBlinkAnimationState.bottomEyelidY = eyelidsTouchHight;
        m_endBlinkAnimationState.topEyelidAngle = eyelidsTouchAngle;
        m_endBlinkAnimationState.bottomEyelidAngle = eyelidsTouchAngle;
        m_endBlinkAnimationState.pupilRadius = 0;
    }

    m_currentBlinkingTime += m_msUpdateMove;

    if (m_isGoingDown && m_currentBlinkingTime > (m_blinkDuration * 0.5 + m_blinkDelay)) {
        // Change direction of eyelid movement
        m_isGoingDown = false;

        if(m_isMoveWithBlink) {
            auto moveOnBlinkConfig = (MoveFlags)(m_moveFlags & (EyePosition | EyeSize | PupilSize | PupilRotation));
            m_state.assignSelectively(m_endBlinkAnimationState, moveOnBlinkConfig);
            computeFrame();
        }
    } else if (!m_isGoingDown && m_currentBlinkingTime > (m_blinkDuration + m_blinkDelay)) {
        // Make sure end blink state match starting state
        m_state.assignOnlyBlinkState(m_startBlinkAnimationState);

        // Stop movement
        m_currentBlinkingTime = 0;
        m_isBlinking = false;

        if (!m_isBlinkDryRunning)  m_blinkTimer->stop();
        m_isBlinkDryRunning = false;
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

        m_state.topEyelidY    = lerp(m_startBlinkAnimationState.topEyelidY, m_endBlinkAnimationState.topEyelidY, relativeBlinkingTime);
        m_state.bottomEyelidY = lerp(m_startBlinkAnimationState.bottomEyelidY, m_endBlinkAnimationState.bottomEyelidY, relativeBlinkingTime);

        relativeBlinkingTime      = bezier_1d_cubic(0.1, -0.25, relativeBlinkingTime);
        m_state.topEyelidAngle    = lerp(m_startBlinkAnimationState.topEyelidAngle, m_endBlinkAnimationState.topEyelidAngle, relativeBlinkingTime);
        m_state.bottomEyelidAngle = lerp(m_startBlinkAnimationState.bottomEyelidAngle, m_endBlinkAnimationState.bottomEyelidAngle, relativeBlinkingTime);

        // Contract aperture while blinking
        auto relativeContractionTime = 6 * relativeBlinkingTime;
        m_state.pupilRadius = lerp(m_startBlinkAnimationState.pupilRadius, m_endBlinkAnimationState.pupilRadius, relativeContractionTime);
        m_state.pupilRadius = std::max(m_state.pupilRadius, m_endBlinkAnimationState.pupilRadius);
    }

    computeEyelidTransform();
    computeEyelid();
    computeEye();
    update();
}

void EyeWindow::resetState() {
    m_state.resetColors();
    m_state.resetConfiguration();
}

void EyeWindow::computeFrame() {
    computeEye();
    computeShines();
    computeEyelid();
    update();
}

void EyeWindow::computeEye() {
    float alpha = m_state.pupilAngle * PI/180;
    float absR8 = m_state.radius * m_state.pupilRadius;
    QPointF V = QPointF(m_state.center.x(), m_state.center.y() - absR8);

    m_Pin[0] = V;
    for (int i = 1; i < APERTURE_EDGE_COUNT; i++) {
        m_Pin[i] = rotatePoint(m_Pin[i - 1], m_state.center, PI/4);
    }

    float betta = PI/8;
    float gamma = asin(absR8*sin(5*PI/8)/m_state.radius);
    float delta = 3*PI/8 - gamma;
    float l = sqrt(m_state.radius*m_state.radius + absR8*absR8 - 2*m_state.radius*absR8*cos(delta));
    float dx = l*cos(betta);
    float dy = l*sin(betta);

    m_Pout[0] = QPointF(V.x() - dx, V.y() - dy);
    for (int i = 1; i < APERTURE_EDGE_COUNT; i++) {
        m_Pout[i] = rotatePoint(m_Pout[i - 1], m_state.center, PI/4);
    }

    if (m_state.pupilAngle != 0) {
        for (int i = 0; i < APERTURE_EDGE_COUNT; i++) {
            m_Pin[i] = rotatePoint(m_Pin[i], m_state.center, alpha);
            m_Pout[i] = rotatePoint(m_Pout[i], m_state.center, alpha);
        }
    }

    if (m_state.radiusRatio != 1.0) {
        for (int i = 0; i < APERTURE_EDGE_COUNT; i++) {
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
    for (int i = 0; i < EyePathCount; i++) {
        m_eyePaths[i] = QPainterPath();
    }
    m_eyePaths[GreenEllipse].addEllipse(m_state.center, m_state.radius2, m_state.radius);

    m_Pin[APERTURE_EDGE_COUNT] = m_Pin[0];
    m_eyePaths[BlackOctagonAndLines].addPolygon(QPolygonF(m_Pin));
    for (int i = 0; i < APERTURE_EDGE_COUNT; i++) {
        m_eyePaths[BlackOctagonAndLines].moveTo(m_Pout[i]);
        m_eyePaths[BlackOctagonAndLines].lineTo(m_Pin[i]);
    }

    m_eyePaths[WhiteArea].addEllipse(m_state.center, m_state.radius2, m_state.radius);
    if (m_state.angle != 0) {
        for (int i = 0; i < EyePathCount; i++) {
            m_eyePaths[i] = m_eyeTransform.map(m_eyePaths[i]);
        }
    }
    m_eyePaths[WhiteArea].addRect(0, 0, WIDTH, HEIGHT);
}

void EyeWindow::computeEyeTransform() {
    m_eyeTransform.reset();
    // @Temporary: Add separate rotation points for eyeTransform and eye itself
    m_eyeTransform.translate(WIDTH/2, HEIGHT/2);
    m_eyeTransform.rotate(m_state.angle);
    m_eyeTransform.translate(-WIDTH/2, -HEIGHT/2);
}


void EyeWindow::computeEyelidTransform() {
    m_topEyelidTransform.reset();
    m_topEyelidTransform.translate(WIDTH/2, m_state.topEyelidY);
    switch (m_state.side) {
    case LEFT:   m_topEyelidTransform.rotate(m_state.topEyelidAngle); break;
    case RIGHT:  m_topEyelidTransform.rotate(-m_state.topEyelidAngle); break;
    }
    m_topEyelidTransform.translate(-WIDTH/2, -m_state.topEyelidY);

    m_bottomEyelidTransform.reset();
    m_bottomEyelidTransform.translate(WIDTH/2, m_state.bottomEyelidY);
    switch (m_state.side) {
    case LEFT:   m_bottomEyelidTransform.rotate(m_state.bottomEyelidAngle); break;
    case RIGHT:  m_bottomEyelidTransform.rotate(-m_state.bottomEyelidAngle); break;
    }
    m_bottomEyelidTransform.translate(-WIDTH/2, -m_state.bottomEyelidY);
}

