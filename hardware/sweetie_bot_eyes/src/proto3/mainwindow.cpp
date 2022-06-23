#include "mainwindow.h"
#include "consts.h"
#include "eye_state.h"

#include <QThread>

MainWindow::MainWindow(QWidget *parent) :
    QOpenGLWidget(parent),

    m_moveTimer(new QTimer(this)),
    m_blinkTimer(new QTimer(this)),
    m_randomBlinkingTimer(new QTimer(this)),

    m_msBlinkGenerationInterval(500),
    m_blinkGenerationTime(0),
    m_timeUntilNextBlink(0),

    m_playingAnimation(nullptr),

    // TODO: In future provide several options for simulating different
    // concentration levels during resting, converstaion and searching behavious
    m_blinkTimeDistribution(2.0, 0.65),
    m_uniformDist(0, 1.0),

    m_autoBlinkDelay(50),

    m_rightEye(new EyeWindow(false, this)),
    m_leftEye(new EyeWindow(true, this)),
    m_uiLayout(new QHBoxLayout(this))
{
    resize(2 * WIDTH, HEIGHT);

    m_uiLayout->addWidget(m_rightEye);
    m_uiLayout->addWidget(m_leftEye);
    m_uiLayout->setMargin(0);
    m_uiLayout->setSpacing(0);

    // subscribers
    sub_control_ = node_.subscribe<sweetie_bot_text_msgs::TextCommand>("control", 1, &MainWindow::controlCallback, this, ros::TransportHints().tcpNoDelay());
    sub_joint_state_ = node_.subscribe<sensor_msgs::JointState>("joint_states", 1, &MainWindow::moveCallback, this, ros::TransportHints().tcpNoDelay());


    m_leftEye->connectMoveTimer(m_moveTimer);
    m_leftEye->connectBlinkTimer(m_blinkTimer);

    m_rightEye->connectMoveTimer(m_moveTimer);
    m_rightEye->connectBlinkTimer(m_blinkTimer);

    int msUpdateInterval = 16;
    m_moveTimer->setInterval(msUpdateInterval);
    m_blinkTimer->setInterval(msUpdateInterval);
    m_randomBlinkingTimer->setInterval(m_msBlinkGenerationInterval);

    connect(m_blinkTimer, SIGNAL(timeout()), this, SLOT(updateBlink()));
    connect(m_moveTimer, SIGNAL(timeout()), this, SLOT(updateMove()));
    connect(m_randomBlinkingTimer, SIGNAL(timeout()), this, SLOT(randomBlinkingGeneration()));

    ros::param::get("~publish_pixmap", m_publishPixmap);

    bool blinkingGenerationEnabled = false;
    ros::param::get("~start_blinking", blinkingGenerationEnabled);
    ros::param::get("~auto_blink_delay", m_autoBlinkDelay);

    m_noiseGenerator.seed(m_rd());
    if (blinkingGenerationEnabled)  m_randomBlinkingTimer->start();

    initAnimations();


    // Start ros and abandon it
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(rosSpin()));
    timer->start(10);
}

MainWindow::~MainWindow() {
    m_randomBlinkingTimer->stop();
}

void MainWindow::rosSpin()
{
    if(!ros::ok()) QApplication::quit();
    ros::spinOnce();
}

// @Hardcode
void MainWindow::initAnimations() {
    EyeState stateRight(false);
    EyeState stateLeft(true);
    int ms = 50;

    m_animations.insert(std::make_pair<std::string, EyesAnimation *>("eye_roll", new EyesAnimation()));

    // 0
    stateRight.center = QPointF(209, 400);
    stateLeft.center = QPointF(209, 400);
    // stateRight.topEyelidY = 180;
    // stateLeft.topEyelidY = 180;

    m_animations["eye_roll"]->setInitialStates(stateLeft, stateRight);

    // 1
    stateRight.center = QPointF(240, 325);
    stateLeft.center = QPointF(240, 325);

    m_animations["eye_roll"]->appendStates(stateLeft, stateRight, ms);

    // 2
    stateRight.center = QPointF(253, 270);
    stateLeft.center = QPointF(253, 270);

    m_animations["eye_roll"]->appendStates(stateLeft, stateRight, ms);

    // 3
    stateRight.center = QPointF(292, 235);
    stateLeft.center = QPointF(292, 235);

    m_animations["eye_roll"]->appendStates(stateLeft, stateRight, ms);

    // 4
    stateRight.center = QPointF(330, 222);
    stateLeft.center = QPointF(330, 222);

    m_animations["eye_roll"]->appendStates(stateLeft, stateRight, ms);

    // 5
    stateRight.center = QPointF(376, 216);
    stateLeft.center = QPointF(376, 216);

    m_animations["eye_roll"]->appendStates(stateLeft, stateRight, ms);

    // 6
    stateRight.center = QPointF(430, 216);
    stateLeft.center = QPointF(430, 216);

    m_animations["eye_roll"]->appendStates(stateLeft, stateRight, ms);

    // 7
    stateRight.center = QPointF(484, 230);
    stateLeft.center = QPointF(484, 230);

    m_animations["eye_roll"]->appendStates(stateLeft, stateRight, ms);

    // 8
    stateRight.center = QPointF(537, 266);
    stateLeft.center = QPointF(537, 266);

    m_animations["eye_roll"]->appendStates(stateLeft, stateRight, ms);

    // 9
    stateRight.center = QPointF(582, 297);
    stateLeft.center = QPointF(582, 297);

    m_animations["eye_roll"]->appendStates(stateLeft, stateRight, ms);

    // 10
    stateRight.center = QPointF(582, 297);
    stateLeft.center = QPointF(582, 297);

    ms = 500;
    m_animations["eye_roll"]->appendStates(stateLeft, stateRight, ms);

}

constexpr unsigned int str2hash(const char* str, int h = 0)
{
    return !str[h] ? 5381 : (str2hash(str, h+1)*33) ^ str[h];
}

EyeState MainWindow::generateEmotion(const EyeState &baseState, const char *emotionName) {
    auto s = baseState;

    switch(str2hash(emotionName)){
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
        s.eyelidColor = QColor(117,15,32);
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
        s.topEyelidY = 320;
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
        s.topEyelidY = 100;
        s.bottomEyelidY = 590;
        s.bottomEyelidAngle = -3;
        break;

    case str2hash("tender_look"):
        s.resetConfiguration();
        s.topEyelidAngle = 4;
        s.topEyelidY = 150;
        s.bottomEyelidAngle = -7;
        s.bottomEyelidY = 620;
        break;

    case str2hash("high_look"):
        s.resetConfiguration();
        s.topEyelidAngle = 6;
        s.topEyelidY = 310;
        s.radius = 290.5;
        s.pupilRadius = 0.87;

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
	s.topEyelidAngle = -2;
        if (baseState.side == LEFT)  s.topEyelidY = 119;
        else                         s.topEyelidY = 320;
        break;

    case str2hash("raised_left_eyebrow_look"):
        s.resetConfiguration();
	s.topEyelidAngle = -2;
        if (baseState.side == RIGHT)  s.topEyelidY = 119;
        else                          s.topEyelidY = 320;
        break;

    case str2hash("evil_look"):
        s.resetConfiguration();
        s.topEyelidAngle = -30;
        s.topEyelidY = 200;
        break;
    }

    return s;
}

void MainWindow::controlCallback(const sweetie_bot_text_msgs::TextCommand::ConstPtr& msg)
{
	//ROS_INFO_STREAM("\n" << *msg);
	switch(str2hash(msg->type.c_str())) {
    case str2hash("eyes/action"):
        switch(str2hash(msg->command.c_str())){
        case str2hash("start_blinking"):
            m_randomBlinkingTimer->start();
            break;
        case str2hash("stop_blinking"):
            m_randomBlinkingTimer->stop();
            break;

        case str2hash("blink"):
            blinkBothEyes(100);
            break;
        case str2hash("slow_blink"):
            blinkBothEyes(1000);
            break;

        case str2hash("eye_roll"):
            playAnimation(m_animations["eye_roll"]);
            break;

            // TODO: Add eyes close
            // TODO: Add aperture movements
            // TODO: Add randomized eye jumps
        }				
        break;

    case str2hash("eyes/emotion"): {
        m_restartAutoblink = m_randomBlinkingTimer->isActive();
        if (m_restartAutoblink)  m_randomBlinkingTimer->stop();

        auto emotionName = msg->command.c_str();

        auto &leftState  = m_leftEye->getState();
        auto &rightState = m_rightEye->getState();

        auto targetLeftState  = generateEmotion(leftState,  emotionName);
        auto targetRightState = generateEmotion(rightState, emotionName);

        // Smoothing transition between emotions
        if (str2hash(msg->command.c_str()) != str2hash("reset")) {
            moveBothEyes((MoveFlags)(~EyePosition), 200, targetLeftState, targetRightState);
        } else {
            m_leftEye->resetState();
            m_rightEye->resetState();
        }
        break;
    }
    }

    m_leftEye->computeEyelidTransform();
    m_leftEye->computeFrame();

    m_rightEye->computeEyelidTransform();
    m_rightEye->computeFrame();

    if (m_publishPixmap) {
        m_leftEye->PublishImage();
        m_rightEye->PublishImage();
    }
}

void MainWindow::moveCallback(const sensor_msgs::JointState::ConstPtr& msg) {
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

    auto diffLeft_x = abs(eyeToX - m_leftEye->getState().center.x());
    auto diffLeft_y = abs(eyeToY - m_leftEye->getState().center.y());

    auto diffRight_x = abs(eyeToX - m_leftEye->getState().center.x());
    auto diffRight_y = abs(eyeToY - m_leftEye->getState().center.y());

    bool isControllOverrided = m_animationStage > 0 || m_leftEye->isMouseEnabled() || m_rightEye->isMouseEnabled();
    if (!isControllOverrided) {
        // @Hack: Instead, should've implemented unified update-loop where
        //        all updating takes place without turning timer on and off at each
        //        received joint_state mesage, which is not effective at all.
        if (diffLeft_x  > 1.0 || diffLeft_y  > 1.0 ||
            diffRight_x > 1.0 || diffRight_y > 1.0) {
            EyeState state;
            state.center = QPointF(eyeToX, eyeToY);
            moveBothEyes(EyePosition, 30, state, state);
        }
    }

    if (m_publishPixmap) {
        m_leftEye->PublishImage();
        m_rightEye->PublishImage();
    }
}

void MainWindow::playAnimation(EyesAnimation *animation) {
    if (!m_leftEye->isMoving() && !m_rightEye->isMoving()) {
        m_animationStage = 1;
        m_playingAnimation = animation;

        m_restartAutoblink = false;
        if (m_randomBlinkingTimer->isActive()) {
            m_randomBlinkingTimer->stop();
            m_restartAutoblink = true;
        }

        // Saved current states to restore them later
        m_leftSavedState = m_leftEye->getState();
        m_rightSavedState = m_rightEye->getState();

        auto &startLeftState = animation->leftEyeAnimation.initState;
        auto &startRightState = animation->rightEyeAnimation.initState; 
        bool moveWithBlink = true;

        // :AnimationStages
        // Animation Stage 1. Initialize first position with blink motion
        moveBothEyes((MoveFlags)0xFFFF, 30, startLeftState, startRightState, moveWithBlink);
    }
}

void MainWindow::moveBothEyes(MoveFlags flags, EyeAnimation *targetSequenceLeft, EyeAnimation *targetSequenceRight) {
    if (!m_leftEye->isMoving() && !m_rightEye->isMoving()) {
        bool dryRun = true; // Running moves in dry mode, without starting a timer
        bool moveWithBlink = false;
        m_leftEye->move(flags, targetSequenceLeft, moveWithBlink, dryRun);
        m_rightEye->move(flags, targetSequenceRight, moveWithBlink, dryRun);

        if (flags & ~EyePosition && m_blinkTimer->isActive()) {
            // Delay move start until blink's finished
            m_delayedMoveWaiting = true;
        } else {
            // Only now fire up move on both eyes simultaneously
            m_moveTimer->start();
        }
    }
}

void MainWindow::moveBothEyes(MoveFlags flags, int ms, EyeState targetStateLeft, EyeState targetStateRight, bool moveWithBlink) {
    if (!m_leftEye->isMoving() && !m_rightEye->isMoving()) {
        bool dryRun = true; // Running moves in dry mode, without starting a timer
        m_leftEye->move(flags, ms, targetStateLeft, moveWithBlink, dryRun);
        m_rightEye->move(flags, ms, targetStateRight, moveWithBlink, dryRun);

        if (flags & ~EyePosition && m_blinkTimer->isActive()) {
            // Delay move start until blink's finished
            if (moveWithBlink) {
                m_delayedBlinkWaiting = true;
            } else {
                m_delayedMoveWaiting = true;
            }
        } else {
            if (moveWithBlink) {
                m_blinkTimer->start();
            } else {
                // Only now fire up move on both eyes simultaneously
                m_moveTimer->start();
            }
        }
    }
}

void MainWindow::blinkBothEyes(int ms) {
    if (!m_leftEye->isBlinking() && !m_rightEye->isBlinking()) {
        bool dryRun = true; // Running blinks in dry mode, without starting a timer
        m_leftEye->blink(ms, dryRun);
        m_rightEye->blink(ms, dryRun);

        if (m_moveTimer->isActive() || m_animationStage > 0) {
            // Delay blink start until move's finished
            m_delayedBlinkWaiting = true;
        } else {
            // Only now fire up blink on both eyes simultaneously
            m_blinkTimer->start();
        }
    }
}

void MainWindow::randomBlinkingGeneration() {
    m_blinkGenerationTime += m_msBlinkGenerationInterval;

    if (m_blinkGenerationTime >= m_timeUntilNextBlink && !m_blinkTimer->isActive()) {
        m_blinkGenerationTime = 0;

        auto probability = m_uniformDist(m_noiseGenerator);

        m_timeUntilNextBlink = 1000 * floor(boost::math::quantile(m_blinkTimeDistribution, probability));
        m_timeUntilNextBlink = std::max(1000, m_timeUntilNextBlink);

        blinkBothEyes(m_autoBlinkDelay);
    }
}

void MainWindow::updateBlink() {
    if (!m_leftEye->isBlinking() && !m_rightEye->isBlinking()) {
        m_blinkTimer->stop();

        // Run waiting move
        if (m_delayedMoveWaiting) {
            m_delayedMoveWaiting = false;
            m_moveTimer->start();
        }

        // :AnimationStages
        // Animation Stage 2. Continue playing main animation sequence
        if (m_animationStage == 1 && m_playingAnimation != nullptr) {
            m_animationStage = 2;
            moveBothEyes((MoveFlags)0xFFFF, &m_playingAnimation->leftEyeAnimation, &m_playingAnimation->rightEyeAnimation);
        } else if (m_animationStage == 3) {
            m_animationStage = 0; // Stop animation
        }

        if (m_animationStage == 0 && m_restartAutoblink) {
            // Restore autoblink, but not in the middle of animation
            m_randomBlinkingTimer->start();
        }
    }
}

void MainWindow::updateMove() {
    if (!m_leftEye->isMoving() && !m_rightEye->isMoving()) {
        m_moveTimer->stop();

        // Run waiting blink
        if (m_delayedBlinkWaiting && m_animationStage == 0) {
            m_delayedBlinkWaiting = false;
            m_blinkTimer->start();
        }

        // :AnimationStages
        // Animation Stage 3. Finish animation by restoring saved position with blink motion
        if (m_animationStage == 2) {
            m_animationStage = 3;
            bool moveWithBlink = true;
            moveBothEyes((MoveFlags)0xFFFF, 30, m_leftSavedState, m_rightSavedState, moveWithBlink);
        }

        // In the end of animation restore saved position with a blink
        if (m_animationStage == 0 && m_restartAutoblink) {
            m_randomBlinkingTimer->start();
        }
    }
}
