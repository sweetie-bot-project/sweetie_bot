#include "mainwindow.h"

MainWindow::MainWindow(int argc, char *argv[],bool bShowOnDesktop, bool bSendImage, QWidget *parent) : QOpenGLWidget(parent)
{
  setWindowFlags(Qt::FramelessWindowHint);
  if (bShowOnDesktop)  setFixedSize(600,300);
  else setFixedSize(1600,800);
  move(0,0);

  // ConnectWidget - инициализирует ROS, Дает сигнал на подключение для ros_subscriber_widget
  ConnectWidget *pConnectWidget=new ConnectWidget(argc, argv,nullptr);
  // ros_subscriber_widget - Подключается к топикам, получает сообщения, отравляет их на парсер
  ros_subscriber_widget *pSubscriber=new ros_subscriber_widget(this);
  connect(pConnectWidget,SIGNAL(RosStateChanged(int)),pSubscriber,SLOT(RosStateChanged(int)));

  // ros_publisher_widget - Подключается к топикам, отправляет img глаза от eye_widget
  ros_publisher_widget *pPublisher;
  if (bSendImage) pPublisher=new ros_publisher_widget(this);
  if (bSendImage)connect(pConnectWidget,SIGNAL(RosStateChanged(int)), pPublisher,SLOT(RosStateChanged(int)));

  //RosMsgParser - разбирает сообщения, посылает их EyeMoveProcessor
  RosMsgParser* pRosMsgParser=new RosMsgParser(this);
  connect(pSubscriber,SIGNAL(JointStateNewMsg(sensor_msgs::JointState*)),pRosMsgParser,SLOT(NewJointState(sensor_msgs::JointState*)));
  connect(pSubscriber,SIGNAL(TextCommandNewMsg(sweetie_bot_text_msgs::TextCommand*)),pRosMsgParser,SLOT(NewTextCommand(sweetie_bot_text_msgs::TextCommand*)));

  // pLeftEye,pRigthEye  - виджет отрисовки глаз
  pMainLayout=new QHBoxLayout(this);

  pMainLayout->setMargin(0);
  pMainLayout->setSpacing(0);

  this->setLayout(pMainLayout);
  pRigthEye=new EyeWidget(this,false,bShowOnDesktop,bSendImage);
  pLeftEye=new EyeWidget(this,true,bShowOnDesktop,bSendImage);

  //сначала добавляем правый глаз, потом левый.
  pMainLayout->addWidget(pRigthEye);
  pMainLayout->addWidget(pLeftEye);

  connect(pRosMsgParser,SIGNAL(SetNewLidParamCommand(QString*)),pRigthEye,SLOT(NewSetLidParamCommand (QString*)));
  connect(pRosMsgParser,SIGNAL(SetNewLidParamCommand(QString*)),pLeftEye,SLOT(NewSetLidParamCommand (QString*)));

  // EyeMoveProcessor - получает данные от парсера, управляет движением и размером глаз
  pEyeMoveProcessor=new EyeAnimationManager(this);

  connect(pRosMsgParser,SIGNAL(SetNewPitch(double)),pEyeMoveProcessor,SLOT(NewPitch(double)));
  connect(pRosMsgParser,SIGNAL(SetNewYaw(double)),pEyeMoveProcessor,SLOT(NewYaw(double)));
  connect(pRosMsgParser,SIGNAL(SetNewParamCommand(QString*)),pEyeMoveProcessor,SLOT(NewSetParamCommand (QString*)));
  connect(pRosMsgParser,SIGNAL(SetNewAnimationMode(QString*)),pEyeMoveProcessor,SLOT(NewAnimationMode (QString*)));
  connect(pRosMsgParser,SIGNAL(SetNewEmotionCommand(QString*)),pEyeMoveProcessor,SLOT(NewEmotionCommand (QString*)));

  // - подключаем левый глаз к аниматору
  connect(pEyeMoveProcessor,SIGNAL(EyeParamSend(EyeParametrStruct*)),pLeftEye,SLOT(SetEyePeram(EyeParametrStruct*)));
  connect(pEyeMoveProcessor,SIGNAL(EyeFigureName(QString*)),pLeftEye,SLOT(SetEyeName(QString*)));
  connect(pEyeMoveProcessor,SIGNAL(EyeLeftLidStateSend(LidDisplayStateStruct*)),pLeftEye,SLOT(SetLidState(LidDisplayStateStruct*)));
  // - подключаем правый глаз к аниматору
  connect(pEyeMoveProcessor,SIGNAL(EyeParamSend(EyeParametrStruct*)),pRigthEye,SLOT(SetEyePeram(EyeParametrStruct*)));
  connect(pEyeMoveProcessor,SIGNAL(EyeFigureName(QString*)),pRigthEye,SLOT(SetEyeName(QString*)));
  connect(pEyeMoveProcessor,SIGNAL(EyeRigthLidStateSend(LidDisplayStateStruct*)),pRigthEye,SLOT(SetLidState(LidDisplayStateStruct*)));

  // Тактовый сигнал для отрисовки кадров
  pTimer= new QTimer(this);
  connect(pTimer, SIGNAL(timeout()), pEyeMoveProcessor, SLOT(timer_clk()));

  connect(pTimer, SIGNAL(timeout()), pLeftEye, SLOT(timer_clk()));
  connect(pTimer, SIGNAL(timeout()), pRigthEye, SLOT(timer_clk()));
 // connect(pTimer, SIGNAL(timeout()), this, SLOT(timer_clk()));

  //Изображения глаз отправляются в топик eye_image_left/eye_image_rigth
  if (bSendImage) connect(pLeftEye,SIGNAL(SendEyeImage(QImage*)),pPublisher,SLOT(PublishImageLeft(QImage*)));
  if (bSendImage) connect(pRigthEye,SIGNAL(SendEyeImage(QImage*)),pPublisher,SLOT(PublishImageRigth(QImage*)));

  pTimer->start(33);
}

MainWindow::~MainWindow()
{
  delete pTimer;
}
/*
void MainWindow::initializeGL()
{
  //this->makeCurrent();
  // initializeOpenGLFunctions();
}

void MainWindow::paintGL()
{

  //glClearColor(100,0,100,50);
}
*/
/*
void MainWindow::paintEvent(QPaintEvent *) {
   QPainter painter;//(this);

   painter.begin(this);
   painter.setRenderHint(QPainter::Antialiasing);

    painter.setPen(QPen(QColor(0, 255, 0)));
    painter.setBrush(QColor(QColor(0, 0, 255)));
    painter.fillRect(1,1,50,50,QColor(0, 0, 255));

   painter.end();
}
*/

void MainWindow::timer_clk()
{/*
uiTimeCounter++;
if (uiTimeCounter>10)
{
  uiTimeCounter=0;
  if (bTempFalg) bTempFalg=false;
  else bTempFalg=true;
}
    this->repaint();*/
}
//-------- Для перемещения окна мышкой----------------
void MainWindow::mousePressEvent(QMouseEvent *event)
{
  if (event->button() == Qt::LeftButton) {
    m_mousePoint = event->pos();
    event->accept();
  }
}
void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
  if (event->buttons() == Qt::LeftButton)  {
    const QPointF delta = event->globalPos() - m_mousePoint;
    move(delta.toPoint());
    event->accept();
  }
}
//---------------------------------------------------------------


