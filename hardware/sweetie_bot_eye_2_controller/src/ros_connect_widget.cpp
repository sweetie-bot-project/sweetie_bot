#include "ros_connect_widget.h"
#include "ui_ros_connect_widget.h"

ConnectWidget::ConnectWidget(int argc, char *argv[],QWidget *parent) :
  QWidget(parent),
  ui(new Ui::ConnectWidget)
{
  ui->setupUi(this);
  sState="Disconnect";
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(timer_clk()));
  timer->start(100);
  ros::init(argc, argv, "eye_2_controller");
  ros::start();

  eStep=WAITE_ROS_INIT;
  SetIndicatorColor(ui->lbRosIsOk,IC_RED);
  SetIndicatorColor(ui->lbRosIsInit,IC_RED);
  SetIndicatorColor(ui->lbRosMasterCheck,IC_RED);
  // eStep=START_ROS_INIT;
}

//void ConnectWidget::RosInit()
//{
//  int argc=1;
//  QString s="fasfasfasf";
//  char *argv;
//  argv=s.toUtf8().data();
//  ros::init(argc, &argv, "motor_state_viewer");
//}

ConnectWidget::~ConnectWidget()
{
  delete ui;
}

void ConnectWidget::Processing()
{
  int iRosErrorFlag=0;
  switch (eStep)
  {
  //  case START_ROS_INIT:   // запускаем инициализацию ROS
  //    sState="running ROS initialization";
  //    RosInit();
  //    eStep=WAITE_ROS_INIT;
  //    break;
  case WAITE_ROS_INIT:   // проверяем инициализацию ROS
    sState="waiting ROS initialization";
    if (ros::isInitialized() & ros::ok())
    {
      eStep=WAITE_MASTER_NODE;
    }
    break;
  case WAITE_MASTER_NODE:   // проверяем наличие мастер ноды
    if (ros::master::check()) eStep=CREATE_NODE;
    else sState="master node error";
    break;
  case CREATE_NODE: // можно создавать свои ноды
    sState="Node created";
    emit RosStateChanged(0);
    eStep=MONITOR;
    break;
  case MONITOR: // мониторим состояние ROS
    sState="OK";
    if (!ros::ok())iRosErrorFlag=1; //здесь "no OK" может говорить об отсутствии подключенных нод. иногда не является ошибкой ROS
    if (!ros::isInitialized()) iRosErrorFlag=2;
    if (!ros::master::check()) iRosErrorFlag=3;
    if (iRosErrorFlag!=0)
    {
      //eStep=START_ROS_INIT;
      eStep=WAITE_ROS_INIT;
      emit RosStateChanged(iRosErrorFlag);
    }
    ros::spinOnce();
    break;
  }
  ui->lbState->setText(sState);
}

void ConnectWidget::RosMonitor()
{
  if (ros::isInitialized())SetIndicatorColor(ui->lbRosIsInit,IC_GREEN);
  else SetIndicatorColor(ui->lbRosIsInit,IC_RED);
  if(ros::ok()) SetIndicatorColor (ui->lbRosIsOk,IC_GREEN);
  else SetIndicatorColor (ui->lbRosIsOk,IC_RED); //ok() becomes false once ros::shutdown() has been called and is finished
  if (ros::master::check()) SetIndicatorColor(ui->lbRosMasterCheck,IC_GREEN);
  else SetIndicatorColor(ui->lbRosMasterCheck,IC_RED);
  ui->lbMasterURL->setText(QString::fromStdString(ros::master::getURI()));
}

void ConnectWidget::SetIndicatorColor(QLabel *lb,eIndicatorColors eColor)
{
  lb->setFixedSize(20,20);
  switch (eColor)
  {
  case IC_RED:
    lb->setStyleSheet( "border-radius: 10px; background-color: red;" );
    break;
  case IC_GREEN:
    lb->setStyleSheet( "border-radius: 10px; background-color: green;" );
    break;
  }
}

void ConnectWidget::timer_clk()
{
  RosMonitor();
  Processing();
}

