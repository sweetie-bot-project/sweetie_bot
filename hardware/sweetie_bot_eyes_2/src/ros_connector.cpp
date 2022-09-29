#include "ros_connector.h"

ConnectWidget::ConnectWidget(int argc, char *argv[],QWidget *parent) :
  QWidget(parent)
{
  sState="Disconnect";
  pTimer = new QTimer(this);
  connect(pTimer, SIGNAL(timeout()), this, SLOT(timer_clk()));
  pTimer->start(100);
  ros::init(argc, argv, "eye_2");
  ros::start(); // Программа не запустится, пока не подключится к roscore,
  // но зато будет работать выход по Ctrl+C
  eStep=WAITE_ROS_INIT;
  QString s="ROS started";
  PrintMsg(&s);
  //eStep=START_ROS_INIT;
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
  delete pTimer;
}

void ConnectWidget::Processing()
{
  int iRosErrorFlag=0;
  QString sMsg;
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
      sMsg="Waiting ROS initialization";
      PrintMsg(&sMsg);
    }
    break;
  case WAITE_MASTER_NODE:   // проверяем наличие мастер ноды
    if (ros::master::check())
    {
      eStep=CREATE_NODE;
      sMsg="ROS master conected";
      PrintMsg(&sMsg);
    }
    else sState="master node error";
    break;
  case CREATE_NODE: // можно создавать свои ноды
    sState="Node created";
    emit RosStateChanged(0);
    sMsg="ROS URL:"+QString::fromStdString(ros::master::getURI());
    PrintMsg(&sMsg);
    eStep=MONITOR;
    break;
  case MONITOR: // мониторим состояние ROS
    sState="ROS is OK";
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
}

void ConnectWidget::RosMonitor()
{
  /*
  if(ros::ok()) SetIndicatorColor (ui->lbRosIsOk,IC_GREEN);
       else SetIndicatorColor (ui->lbRosIsOk,IC_RED); //ok() becomes false once ros::shutdown() has been called and is finished
  if (ros::isInitialized())SetIndicatorColor(ui->lbRosIsInit,IC_GREEN);
    else SetIndicatorColor(ui->lbRosIsInit,IC_RED);
  if (ros::master::check()) SetIndicatorColor(ui->lbRosMasterCheck,IC_GREEN);
    else SetIndicatorColor(ui->lbRosMasterCheck,IC_RED);
  ui->lbMasterURL->setText(QString::fromStdString(ros::master::getURI()));
*/
}

void ConnectWidget::timer_clk()
{
  RosMonitor();
  Processing();
}

void ConnectWidget::PrintMsg(const QString* s)
{
  std::cout << s->toStdString()<<  std::endl;
}
