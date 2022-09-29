#include "ros_publisher_widget.h"
#include "ui_ros_publisher_widget.h"

ros_publisher_widget::ros_publisher_widget(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::ros_publisher_widget)
{
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(timer_clk()));
  timer->start(100);
  ui->setupUi(this);

  tx_node=nullptr;
}

ros_publisher_widget::~ros_publisher_widget()
{
  delete ui;
  delete tx_node;
}
void ros_publisher_widget ::RosStateChanged(int iState)
{
  iRosSate=iState;
  if (iRosSate==0)
  {
    ui->lbState->setText("online");
    if (tx_node==nullptr)  tx_node=new ros::NodeHandle ;
    servo_commands_pub = tx_node->advertise<sweetie_bot_text_msgs::TextCommand>("control", 1);
  }
  else
  {
    ui->lbState->setText("offline ");// +QString::number(iState));
    if (tx_node!=nullptr)
    {
      servo_commands_pub.shutdown();
      delete tx_node;
      tx_node=nullptr;
    }
  }
}

void ros_publisher_widget::timer_clk(){ros::spinOnce();}

void ros_publisher_widget::PublishCommand(std::string sType, std::string sCommand)
{
  if (tx_node==nullptr) return; // нода не создана
  my_msgs.type=sType;
  my_msgs.command=sCommand;
  servo_commands_pub.publish(my_msgs);
  iCount++;
  ui->lbCounter->setText(QString::number(iCount));
}

/*
void ros_publisher_widget::on_pbSendMsg_1_clicked()
{
  iCount++;
  QString s="leg2_joint1";
 // QTextStream(&s)<<"Count="<<iCount;
  //my_msgs.name.push_back("leg1_joint1");
   my_msgs.name.push_back(s.toStdString());
  my_msgs.pos.push_back(iCount);
  my_msgs.pwm.push_back(iCount);
  my_msgs.vel.push_back(iCount);
  my_msgs.pos_goal.push_back(iCount);
  my_msgs.pos_desired.push_back(iCount);
  my_msgs.vel_desired.push_back(iCount);
  //NO_MOVING = 0x00, NO_ERROR_TEMPERATURE=0
  my_msgs.status_detail.push_back(0xff);
  my_msgs.status_error.push_back(0xff);
  test_pub.publish(my_msgs);
  ui->lbCounter->setText(QString::number(iCount));
}

void ros_publisher_widget::on_pbRunScript_clicked()
{
  QProcess *process = new QProcess;
  QStringList arg;
  arg<<"/home/celestia/msgs_data_records/play_msgs.sh";
  process->start("/bin/bash",arg);
}

void ros_publisher_widget::on_pbSendMsg_2_clicked()
{
  iCount++;
  QString s="leg2_joint1";
  my_msgs.name.push_back(s.toStdString());
  my_msgs.pos.push_back(iCount);
  my_msgs.pwm.push_back(iCount);
  my_msgs.vel.push_back(iCount);
  my_msgs.pos_goal.push_back(iCount);
  my_msgs.pos_desired.push_back(iCount);
  my_msgs.vel_desired.push_back(iCount);
  //MOVING = 0x01, ERROR_TEMPERATURE = 0x04,
 // uint16_t uiStatus=0x0104;
  my_msgs.status_detail.push_back(0x01);
  my_msgs.status_error.push_back(0x04);
  test_pub.publish(my_msgs);
  ui->lbCounter->setText(QString::number(iCount));
}

void ros_publisher_widget::on_pbSendMsg_3_clicked()
{
  iCount++;
  QString s="leg2_joint1";
  my_msgs.name.push_back(s.toStdString());
  my_msgs.pos.push_back(iCount);
  my_msgs.pwm.push_back(iCount);
  my_msgs.vel.push_back(iCount);
  my_msgs.pos_goal.push_back(iCount);
  my_msgs.pos_desired.push_back(iCount);
  my_msgs.vel_desired.push_back(iCount);
  //MOVING = 0x01 + MOTOR_ON = 0x40,, ERROR_OVERLOAD = 0x10
  //uint16_t uiStatus=0x4110;
  my_msgs.status_detail.push_back(0x41);
  my_msgs.status_error.push_back(0x10);
  test_pub.publish(my_msgs);
  ui->lbCounter->setText(QString::number(iCount));
}
*/
