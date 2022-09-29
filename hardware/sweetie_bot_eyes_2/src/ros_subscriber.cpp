#include "ros_subscriber.h"

ros_subscriber_widget::ros_subscriber_widget(QWidget *parent) :
  QWidget(parent)
{
  // ui->setupUi(this);
}

ros_subscriber_widget::~ros_subscriber_widget()
{
  // delete ui;
}

void ros_subscriber_widget ::RosStateChanged(int iState)
{
  //  iRosSate=iState;
  if (iState==0)
  {
    //ui->lbState->setText("online");
    if (rx_node==nullptr)  rx_node=new ros::NodeHandle ;
    sub_text_command = rx_node->subscribe<sweetie_bot_text_msgs::TextCommand>("control", 10, &ros_subscriber_widget::TextCommandCallback, this,ros::TransportHints().tcpNoDelay());
    sub_joint_state = rx_node->subscribe<sensor_msgs::JointState>("joint_states", 10, &ros_subscriber_widget::JointStateCallback, this, ros::TransportHints().tcpNoDelay());
    // sc_sub = rx_node->subscribe<sweetie_bot_herkulex_msgs::ServoCommands>("/motion/herkulex/servo_comands", 10000, &ros_subscriber_widget::ScCallback, this);
  }
  else
  {
    //   ui->lbState->setText("offline ");// +QString::number(iState));
    if (rx_node!=nullptr)
    {
      sub_text_command.shutdown();
      sub_joint_state.shutdown();
      delete rx_node;
      rx_node=nullptr;
    }
  }
}

void ros_subscriber_widget::TextCommandCallback(const sweetie_bot_text_msgs::TextCommand::ConstPtr& msg)
{
  //  unsigned long iMsgSize=0;
  text_command_msgs=*msg;
  // i_HJS_Count++;
  // iMsgSize=hjs_msgs.name.size();
  // ui->lbHjsMsgCount->setText(QString::number(i_HJS_Count));
  emit TextCommandNewMsg(&text_command_msgs);
}
/*
void ros_subscriber_widget::ScCallback(const sweetie_bot_herkulex_msgs::ServoCommands::ConstPtr& msgs)
{
  sc_msgs=*msgs;
  std::string sName;
  uint8_t uiCommand;
  sName=sc_msgs.name.at(0);
  uiCommand=sc_msgs.command;
  QString s;
  s="Servo "+QString::fromStdString(sName)+" Get command " +QString::number(uiCommand);
  emit  logString(s);

  i_SC_Count++;
  ui->lbScMsgCount->setText(QString::number(i_SC_Count));
}
*/
void ros_subscriber_widget::JointStateCallback(const sensor_msgs::JointState::ConstPtr& msgs)
{
  // unsigned long iMsgSize=0;
  joint_state_msgs=*msgs;
  //  i_HS_Count++;
  //  iMsgSize=hs_msgs.name.size();
  //  ui->lbHsMsgCount->setText(QString::number(i_HS_Count));
  emit JointStateNewMsg(&joint_state_msgs);
}

