#ifndef ROS_SUBSCRIBER_H
#define ROS_SUBSCRIBER_H

#include <QWidget>

// ROS
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <sweetie_bot_text_msgs/TextCommand.h>


class ros_subscriber_widget : public QWidget
{
  Q_OBJECT
public:
  explicit ros_subscriber_widget(QWidget *parent = nullptr);
  ~ros_subscriber_widget();
public slots:
  void RosStateChanged(int iState);
signals:
  void TextCommandNewMsg(sweetie_bot_text_msgs::TextCommand* msg);
  void JointStateNewMsg(sensor_msgs::JointState* msg);
private:
  ros::NodeHandle *rx_node=nullptr;
  ros::Subscriber sub_text_command, sub_joint_state;

  sweetie_bot_text_msgs::TextCommand text_command_msgs;
  sensor_msgs::JointState joint_state_msgs;


  void TextCommandCallback(const sweetie_bot_text_msgs::TextCommand::ConstPtr& msgs);
  void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msgs);

};

#endif // ROS_SUBSCRIBER_H
