#ifndef ROS_PUBLISHER_WIDGET_H
#define ROS_PUBLISHER_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <QTextStream>
#include <QProcess>
// ROS
#include <ros/ros.h>

//#include <sensor_msgs/JointState.h>
#include <sweetie_bot_text_msgs/TextCommand.h>

namespace Ui {
class ros_publisher_widget;
}

class ros_publisher_widget : public QWidget
{
  Q_OBJECT

public:
  explicit ros_publisher_widget(QWidget *parent = nullptr);
  ~ros_publisher_widget();
public slots:
  void RosStateChanged(int iState);
  void PublishCommand(std::string sType, std::string sCommand);
private slots:
  void timer_clk();
private:
  int iRosSate=0;
  int iCount=0;
  QTimer * timer;
  ros::NodeHandle *tx_node;
  ros::Publisher servo_commands_pub;
  sweetie_bot_text_msgs::TextCommand my_msgs;
  Ui::ros_publisher_widget *ui;
};

#endif // ROS_PUBLISHER_WIDGET_H
