#ifndef ROS_SUBSCRIBER_WIDGET_H
#define ROS_SUBSCRIBER_WIDGET_H

#include <QWidget>
#include <QLabel>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace Ui {
class ros_subscriber_widget;

}

class ros_subscriber_widget : public QWidget
{
  Q_OBJECT
public:
  explicit ros_subscriber_widget(QWidget *parent = nullptr);
  ~ros_subscriber_widget();
public slots:
  void RosStateChanged(int iState);
signals:
  void GetNewImage(QImage* pImage);
private:
  int iRosSate=0;
  int iImageCount=0;
  ros::NodeHandle *rx_node;
  ros::Subscriber img_sub;
  void RxCallback(const sensor_msgs::ImageConstPtr &msg);
  Ui::ros_subscriber_widget *ui;
};

#endif // ROS_SUBSCRIBER_WIDGET_H
