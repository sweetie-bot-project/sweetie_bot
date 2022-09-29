#ifndef ROS_PUBLISHER_H
#define ROS_PUBLISHER_H

#include <QWidget>
#include <QImage>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

//#include <sensor_msgs/JointState.h>
//#include <sweetie_bot_text_msgs/TextCommand.h>


class ros_publisher_widget : public QWidget
{
  Q_OBJECT

public:
  explicit ros_publisher_widget(QWidget *parent = nullptr);
  ~ros_publisher_widget();
public slots:
  void RosStateChanged(int iState);
  void PublishImageLeft(QImage* pImage);
  void PublishImageRigth(QImage* pImage);
private:

  ros::NodeHandle *tx_node;
  ros::Publisher left_eye_image_pub;
  ros::Publisher rigth_eye_image_pub;


};

#endif // ROS_PUBLISHER_H
