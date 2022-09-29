#include "ros_publisher.h"

ros_publisher_widget::ros_publisher_widget(QWidget *parent) :
  QWidget(parent)
{
  tx_node=nullptr;
}

ros_publisher_widget::~ros_publisher_widget()
{
  delete tx_node;
}
void ros_publisher_widget ::RosStateChanged(int iState)
{
  if (iState==0)
  {
    if (tx_node==nullptr)  tx_node=new ros::NodeHandle ;
    left_eye_image_pub = tx_node->advertise<sensor_msgs::Image>("eye_image_left", 1);
    rigth_eye_image_pub = tx_node->advertise<sensor_msgs::Image>("eye_image_right", 1);
  }
  else
  {
    if (tx_node!=nullptr)
    {
      left_eye_image_pub.shutdown();
      rigth_eye_image_pub.shutdown();
      delete tx_node;
      tx_node=nullptr;
    }
  }
}

void ros_publisher_widget::PublishImageLeft(QImage *pImage)
{
  if (tx_node==nullptr) return; // нода не создана
  sensor_msgs::Image img;
  img.header.stamp = ros::Time::now();
  img.width = pImage->width();
  img.height = pImage->height();
  img.encoding = "rgba8";
  img.step = pImage->bytesPerLine();
  //ROS_INFO("format=%d bytesPerLine=%d", int(image.format()), image.bytesPerLine());
  img.data = std::vector<unsigned char>(pImage->bits(), pImage->bits() + pImage->byteCount());
  left_eye_image_pub.publish(img);
}
void ros_publisher_widget::PublishImageRigth(QImage *pImage)
{
  if (tx_node==nullptr) return; // нода не создана
  sensor_msgs::Image img;
  img.header.stamp = ros::Time::now();
  img.width = pImage->width();
  img.height = pImage->height();
  img.encoding = "rgba8";
  img.step = pImage->bytesPerLine();
  //ROS_INFO("format=%d bytesPerLine=%d", int(image.format()), image.bytesPerLine());
  img.data = std::vector<unsigned char>(pImage->bits(), pImage->bits() + pImage->byteCount());
  rigth_eye_image_pub.publish(img);
}
