#include "ros_subscriber_widget.h"
#include "ui_ros_subscriber_widget.h"

ros_subscriber_widget::ros_subscriber_widget(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::ros_subscriber_widget)
{
  ui->setupUi(this);
  rx_node=nullptr;
}

ros_subscriber_widget::~ros_subscriber_widget()
{
  delete ui;
}

void ros_subscriber_widget ::RosStateChanged(int iState)
{
  iRosSate=iState;
  if (iRosSate==0)
  {
    ui->lbState->setText("online");
    if (rx_node==nullptr)  rx_node=new ros::NodeHandle ;
    img_sub = rx_node->subscribe<sensor_msgs::Image>("eye_image_left", 1, &ros_subscriber_widget::RxCallback, this);
  }
  else
  {
    ui->lbState->setText("offline ");
    if (rx_node!=nullptr)
    {
      img_sub.shutdown();
      delete rx_node;
      rx_node=nullptr;
    }
  }
}

void ros_subscriber_widget::RxCallback(const sensor_msgs::ImageConstPtr &msg)
{
  QImage myImage(&msg->data[0], msg->width, msg->height, QImage::Format_RGBA8888);
  iImageCount++;
  ui->lbImageCounter->setText(QString::number(iImageCount));
  emit GetNewImage(&myImage);
}
