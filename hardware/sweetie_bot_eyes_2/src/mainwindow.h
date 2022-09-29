#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QOpenGLWidget>
#include <QTimer>
#include <QKeyEvent>
#include <QHBoxLayout>
//#include <QOpenGLFunctions>

#include "my_types.h"
#include "ros_connector.h"
#include "ros_subscriber.h"
#include "ros_publisher.h"
#include "eye_animation_manager.h"
#include "ros_msg_parser.h"
#include "eye_widget.h"

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
//#include <sweetie_bot_text_msgs/TextCommand.h>


class MainWindow : public QOpenGLWidget, protected QOpenGLFunctions{
  Q_OBJECT

private:
  //  void repaint(void);
  bool bTempFalg;
  // QGraphicsPixmapItem* pItem;

  EyeWidget* pLeftEye;
  EyeWidget* pRigthEye;

  EyeAnimationManager* pEyeMoveProcessor;

  QTimer* pTimer;
  uint uiTimeCounter=0;

  QHBoxLayout *pMainLayout;

  void mouseMoveEvent(QMouseEvent *event);
  void mousePressEvent(QMouseEvent *event);
  QPoint m_mousePoint;

public:
  MainWindow(int argc, char *argv[],bool bShowOnDesktop,bool bSendImage, QWidget *parent = nullptr);
  ~MainWindow();
  //void initializeGL();
  // void paintGL();

signals:



private slots:
  void timer_clk();

};

#endif // MAINWINDOW_H
