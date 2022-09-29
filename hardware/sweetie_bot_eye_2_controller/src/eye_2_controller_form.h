#ifndef EYE_2_CONTROLLER_FORM_H
#define EYE_2_CONTROLLER_FORM_H

#include <QMainWindow>
#include <QTimer>
#include <QGraphicsView>
#include <QGraphicsPixmapItem>

#include "ros_connect_widget.h"
#include "ros_subscriber_widget.h"
#include "ros_publisher_widget.h"
#include "mouse_eye_mover.h"

#include "lid_manual_control.h"

// ROS
#include <ros/ros.h>

#include "ui_eye_2_controller_form.h"


namespace Ui {
class Eye2ControllerForm;
}

class Eye2ControllerForm : public QMainWindow
{
  Q_OBJECT
public:
  explicit Eye2ControllerForm(int argc, char *argv[],  QWidget *parent = 0);
  void bootstrap();
  ~Eye2ControllerForm();

signals:
  void SendCommand(std::string sType, std::string sCommand);
public slots:
  void logAddString(QString s);
  void GetEyeImage(QImage* pImage);


private:

  bool bTimerElapsedFlag;
  QStringList slLog;
  QStringList slJointList;
  Ui::Eye2ControllerForm ui;
  QGraphicsScene qGScene;

  QTimer * timer;
  QGraphicsPixmapItem *pixmap_item;

  LidManualControl *pUpLidControl;
  LidManualControl *pDownLidControl;
private slots:

  void EyePos(QPointF pos);
  void timer_clk();
  void on_horizontalSlider_sliderMoved(int position);
  void on_pushButton_3_clicked();
  void on_horizontalSlider_valueChanged(int value);
  void on_horizontalSlider_2_valueChanged(int value);
  void on_pushButton_18_clicked();
  void on_pushButton_19_clicked();
  void on_pushButton_17_clicked();
  void on_pbAnimNormal_clicked();
  void on_pbAnimSpin_clicked();
  void on_pbAnimShake_clicked();
  void on_pbBlink_clicked();
  void on_pbCloseEye_clicked();
  void on_pushButton_4_clicked();
  void on_pbEmNormal_clicked();
  void on_pbEmShy_clicked();
  void on_pbEmAngry_clicked();
  void on_pbEmTired_clicked();
  void on_pbEmSuspect_clicked();
  void on_hsCloseSlider_valueChanged(int value);
  void on_pbAnimNormalSize_clicked();
  void on_pbLidNormal_clicked();
  void on_pushButton_2_clicked();
};


#endif // EYE_2_CONTROLLER_FORM_H
