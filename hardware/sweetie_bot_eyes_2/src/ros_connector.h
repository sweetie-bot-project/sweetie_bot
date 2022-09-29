#ifndef CONNECT_FORM_H
#define CONNECT_FORM_H

#include <QWidget>

#include <QTimer>
#include <QTextStream>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

#include "my_types.h"
/*
namespace Ui {
class ConnectWidget;
}
*/

#define CW_TEXT_MSG_SLOT 1
enum eRosInitState
{
  //  START_ROS_INIT,
  WAITE_ROS_INIT,
  WAITE_MASTER_NODE,
  CREATE_NODE,
  MONITOR,
};


class ConnectWidget : public QWidget
{
  Q_OBJECT

public:
  explicit ConnectWidget(int argc, char *argv[], QWidget *parent = nullptr);
  ~ConnectWidget();
  QString GetState(){return sState;}

private slots:
  void timer_clk();

private:
  void Processing();
  void RosMonitor();
  void PrintMsg(const QString* s);
  //void SetIndicatorColor(QLabel *lb,eIndicatorColors eColor);
  //  void RosInit();
  eRosInitState eStep;
  QString sState;
  QTimer* pTimer;

signals:
  void RosStateChanged(int i);

};

#endif // CONNECT_FORM_H
