#ifndef CONNECT_FORM_H
#define CONNECT_FORM_H

#include <QWidget>

#include <QLabel>
#include <QTimer>
#include <QTextStream>

// ROS
#include <ros/ros.h>

namespace Ui {
class ConnectWidget;
}
enum eIndicatorColors
{
  IC_RED,
  IC_GREEN,
};

enum eRosInitState
{
  START_ROS_INIT,
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
  void SetIndicatorColor(QLabel *lb,eIndicatorColors eColor);
  void RosInit();
  eRosInitState eStep;
  QString sState;
  QTimer * timer;

signals:
  void RosStateChanged(int i);

private:
  Ui::ConnectWidget *ui;
};

#endif // CONNECT_FORM_H
