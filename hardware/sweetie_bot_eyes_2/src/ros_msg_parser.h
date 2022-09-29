#ifndef ROS_MSG_PARSER_H
#define ROS_MSG_PARSER_H

#include <QWidget>
// ROS
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <sweetie_bot_text_msgs/TextCommand.h>

class RosMsgParser : public QWidget
{
  Q_OBJECT
public:
  explicit RosMsgParser(QWidget *parent = nullptr);
private:
  QPoint point;

signals:
  void SetNewPitch(double pitch);
  void SetNewYaw(double yaw);
  void SetNewParamCommand(QString* sCommand);
  void SetNewAnimationMode(QString* sAnimName);
  void SetNewEffect(QString* sEffectName);
  void SetNewLidParamCommand(QString* sCommand);
  void SetNewEmotionCommand(QString* sCommand);

public slots:
  void NewTextCommand(sweetie_bot_text_msgs::TextCommand* msg);
  void NewJointState(sensor_msgs::JointState* msg);
};

#endif // ROS_MSG_PARSER_H
