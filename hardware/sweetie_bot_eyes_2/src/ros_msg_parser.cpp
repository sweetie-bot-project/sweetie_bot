/* Парсер получает сообщения
 * sensor_msgs::JointState      положениt робота и глаз
 * sweetie_bot_text_msgs::TextCommand   текстовые команды
 *
 * В соощениях ищутся данные касающиеся глаз и отправляются
 * через сигналы
 *     SetNewPitch(double ) - положения робота
       SetNewYaw(double )   - положения робота
       SetNewParamCommand(QString* )  - команда типа eyes/set_param
       SetNewAnimationMode(QString* ) - команда типа eyes/animation_mode
       SetNewEffect(QString* )        - команда типа eyes/effect
 * */


#include "ros_msg_parser.h"

RosMsgParser::RosMsgParser(QWidget *parent) : QWidget(parent)
{

}

/*NewJointState - обрабатывает сообщение от JointStatePublisher
 * Генерирует сигналы SetNewPitch и SetNewYaw * */

void RosMsgParser::NewJointState(sensor_msgs::JointState* msg)
{
  auto pos = std::find(msg->name.begin(), msg->name.end(), "eyes_pitch");
  if(pos != msg->name.end()) {
    unsigned long n = std::distance(msg->name.begin(), pos);
    if(msg->position.size() > n)
      emit SetNewPitch(msg->position[n]);
  }

  pos = std::find(msg->name.begin(), msg->name.end(), "eyes_yaw");
  if(pos != msg->name.end()) {
    unsigned long n = std::distance(msg->name.begin(), pos);
    if(msg->position.size() > n)
      emit SetNewYaw(msg->position[n]);
  }
}
constexpr unsigned int str2hash(const char* str, int h = 0)
{
  return !str[h] ? 5381 : (str2hash(str, h+1)*33) ^ str[h];
}
/*NewTextCommand - обрабатывает тектовые сообщения
 * ищет по типу команды для глаза и отправляет их через сигналы */
void RosMsgParser::NewTextCommand(sweetie_bot_text_msgs::TextCommand* msg)
{
  QString sCommand;

  switch(str2hash(msg->type.c_str()))
  {
  case str2hash("eyes/set_param"):
    sCommand=QString::fromStdString(msg->command);
    emit SetNewParamCommand(&sCommand);
    break;
  case str2hash("eyes/animation_mode"):
    sCommand=QString::fromStdString(msg->command);
    emit SetNewAnimationMode(&sCommand);
    break;
  case str2hash("eyes/effect"):
    sCommand=QString::fromStdString(msg->command);
    emit SetNewEffect(&sCommand);
    break;
  case str2hash("eyes/emotion"):
    sCommand=QString::fromStdString(msg->command);
    emit SetNewEmotionCommand(&sCommand);
    break;
  case str2hash("eyes/set_lid_param"):
    sCommand=QString::fromStdString(msg->command);
    emit SetNewLidParamCommand(&sCommand);
    break;
  }

  // TODO:   Сделать поддержку Legasy команд
  /*
eyes/action --- perform scripted action and return to previous state. ("blink", "slow_blink")

eyes/emotion --- change eyes state ("normal", "red_eyes", "sad_look", "evil_look")
*/

}
