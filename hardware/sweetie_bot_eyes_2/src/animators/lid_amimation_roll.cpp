#include "lid_amimation_roll.h"

LidAmimationRoll::LidAmimationRoll()
{
  pRollBlinkAnim=new QVariantAnimation;
  pRollBlinkAnim->setStartValue((float)0);
  pRollBlinkAnim->setKeyValueAt((qreal)ERB_CLOSURE_TIME_1/ERB_EFFECT_DURATION,(float)1);
  pRollBlinkAnim->setKeyValueAt((qreal)ERB_CLOSE_TIME_1/ERB_EFFECT_DURATION,(float)1);
  pRollBlinkAnim->setKeyValueAt((qreal)ERB_OPENING_TIME_1/ERB_EFFECT_DURATION,(float)0);

  pRollBlinkAnim->setKeyValueAt((qreal)ERB_WATE_ROLL/ERB_EFFECT_DURATION,(float)0);

  pRollBlinkAnim->setKeyValueAt((qreal)ERB_CLOSURE_TIME_2/ERB_EFFECT_DURATION,(float)1);
  pRollBlinkAnim->setKeyValueAt((qreal)ERB_CLOSE_TIME_2/ERB_EFFECT_DURATION,(float)1);
  pRollBlinkAnim->setKeyValueAt((qreal)ERB_OPENING_TIME_2/ERB_EFFECT_DURATION,(float)0);

  pRollBlinkAnim->setEndValue((float)0);
  pRollBlinkAnim->setDuration(ERB_EFFECT_DURATION);
}

void LidAmimationRoll::SetLidData(EyeLidDataStruct *pEyeLidData)
{
  pLidData=pEyeLidData;
}

bool LidAmimationRoll::Process()
{
  bool bChangeFlag=false; // флаг показывает были ли изменения

  if (iRollBlinkStepCounter<ERB_EFFECT_DURATION) // анимация не завршена
  {
    iRollBlinkStepCounter++;

    if (bNeedStop)// если пришла команда стоп
    {
      if ((iRollBlinkStepCounter>ERB_CLOSURE_TIME_1) & (iRollBlinkStepCounter<ERB_CLOSE_TIME_1))// глаз закрыт 1 раз
      {
        iRollBlinkStepCounter=ERB_CLOSE_TIME_2; // переходим к завершающиму открытию
      }
      if ((iRollBlinkStepCounter>ERB_CLOSURE_TIME_2) & (iRollBlinkStepCounter<ERB_CLOSE_TIME_2))// глаз закрыт 2 раз
      {
        iRollBlinkStepCounter=ERB_CLOSE_TIME_2; // переходим к завершающиму открытию
      }
      if ((iRollBlinkStepCounter>ERB_OPENING_TIME_1) & (iRollBlinkStepCounter<ERB_WATE_ROLL))// глаз открыт
      {
        iRollBlinkStepCounter=ERB_OPENING_TIME_2; // сразу конец анимации
      }
    }


  /*  if ((iBlinkStepCounter>EEB_CLOSURE_TIME) & (iBlinkStepCounter<(EEB_CLOSURE_TIME+EEB_CLOSE_TIME)))
    {
      if (bNeedStop) iBlinkStepCounter=(EEB_CLOSURE_TIME+EEB_CLOSE_TIME);
    }
 */   pRollBlinkAnim->setCurrentTime(iRollBlinkStepCounter);
    pLidData->sLeftLidState.fClosePercent=pRollBlinkAnim->currentValue().toFloat();
    pLidData->sRigthLidState.fClosePercent=pLidData->sLeftLidState.fClosePercent;
    bChangeFlag=true;
  }
  if (pLidData->eCurrentEmotionState!=pLidData->eNewEmotionState)  // меняется эмоция
  {
    pLidData->eCurrentEmotionState=pLidData->eNewEmotionState;
    // sLeftLidState и sRigthLidState будут переданы дальше для отрисовки век
    pLidData->sLeftLidState.eEmotionCondition=pLidData->eCurrentEmotionState;
    pLidData->sRigthLidState.eEmotionCondition=pLidData->eCurrentEmotionState;
    bChangeFlag=true;
  }
  return bChangeFlag;
}

void LidAmimationRoll::Start()
{
 //  std::cout << "LidAnimatorDefault start "<< std::endl;
  iRollBlinkStepCounter=0;
  bNeedStop=false;
}
void LidAmimationRoll::Stop()
{
  //std::cout << "LidAnimatorDefault stop "<< std::endl;
bNeedStop=true;
}
bool LidAmimationRoll::isStoped()
{
  // пока анимация не завершится - возвращаем false
  return (iRollBlinkStepCounter>=ERB_EFFECT_DURATION);
}
