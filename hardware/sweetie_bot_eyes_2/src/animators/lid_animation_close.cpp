#include "lid_animation_close.h"

LidAnimationClose::LidAnimationClose()
{
  // TODO: можно сделать полавнее моргание используя QEasingCurve::InOutQuad
  // но это когда-нибудь потом
 // pEasingCurve=new QEasingCurve(QEasingCurve::InOutQuad);
  pBlinkAnim=new QVariantAnimation;
  pBlinkAnim->setStartValue((float)0);
  pBlinkAnim->setKeyValueAt((qreal)LAC_CLOSURE_TIME/LAC_EFFECT_DURATION,(float)1);
  pBlinkAnim->setKeyValueAt((qreal)(LAC_CLOSE_TIME)/LAC_EFFECT_DURATION,(float)1);
  pBlinkAnim->setEndValue((float)0);
  pBlinkAnim->setDuration(LAC_EFFECT_DURATION);
}

void LidAnimationClose::SetLidData(EyeLidDataStruct *pEyeLidData)
{
  pLidData=pEyeLidData;
}

bool LidAnimationClose::Process()
{
  bool bChangeFlag=false; // флаг показывает были ли изменения

  if (iBlinkStepCounter<LAC_EFFECT_DURATION) // анимация еще не завершена
  {
    iBlinkStepCounter++;
    // если пришла команда стоп - переходим к открыванию глаза
    if (bNeedStop)
    {
      if ((iBlinkStepCounter>LAC_CLOSURE_TIME) & (iBlinkStepCounter<LAC_CLOSE_TIME))iBlinkStepCounter=LAC_CLOSE_TIME;
    }
    else if (iBlinkStepCounter>LAC_CLOSURE_TIME) iBlinkStepCounter=LAC_CLOSURE_TIME;

    pBlinkAnim->setCurrentTime(iBlinkStepCounter);
    pLidData->sLeftLidState.fClosePercent=pBlinkAnim->currentValue().toFloat();
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

void LidAnimationClose::Start()
{
  iBlinkStepCounter=0;
  bNeedStop=false;
}
void LidAnimationClose::Stop()
{
  bNeedStop=true;
}

bool LidAnimationClose::isStoped()
{
  // пока анимация не завершится - возвращаем false
  return (iBlinkStepCounter>=LAC_EFFECT_DURATION);
}
