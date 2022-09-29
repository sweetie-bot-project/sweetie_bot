#include "lid_animation_blink.h"

LidAnimationBlink::LidAnimationBlink()
{
  // TODO: можно сделать полавнее моргание используя QEasingCurve::InOutQuad
  // но это когда-нибудь потом
 // pEasingCurve=new QEasingCurve(QEasingCurve::InOutQuad);
  pBlinkAnim=new QVariantAnimation;
  pBlinkAnim->setStartValue((float)0);
  pBlinkAnim->setKeyValueAt((qreal)EEB_CLOSURE_TIME/EEB_EFFECT_DURATION,(float)1);
  pBlinkAnim->setKeyValueAt((qreal)(EEB_CLOSURE_TIME+EEB_CLOSE_TIME)/EEB_EFFECT_DURATION,(float)1);
  pBlinkAnim->setEndValue((float)0);
  pBlinkAnim->setDuration(EEB_EFFECT_DURATION);
}

void LidAnimationBlink::SetLidData(EyeLidDataStruct *pEyeLidData)
{
  pLidData=pEyeLidData;
}

bool LidAnimationBlink::Process()
{
  bool bChangeFlag=false; // флаг показывает были ли изменения

  if (iBlinkStepCounter<EEB_EFFECT_DURATION) // процесс моргания
  {
    iBlinkStepCounter++;
    // если пришла команда стоп - пропускаем паузу при моргании
    if (bNeedStop)
    {
      if ((iBlinkStepCounter>EEB_CLOSURE_TIME) & (iBlinkStepCounter<(EEB_CLOSURE_TIME+EEB_CLOSE_TIME)))iBlinkStepCounter=(EEB_CLOSURE_TIME+EEB_CLOSE_TIME);
    }

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

void LidAnimationBlink::Start()
{
   // std::cout << "LidAnimationBlink start "<< std::endl;
  iBlinkStepCounter=0;
  bNeedStop=false;

}
void LidAnimationBlink::Stop()
{
  bNeedStop=true;
}

bool LidAnimationBlink::isStoped()
{
  // пока анимация не завершится - возвращаем false
  return (iBlinkStepCounter>=EEB_EFFECT_DURATION);
}
