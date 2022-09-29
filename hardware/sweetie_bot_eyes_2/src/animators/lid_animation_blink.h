#ifndef LIDANIMATIONBLINK_H
#define LIDANIMATIONBLINK_H

#include <QVariantAnimation>
#include <iostream>

#include "eye_animator_interface.h"
/***********************************************************
 * Моргание длится 20 кадров. Из них:
 * закрывание глаза 8
 * закрытый глаз 4
 * открывание глаза 8
 * Если прила команда стоп - сразу переходим к открыванию глаза
 * **********************************************************************/
#define EEB_CLOSURE_TIME  8
#define EEB_CLOSE_TIME    4
#define EEB_OPENING_TIME  8
#define EEB_EFFECT_DURATION  (EEB_CLOSURE_TIME+EEB_CLOSE_TIME+EEB_OPENING_TIME)

class LidAnimationBlink:public ILidAnimatorInterface
{
public:
  LidAnimationBlink();
  bool Process();
  void Start();
  void Stop();
  bool isStoped();
  void SetLidData(EyeLidDataStruct* pEyeLidData);
private:
  QVariantAnimation *pBlinkAnim;
 // float fBlinkProgress;
  bool bIsStopFlag=false;
  bool bNeedStop=false;
  int iBlinkStepCounter=0;
  EyeLidDataStruct* pLidData;
};

#endif // LIDANIMATIONBLINK_H
