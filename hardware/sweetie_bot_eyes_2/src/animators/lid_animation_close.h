#ifndef LIDANIMATIONCLOSE_H
#define LIDANIMATIONCLOSE_H

#include <QVariantAnimation>
#include <iostream>

#include "eye_animator_interface.h"
/***********************************************************
 * Закрывает глаз и ждет команды Stop(). потом открывает
 * закрывание глаза 8
 * закрытый глаз 4
 * открывание глаза 8
 * Если прила команда стоп - сразу переходим к открыванию глаза
 * **********************************************************************/
#define LAC_CLOSURE_TIME  8
#define LAC_CLOSE_TIME    (4+LAC_CLOSURE_TIME)
#define LAC_OPENING_TIME  (8+LAC_CLOSE_TIME)
#define LAC_EFFECT_DURATION  LAC_OPENING_TIME

class LidAnimationClose:public ILidAnimatorInterface
{
public:
  LidAnimationClose();
  bool Process();
  void Start();
  void Stop();
  bool isStoped();
  void SetLidData(EyeLidDataStruct* pEyeLidData);
private:
  QVariantAnimation *pBlinkAnim;
  bool bNeedStop=false;
  int iBlinkStepCounter=0;
  EyeLidDataStruct* pLidData;
};


#endif // LIDANIMATIONCLOSE_H
