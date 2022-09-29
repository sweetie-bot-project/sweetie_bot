#ifndef LIDAMIMATIONROLL_H
#define LIDAMIMATIONROLL_H


#include <QVariantAnimation>
#include <iostream>

#include "eye_mover_roll.h"

#include "eye_animator_interface.h"
/***********************************************************
 * Анимация двух морганий при вращении глаз.
 * Запускается совместно с EyeMoverRoll
 * Моргание длится 20 кадров. Из них:
 * закрывание глаза 8
 * закрытый глаз 4
 * открывание глаза 8
 * Если прила команда стоп - сразу переходим к открыванию глаза
 * **********************************************************************/
#define ERB_CLOSURE_TIME_1  8
#define ERB_CLOSE_TIME_1    (4+ERB_CLOSURE_TIME_1)
#define ERB_OPENING_TIME_1  (8+ERB_CLOSE_TIME_1)
#define ERB_WATE_ROLL       (MR_ROLL_DURATION+ERB_OPENING_TIME_1)
#define ERB_CLOSURE_TIME_2  (8+ERB_WATE_ROLL)
#define ERB_CLOSE_TIME_2    (4+ERB_CLOSURE_TIME_2)
#define ERB_OPENING_TIME_2  (8+ERB_CLOSE_TIME_2)


#define ERB_EFFECT_DURATION ERB_OPENING_TIME_2


class LidAmimationRoll:public ILidAnimatorInterface
{
public:
  LidAmimationRoll();
  bool Process();
  void Start();
  void Stop();
  bool isStoped();
  void SetLidData(EyeLidDataStruct* pEyeLidData);
private:
  QVariantAnimation *pRollBlinkAnim;
 // float fRollBlinkProgress;
  bool bIsStopFlag=false;
  bool bNeedStop=false;
  int iRollBlinkStepCounter=0;
  EyeLidDataStruct* pLidData;
};


#endif // LIDAMIMATIONROLL_H
