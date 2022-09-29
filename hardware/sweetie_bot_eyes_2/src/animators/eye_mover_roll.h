#ifndef EYEMOVERROLL_H
#define EYEMOVERROLL_H

#include <QVariantAnimation>
#include <iostream>
#include <QtMath>

#include "eye_animator_interface.h"

#define MR_SIDE_POINT -0.5,0.5
#define MR_ROLL_DURATION  50
#define MR_ROLL_ANGLE  200
#define MR_MOVE_SIDE_DURATION 10   // сколько кадров длится движение глаза из текущей позиции в точку начала вращения

enum eMoverRollState
{
  MR_START,
  MR_GO_SIDE,
  MR_ROLL,
  MR_STOP,
};

class EyeMoverRoll:public IMoveAnimatorInterface
{
public:
  EyeMoverRoll();
  void Init();
  bool Process();
  void Start();
  void Stop();
  bool isStoped();

  bool isHaveLidAnimaton(){return true;}
  std::string GetLidAnimationName(){return "Blink";}
  void SetMoveData(EyeMoveDataStruct* pEyeMoveData);
private:
  EyeMoveDataStruct* pMoveData;
  int iCounter;
  qreal mySin;
  qreal myCos;

  QVariantAnimation vaMoveSideAnimator;
  eMoverRollState eRollState=MR_START;
  QPointF qpSidePoint;
  QPointF qpStartPoint;
  QPointF qpRollPoint;

};

#endif // EYEMOVERROLL_H
