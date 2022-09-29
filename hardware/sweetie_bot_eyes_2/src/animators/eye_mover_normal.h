#ifndef EYE_MOVER_NORMAL_H
#define EYE_MOVER_NORMAL_H

#include <QString>
#include <QLineF>
#include <QtMath>
#include <QObject>
#include <QOpenGLFunctions>

#include "my_types.h"
#include "eye_animator_interface.h"

#define A_MOVE_STEEP        20   // за сколько кадров достигнет новой точки
#define A_MAX_SPEED        2
#define A_ACCELERATION     0.002

enum eMoveStateType
{
  eAccState,
  eDecState,
  eActionState,
  eStopState
};


class EyeMoverNormal : public IMoveAnimatorInterface
{
public:
  EyeMoverNormal();
// void Init(){};
  bool Process();
  void Start();
  void Stop();
  bool isStoped();
  std::string GetLidAnimationName(){ return "";}
  void SetMoveData(EyeMoveDataStruct* pEyeMoveData);

private:
  EyeMoveDataStruct* pMoveData;

  bool bNeedStop;
  bool bStartedFlag;
  eMoveStateType eMoveState =eAccState;
  QPointF qpOldTarget;
  QPointF qpVectSpeed;

  qreal qrSinAlf=1, qrCosAlf=0;
  int iDecCount=0;
  qreal SpeedDecelerat(qreal qrCurSpeed, qreal qrLength, qreal qrDecStep);
  qreal SpeedAccelerat(qreal qrCurSpeed, qreal qrLength);

 // QString sEyeAnimatorName="Normal";
};

#endif // EYE_MOVER_NORMAL_H
