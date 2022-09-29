#include "eye_mover_normal.h"
#include <iostream>

EyeMoverNormal::EyeMoverNormal()
{
  qpOldTarget=QPointF(0,0);
  qpVectSpeed=QPointF(0,0);
}

void EyeMoverNormal::Start()
{
  bStartedFlag=true;
  qpOldTarget=QPointF(0,0);
  qpVectSpeed=QPointF(0,0);
  bNeedStop=false;
}
void EyeMoverNormal::Stop()
{
  bNeedStop=true;
}

bool EyeMoverNormal::isStoped()
{
  if (bNeedStop)  return (qpVectSpeed==QPointF(0,0));
  else return false;
}

void EyeMoverNormal::SetMoveData(EyeMoveDataStruct *pEyeMoveData)
{
  pMoveData=pEyeMoveData;
}


bool EyeMoverNormal::Process()
{
  QPointF qpVectSpeedRot;
  if (bStartedFlag) // Сбросить предыдущую точку при старте
  {
    qpOldTarget=pMoveData->qpCurrentPoint;
    bStartedFlag=false;
  }

  qreal rDelta=QLineF(pMoveData->qpCurrentPoint,pMoveData->qpNewPoint).length();
 if ((rDelta<0.001) & (qAbs(qpVectSpeed.manhattanLength())<0.005)) // никуда двигатся не нужно
 {
   qpVectSpeed=QPointF(0,0);
   // todo:    возвращать false, когда сделаю аниматор для размеров
   return false; // аниматор для размерров не здесь
 }
 else // нужно расчитывать движение
 {
   // вычисляем угол движения
   QPointF pDirect=pMoveData->qpNewPoint-pMoveData->qpCurrentPoint;
   qreal qrGip=QLineF(QPointF(0,0), pDirect).length();
   if (qrGip>0.00025)// синус и косинус угла направления
   {
     qrSinAlf=(-1)*pDirect.y()/qrGip;
     qrCosAlf=pDirect.x()/qrGip;
   }

   if ((qpOldTarget!=pMoveData->qpNewPoint) & (bNeedStop==false)) // обновилась точка назначения
   {
     iDecCount=A_MOVE_STEEP; // время достижения точки назначения
     eMoveState=eActionState;
     // синус и косинус направления на старую точку,
     pDirect=qpOldTarget-pMoveData->qpCurrentPoint;
     qrGip=QLineF(QPointF(0,0), pDirect).length();
     qreal qrSinAlf_old=0;
     qreal qrCosAlf_old=0;
     if (qrGip>0.00025)// синус и косинус угла направления
     {
      qrSinAlf_old=pDirect.y()/qrGip;
      qrCosAlf_old=pDirect.x()/qrGip;
     }

     // вектор истинной скорости
     // собирается из касательной и центростремительной составляющих
     // певернутых на минус угол направления до изменения точки назначения
     QPointF qpTrueSpeed;
     qpTrueSpeed.setX(qpVectSpeed.x()*qrCosAlf_old-qpVectSpeed.y()*qrSinAlf_old);
     qpTrueSpeed.setY(qpVectSpeed.x()*qrSinAlf_old+qpVectSpeed.y()*qrCosAlf_old);

     // затем получаем новые касательные X и центростремителные Y
     // составляющие, поворачивая вектор истиной скорости на
     //  угол нового направления
      qpVectSpeed.setX(qpTrueSpeed.x()*qrCosAlf-qpTrueSpeed.y()*qrSinAlf);
      qpVectSpeed.setY(qpTrueSpeed.x()*qrSinAlf+qpTrueSpeed.y()*qrCosAlf);
      // обновляем точку назначнения
      qpOldTarget=pMoveData->qpNewPoint;
   }
   /*********** расчитываем изменения скорости *******************/
   //qreal qrSpeed=qpVectSpeed.x();
   switch (eMoveState)
   {
    case eActionState:
    iDecCount--;
    // торможение/ускорение центростремительной скорости
     qpVectSpeed.setX(SpeedDecelerat(qpVectSpeed.x(),rDelta,iDecCount));
     // гашение касательной составляющей
     //qpVectSpeed.setY(qpVectSpeed.y()-(qpVectSpeed.y()/iDecCount));
     qpVectSpeed.setY(qpVectSpeed.y()-(qpVectSpeed.y()/sqrt(iDecCount)));

     if (iDecCount==1)eMoveState=eStopState;
     break;
   case eStopState:
     qpVectSpeed.setX(0);
     break;
   }

   /******   поворачиваем вектор скорости по ходу движения ******/
   qpVectSpeedRot.setX(qpVectSpeed.x()*qrCosAlf-(-1)*qpVectSpeed.y()*qrSinAlf);
   qpVectSpeedRot.setY((-1)*qpVectSpeed.x()*qrSinAlf+qpVectSpeed.y()*qrCosAlf);

   /********* применяем скорость *************************/
   pMoveData->qpCurrentPoint=pMoveData->qpCurrentPoint+qpVectSpeedRot;
  /***************************************************************/
  } //if
  return true;
}

qreal EyeMoverNormal::SpeedDecelerat(qreal qrCurSpeed, qreal qrLength, qreal qrDecStep)
{
  qreal iStep=qRound(qrDecStep);
  if (iStep<1) iStep=1;
  qreal qrDeceleration=(12/pow(iStep,2))*qrLength - (6/iStep)*qrCurSpeed;
  // ограничить ускорение
 // std::cout << " iStep=" << iStep<< " qrCurSpeed=" << qrCurSpeed <<" qrDeceleration=" << qrDeceleration <<" qrLength=" << qrLength << std::endl;
  return (qrCurSpeed+qrDeceleration);
}

qreal EyeMoverNormal::SpeedAccelerat(qreal qrCurSpeed,qreal qrLength)
{
  qreal qrAccelerat=qrLength/80;
  if (qrAccelerat<A_ACCELERATION) qrAccelerat=A_ACCELERATION;
  qreal qrNewSpeed=qrCurSpeed+qrAccelerat;
  if (qrNewSpeed>A_MAX_SPEED) qrNewSpeed=A_MAX_SPEED;
  return qrNewSpeed;
}




