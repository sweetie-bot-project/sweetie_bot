#include "eye_mover_roll.h"

EyeMoverRoll::EyeMoverRoll()
{
  qreal qAlfa=MR_ROLL_ANGLE/MR_ROLL_DURATION; // угол поворота за один кадр
  qAlfa=qDegreesToRadians(qAlfa);
  mySin=qSin(qAlfa);
  myCos=qCos(qAlfa);
  eRollState=MR_START;
}
void EyeMoverRoll::Init()
{
}

void EyeMoverRoll::Start()
{
  eRollState=MR_START;
}
void EyeMoverRoll::Stop()
{
  eRollState=MR_STOP;
}

bool EyeMoverRoll::isStoped()
{
  return (eRollState==MR_STOP);
}

void EyeMoverRoll::SetMoveData(EyeMoveDataStruct *pEyeMoveData)
{
  pMoveData=pEyeMoveData;
}
/******************************************************
 * Process- изменяет положение глаза.
 * возвращает true, если были изменения и нужно перерисовывать
 *            false - если изменений небыло.
 *    *****************************************************/

bool EyeMoverRoll::Process()
{
  QPointF qpTempPoint;
  switch (eRollState) {
  case MR_START: // начало работы аниматора
    qpStartPoint=pMoveData->qpCurrentPoint;
    //----- вычисляем точку начала окружности-----
    qpSidePoint=QPointF(MR_SIDE_POINT);
    //------------------------------------------------
    vaMoveSideAnimator.setStartValue(qpStartPoint);
    vaMoveSideAnimator.setEndValue(qpSidePoint);
    vaMoveSideAnimator.setDuration(MR_MOVE_SIDE_DURATION);
    iCounter=0;
    eRollState=MR_GO_SIDE;
    break;
  case MR_GO_SIDE: // глаз движется в точку начала вращения
    vaMoveSideAnimator.setCurrentTime(iCounter);
    pMoveData->qpCurrentPoint=vaMoveSideAnimator.currentValue().toPointF();
    iCounter++;
    if (iCounter>MR_MOVE_SIDE_DURATION)
    {
      eRollState=MR_ROLL;
      iCounter=0;
      qpRollPoint=pMoveData->qpCurrentPoint;
    }
    break;
  case MR_ROLL:// поворачиваем точку на угол, вокруг начала коорданат(центра глаза)
    qpTempPoint.setX(qpRollPoint.x()*myCos-qpRollPoint.y()*mySin);
    qpTempPoint.setY(qpRollPoint.x()*mySin+qpRollPoint.y()*myCos);
    qpRollPoint=qpTempPoint;
    pMoveData->qpCurrentPoint=qpRollPoint;
    iCounter++;
    if (iCounter>MR_ROLL_DURATION) eRollState=MR_STOP;
    break;
  case MR_STOP: // анимация движения завершена
    return false;
    break;
  }
return true;
}
