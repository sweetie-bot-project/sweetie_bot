#include "eye_lid_param_preparer.h"

EyeLidParamPreparer::EyeLidParamPreparer()
{
  sCurrentUp.sStart.fAngle=0;
  sCurrentUp.sEnd.fAngle=0;
  sCurrentUp.sStart.fPos=QPointF(0,1.5);
  sCurrentUp.sEnd.fPos=QPointF(0,0.5);

  sCurrentDown.sStart.fAngle=0;
  sCurrentDown.sEnd.fAngle=0;
  sCurrentDown.sStart.fPos=QPointF(0,-1.6);
  sCurrentDown.sEnd.fPos=QPointF(0,-1.5);

  aCurrentEmotion=LF_UNKNOW;
  SetLidEmotion(LF_NORMAL);
}

bool EyeLidParamPreparer::Process() // будет вызываться каждый кадр
{
  if (!bNeedProcess) return false;

  if (fChangeEmotionProgres<1) // идет переход между эмоциями
  {
    fChangeEmotionProgres+=0.05;
    if (fChangeEmotionProgres>1) fChangeEmotionProgres=1;
    sCurrentUp=ChangeEmotion(sNewUp,sOldUp);
    sCurrentDown=ChangeEmotion(sNewDown,sOldDown);
  }
  ApplayClosePercent(&sLidPaintProperty.sUpLid,&sCurrentUp);
  ApplayClosePercent(&sLidPaintProperty.sDownLid,&sCurrentDown);
  if (fChangeEmotionProgres>1) bNeedProcess=false;

  return true;  // true - инициирует перерисовку всего глаза RecalculateTranformMarix();
}
void EyeLidParamPreparer::ApplayClosePercent(LidPaintParamStruct* pLidOut, sLidElimentMoveStruct *pLidCurrent)
{
  pLidOut->fAngle=pLidCurrent->sStart.fAngle*(1-fCurrentClosePercent)+pLidCurrent->sEnd.fAngle*fCurrentClosePercent;
  pLidOut->fPos=pLidCurrent->sStart.fPos*(1-fCurrentClosePercent)+pLidCurrent->sEnd.fPos*fCurrentClosePercent;
}

LidPaintPropertyStruct* EyeLidParamPreparer::GetLidPaintProperty()
{
  return &sLidPaintProperty;
}

//-----  функция для настройки положения век -----///
constexpr unsigned int str2hash(const char* str, int h = 0)
{
  return !str[h] ? 5381 : (str2hash(str, h+1)*33) ^ str[h];
}

void EyeLidParamPreparer::NewSetLidParamCommand(QString *psCommand)
{
  QStringList sList;
  bool bResult;
  double dVal;

  sList = psCommand->split('=', QString::SkipEmptyParts);
  if(sList.size()==0) return;
  dVal=sList[1].toDouble(&bResult);
  if (!bResult) return;
  if (sList[0]=="LidClosePercent")
  {
    SetLidBlinkState(dVal);
    return;
  }
  sLidElimentMoveStruct* pLES=&sCurrentUp;
  if (sList[2]=="down")pLES=&sCurrentDown;

  switch(str2hash(sList[0].toStdString().c_str()))
  {
  case str2hash("LidStartA"):
    pLES->sStart.fAngle=dVal;
    break;
  case str2hash("LidStartX"):
    pLES->sStart.fPos.setX(dVal);
    break;
  case str2hash("LidStartY"):
    pLES->sStart.fPos.setY(dVal);
    break;
  case str2hash("LidEndA"):
    pLES->sEnd.fAngle=dVal;
    break;
  case str2hash("LidEndX"):
    pLES->sEnd.fPos.setX(dVal);
    break;
  case str2hash("LidEndY"):
    pLES->sEnd.fPos.setY(dVal);
    break;

  }
}
/****************************************************************/

void EyeLidParamPreparer::SetLidBlinkState(float fClosePercent)
{
  if (fCurrentClosePercent==fClosePercent) return; // никаких изменений нет
  fCurrentClosePercent=fClosePercent;
  bNeedProcess=true; // нужно пересчитать параметры век
}



/********************  SetLidEmotion  ****************************/
/* функция получает в команде новую эмоцию и
 * запускает изменение эмоции из sOldUp, sOldDown в sNewUp, sNewDown.
 * Задает начальное и конечное положение верхнего и нижнего века
 * для каждого эмоционального состояния.
 * sStart - для полностью открытого глаза
 * sEnd - для полностью закрытого.
 * *****************************************************************/
void EyeLidParamPreparer::SetLidEmotion(eLidEmotionType eLidEmotion)
{
  if (aCurrentEmotion==eLidEmotion) return; // чтоб постоянно не срабатывала
  aCurrentEmotion=eLidEmotion;

  switch (eLidEmotion) {
  case LF_NONE:
    std::cout << "new emotion LF_NONE "<< std::endl;
    sOldUp=sCurrentUp;
    sOldDown=sCurrentDown;

    sNewUp.sStart.fAngle=0;
    sNewUp.sEnd.fAngle=0;
    sNewUp.sStart.fPos=QPointF(0,2);
    sNewUp.sEnd.fPos=QPointF(0,2);

    sNewDown.sStart.fAngle=0;
    sNewDown.sEnd.fAngle=0;
    sNewDown.sStart.fPos=QPointF(0,-2);
    sNewDown.sEnd.fPos=QPointF(0,-2);
    break;
  case LF_NORMAL:
    std::cout << "new emotion LF_NORMAL "<< std::endl;
    sOldUp=sCurrentUp;
    sOldDown=sCurrentDown;

    sNewUp.sStart.fAngle=0;
    sNewUp.sEnd.fAngle=0;
    sNewUp.sStart.fPos=QPointF(0,1.5);
    sNewUp.sEnd.fPos=QPointF(0,0.5);

    sNewDown.sStart.fAngle=0;
    sNewDown.sEnd.fAngle=0;
    sNewDown.sStart.fPos=QPointF(0,-1.6);
    sNewDown.sEnd.fPos=QPointF(0,-1.5);
    break;
  case LF_SHY:
    std::cout << "new emotion LF_SHY "<< std::endl;
    sOldUp=sCurrentUp;
    sOldDown=sCurrentDown;

    sNewUp.sStart.fAngle=-10;
    sNewUp.sEnd.fAngle=0;
    sNewUp.sStart.fPos=QPointF(0,1.2);
    sNewUp.sEnd.fPos=QPointF(0,0.5);

    sNewDown.sStart.fAngle=0;
    sNewDown.sEnd.fAngle=0;
    sNewDown.sStart.fPos=QPointF(0,-1.6);
    sNewDown.sEnd.fPos=QPointF(0,-1.5);
    break;
  default:
    sOldUp=sCurrentUp;
    sOldDown=sCurrentDown;
    std::cout << "unknow new emotion "<< std::endl;
    // aCurrentEmotion=LF_UNKNOW;
    break;
  }
  fChangeEmotionProgres=0; // стартует плавное изменение положения век
  bNeedProcess=true;
}


//------------------  функции для плавного изменения эмоционального положения век -------------//
sLidElimentMoveStruct EyeLidParamPreparer::ChangeEmotion(sLidElimentMoveStruct sNewEmotion, sLidElimentMoveStruct sOldEmotion)
{
  sLidElimentMoveStruct sEmotion;

  sEmotion.sEnd=ChangeLidPaintParam(sNewEmotion.sEnd,sOldEmotion.sEnd);
  sEmotion.sStart=ChangeLidPaintParam(sNewEmotion.sStart,sOldEmotion.sStart);
  return sEmotion;
}
LidPaintParamStruct EyeLidParamPreparer::ChangeLidPaintParam(LidPaintParamStruct sNewParam, LidPaintParamStruct sOldParam)
{
  LidPaintParamStruct sLidParam;
  sLidParam.fAngle=sOldParam.fAngle*(1-fChangeEmotionProgres)+sNewParam.fAngle*fChangeEmotionProgres ;
  sLidParam.fPos=sOldParam.fPos*(1-fChangeEmotionProgres)+sNewParam.fPos*fChangeEmotionProgres ;
  return sLidParam;
}
//---------------------------------------------------------------------------------------------------//
