#include "eye_animation_manager.h"
#include <cmath>

EyeAnimationManager::EyeAnimationManager(QObject *parent) : QObject(parent)
{
  sSizeData.sNewSizes.qrLimbSize=0.5;
  sSizeData.sCurrentSizes.qrLimbSize=0.5;
  sSizeData.sNewSizes.qrIrisSize=0.5;
  sSizeData.sCurrentSizes.qrIrisSize=0.5;
  sSizeData.sNewSizes.qrLigtSize=sSizeData.sNewSizes.qrLimbSize*sSizeData.sNewSizes.qrIrisSize;

  sMoveData.qpNewPoint.setX(0.2);
  sMoveData.qpNewPoint.setY(0.2);
  sMoveData.qpCurrentPoint.setX(0);
  sMoveData.qpCurrentPoint.setY(0);
  sLidData.eCurrentEmotionState=LF_NONE;
  sLidData.eNewEmotionState=LF_NORMAL;
  sLidData.sLeftLidState.fClosePercent=0;
  sLidData.sRigthLidState.fClosePercent=0;

  pEyeNormalMover=new EyeMoverNormal();
  pEyeNormalMover->Init();
  pEyeNormalMover->SetMoveData(&sMoveData);

  pEyeRollMover=new EyeMoverRoll();
  pEyeRollMover->Init();
  pEyeRollMover->SetMoveData(&sMoveData);

  pLidAnimDefault=new LidAnimatorDefault();
  pLidAnimDefault->SetLidData(&sLidData);

  pLidAnimBlink=new LidAnimationBlink();
  pLidAnimBlink->SetLidData(&sLidData);

  pLidAnimRoll=new LidAmimationRoll();
  pLidAnimRoll->SetLidData(&sLidData);

  pLidAnimClose=new LidAnimationClose();
  pLidAnimClose->SetLidData(&sLidData);

  pEyeSizerNormal=new EyeSizerNormal();
  pEyeSizerNormal->SetSizeData(&sSizeData);

  pEyeSizerLimbShake=new EyeSizerLimbShake();
  pEyeSizerLimbShake->SetSizeData(&sSizeData);

  aMove.pCurrent=pEyeNormalMover;
  aMove.pDefault=pEyeNormalMover;

  aLid.pCurrent=pLidAnimDefault;
  aLid.pDefault=pLidAnimDefault;

  aSize.pCurrent=pEyeSizerNormal;
  aSize.pDefault=pEyeSizerNormal;
}

EyeAnimationManager::~EyeAnimationManager()
{
  //delete pEyeMover;
}

void EyeAnimationManager::NewPitch(double p)
{
  p=EAM_TRANSFORM_COEF*p;
  sMoveData.qpNewPoint.setX(p);
}
void EyeAnimationManager::NewYaw(double y)
{
  y=EAM_TRANSFORM_COEF*y;
  sMoveData.qpNewPoint.setY(y);
}
constexpr unsigned int str2hash(const char* str, int h = 0)
{
  return !str[h] ? 5381 : (str2hash(str, h+1)*33) ^ str[h];
}

void EyeAnimationManager::NewSetParamCommand(QString *psCommand)
{
  QStringList sList;
  bool bResult;
  double dVal;
  sList = psCommand->split('=', QString::SkipEmptyParts);
  if(sList.size()==0) return;
  switch(str2hash(sList[0].toStdString().c_str()))
  {
  case str2hash("LimbSize"):
    dVal=sList[1].toDouble(&bResult);
    if (bResult)
    {
      sSizeData.sNewSizes.qrLimbSize=dVal;
      // блики меняются вместе с размером зрачка и радужки.
      sSizeData.sNewSizes.qrLigtSize=sSizeData.sNewSizes.qrIrisSize*sSizeData.sNewSizes.qrLimbSize;
    }
    break;
  case str2hash("IrisSize"):
    dVal=sList[1].toDouble(&bResult);
    if (bResult)
    {
      sSizeData.sNewSizes.qrIrisSize=dVal;
      // блики меняются вместе с размером зрачка и радужки.
      sSizeData.sNewSizes.qrLigtSize=sSizeData.sNewSizes.qrIrisSize*sSizeData.sNewSizes.qrLimbSize;
    }
    break;
  case str2hash("EyeFigureName"):
    emit EyeFigureName(&sList[1]);
    break;
  case str2hash("Pitch"):
    dVal=sList[1].toDouble(&bResult);
    if (bResult) NewPitch(dVal);
    break;
  case str2hash("Yaw"):
    dVal=sList[1].toDouble(&bResult);
    if (bResult) NewYaw(dVal);
    break;
  }
}

void EyeAnimationManager::NewEmotionCommand(QString *psCommand)
{
  //  std::cout << "NewEmotionCommand= "<< psCommand->toStdString()<< std::endl;
  switch(str2hash(psCommand->toStdString().c_str()))
  {
  case str2hash("None"): // веки скрыты
    sLidData.eNewEmotionState=LF_NONE;
    break;
  case str2hash("Normal"):
    sLidData.eNewEmotionState=LF_NORMAL;
    break;
  case str2hash("Shy"):
    sLidData.eNewEmotionState=LF_SHY;
    break;
  case str2hash("Angry"):
    sLidData.eNewEmotionState=LF_ANGRY;
    break;
  case str2hash("Tired"):
    sLidData.eNewEmotionState=LF_TIRED;
    break;
  case str2hash("Suspect"):
    sLidData.eNewEmotionState=LF_SUSPECT;
    break;
  }
}

void EyeAnimationManager::NewAnimationMode(QString *psAnimationMode)
{
  // TODO: прищур. подозревака
  std::cout << "new mover "<< psAnimationMode->toStdString()<< std::endl;
  bGetNewAnimator=true;
  switch(str2hash(psAnimationMode->toStdString().c_str()))
  {
  case str2hash("Normal"):
    PrepareNewAnimator(&aMove,pEyeNormalMover);// следующим запустится NormalMover
    break;
  case str2hash("Spin"):
    PrepareNewAnimator(&aMove,pEyeRollMover);// следующим запустится RollMover
    PrepareNewAnimator(&aLid,pLidAnimRoll); // и соответствующая анимация век
    break;
  case str2hash("Blink"):
    PrepareNewAnimator(&aLid,pLidAnimBlink);// следующим запустится pLidAnimBlink
    break;
  case str2hash("CloseEye"):
    PrepareNewAnimator(&aLid,pLidAnimClose);// следующим запустится pLidAnimClose
    break;
  case str2hash("LidNormal"):
    PrepareNewAnimator(&aLid,pLidAnimDefault);
    break;
  case str2hash("Shake"):
    PrepareNewAnimator(&aSize,pEyeSizerLimbShake);
    break;
  case str2hash("Size_normal"):
    PrepareNewAnimator(&aSize,pEyeSizerNormal);
    break;
  }
}

void EyeAnimationManager::PrepareNewAnimator(AnimatorToggleStruct *pAnimatorStruct, IEyeAnimatorInterface *pNewAnimator)
{
  pAnimatorStruct->pCurrent->Stop();   // команда на завершение действующего
  pAnimatorStruct->pNew=pNewAnimator;  // следующим запустится pNewAnimator
  pAnimatorStruct->bNeedStartNew=true; // нужно новый подключить и стартовать
}

void EyeAnimationManager::StartNewAnimator(AnimatorToggleStruct *pAnimatorStruct)
{
  if(pAnimatorStruct->bNeedStartNew)
  {
    pAnimatorStruct->pCurrent=pAnimatorStruct->pNew;
    pAnimatorStruct->pCurrent->Start();
    pAnimatorStruct->bNeedStartNew=false;
  }
}

bool EyeAnimationManager::ProccesAnimator(AnimatorToggleStruct *pAnimatorStruct)
{
  bool bResult=false;
  // Выполняет текущий аниматор или стартует дефолтный если можно;
  if (pAnimatorStruct->pCurrent->isStoped())
  {
    if (pAnimatorStruct->bNeedStartNew==false)
    {
      pAnimatorStruct->pCurrent=pAnimatorStruct->pDefault;
      pAnimatorStruct->pCurrent->Start();
      bResult=pAnimatorStruct->pCurrent->Process();
    }
  }
  else bResult= pAnimatorStruct->pCurrent->Process();
  return bResult;
}

void EyeAnimationManager::timer_clk()
{
  bool bEyePosOrSizeChangeFlag=false;
  //----- переключение аниматоров --------

  if (bGetNewAnimator) //требуется переключение аниматоров
  {
    //----- синхронизируем старт всех новых аниматоров----------------
    bool bReadyToStartNewAnomators=true;
    // Все ли аниматоры, какие надо, готовы к запуску
    if (aMove.bNeedStartNew) bReadyToStartNewAnomators=(bReadyToStartNewAnomators & aMove.pCurrent->isStoped());
    if (aSize.bNeedStartNew) bReadyToStartNewAnomators=(bReadyToStartNewAnomators & aSize.pCurrent->isStoped());
    if (aLid.bNeedStartNew) bReadyToStartNewAnomators=(bReadyToStartNewAnomators & aLid.pCurrent->isStoped());

    //--- запуск аниматоров --------
    if(bReadyToStartNewAnomators)
    {
      StartNewAnimator(&aMove);
      StartNewAnimator(&aSize);
      StartNewAnimator(&aLid);
      bGetNewAnimator=false; // переключение аниматоров завершено
    }
  }

  // Выполняет текущий аниматор или стартует дефолтный если нужно;
  bEyePosOrSizeChangeFlag|=ProccesAnimator(&aMove);
  bEyePosOrSizeChangeFlag|=ProccesAnimator(&aSize);

  if (ProccesAnimator(&aLid)) // аниматор обновил данные
  {
    emit EyeLeftLidStateSend(&sLidData.sLeftLidState);
    emit EyeRigthLidStateSend(&sLidData.sRigthLidState);
  }

  if(bEyePosOrSizeChangeFlag) // изменилось положение глаза или его размер
  {
    sEyeParam.qpEyePos=sMoveData.qpCurrentPoint;
    sEyeParam.tEyeSizes=sSizeData.sCurrentSizes;
    emit EyeParamSend(&sEyeParam); // отправляем параметры дальше на отрисовщик
  }
  /**************************** дальше старый код **************************/
  // pCurrentMover->Process(&sEyeAnimatorData); // мувер свое дело делает
  /*
  sEyeParam.qpEyePos=sMoveData.qpCurrentPoint;
  sEyeParam.tEyeSizes=sSizeData.sCurrentSizes;

  qreal qrIrisSize=sSizeData.sCurrentSizes.qrIrisSize;

  sEyeParam.tEyeSizes.qrLimbSize=sSizeData.sCurrentSizes.qrLimbSize*qrIrisSize;
  // временно!!!!уменьшение бликов вместе со зрачком
  sEyeParam.tEyeSizes.qrLigtSize=sSizeData.sCurrentSizes.qrLimbSize*qrIrisSize;
  emit EyeParamSend(&sEyeParam); // отправляем параметры дальше на отрисовщик
  */
}
