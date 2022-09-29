#ifndef EYELIDPARAMPREPARER_H
#define EYELIDPARAMPREPARER_H

#include "my_types.h"
#include <iostream>


struct sLidElimentMoveStruct
{
  LidPaintParamStruct sStart;
  LidPaintParamStruct sEnd;
};

class EyeLidParamPreparer
{
public:
  EyeLidParamPreparer();
  bool Process();
  void SetLidEmotion(eLidEmotionType eLidEmotion);
  void SetLidBlinkState(float fClosePercent);
  void NewSetLidParamCommand(QString *psCommand);
  LidPaintPropertyStruct* GetLidPaintProperty();


private:
  eLidEmotionType aCurrentEmotion;
  LidPaintPropertyStruct sLidPaintProperty;
  float fChangeEmotionProgres=1;
  bool bNeedProcess=true;
  sLidElimentMoveStruct sCurrentUp, sCurrentDown;
  sLidElimentMoveStruct sNewUp, sNewDown;
  sLidElimentMoveStruct sOldUp, sOldDown;
  float fCurrentClosePercent=0;
  void ApplayClosePercent(LidPaintParamStruct* pLidOut, sLidElimentMoveStruct *pLidCurrent);
  sLidElimentMoveStruct ChangeEmotion(sLidElimentMoveStruct sNewEmotion, sLidElimentMoveStruct sOldEmotion);
  LidPaintParamStruct ChangeLidPaintParam(LidPaintParamStruct sNewParam, LidPaintParamStruct sOldParam);
public slots:
  void SetNewLidDisplayState(LidDisplayStateStruct *pDisplayState);
};

#endif // EYELIDPARAMPREPARER_H
