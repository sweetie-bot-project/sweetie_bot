#include "lid_animator_default.h"

LidAnimatorDefault::LidAnimatorDefault()
{

}

void LidAnimatorDefault::SetLidData(EyeLidDataStruct *pEyeLidData)
{
  pLidData=pEyeLidData;
}

bool LidAnimatorDefault::Process()
{
  bool bChangeFlag=false; // флаг показывает были ли изменения
 // а что с блинком делать-то??????
  // добавить рандомный блинк для живости
  if (pLidData->eCurrentEmotionState!=pLidData->eNewEmotionState)  // меняется эмоция
  {
    pLidData->eCurrentEmotionState=pLidData->eNewEmotionState;
    // sLeftLidState и sRigthLidState будут переданы дальше для отрисовки век
    pLidData->sLeftLidState.eEmotionCondition=pLidData->eCurrentEmotionState;
    pLidData->sRigthLidState.eEmotionCondition=pLidData->eCurrentEmotionState;
    bChangeFlag=true;
  }

  return bChangeFlag;
}

void LidAnimatorDefault::Start()
{
 //  std::cout << "LidAnimatorDefault start "<< std::endl;
bIsStopFlag=false;
}
void LidAnimatorDefault::Stop()
{
  //std::cout << "LidAnimatorDefault stop "<< std::endl;
bIsStopFlag=true;
}
bool LidAnimatorDefault::isStoped()
{
  return bIsStopFlag;
}
