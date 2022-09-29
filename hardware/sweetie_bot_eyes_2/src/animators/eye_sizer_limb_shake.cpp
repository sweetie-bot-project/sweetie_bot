#include "eye_sizer_limb_shake.h"

EyeSizerLimbShake::EyeSizerLimbShake()
{

}
void EyeSizerLimbShake::SetSizeData(EyeSizeDataStruct *pEyeSizeData)
{
  pSizeData=pEyeSizeData;
}
bool EyeSizerLimbShake::Process()
{
   if (bIsStopt)  // дрожание зрачков остановленно
   {
     if ( pSizeData->sCurrentSizes== pSizeData->sNewSizes) return false;
     pSizeData->sCurrentSizes= pSizeData->sNewSizes;
     return true;
   }
  pSizeData->sCurrentSizes= pSizeData->sNewSizes;
  if (qrLimbShakeProgress>ESL_MAX_DEV) qrShakeStep=ESL_SHAKE_STEP*(-1);
  if (qrLimbShakeProgress<ESL_MIN_DEV) qrShakeStep=ESL_SHAKE_STEP;
  qrLimbShakeProgress=qrLimbShakeProgress+qrShakeStep;

  pSizeData->sCurrentSizes.qrLigtSize*=(1+qrLimbShakeProgress);
  return true;
}

void EyeSizerLimbShake::Start()
{
  qrLimbShakeProgress=0;
  //bNeedStop=false;
  bIsStopt=false;
}

void EyeSizerLimbShake::Stop()
{
  //bNeedStop=true;
  bIsStopt=true;
}
bool EyeSizerLimbShake::isStoped()
{
  return bIsStopt;
}
