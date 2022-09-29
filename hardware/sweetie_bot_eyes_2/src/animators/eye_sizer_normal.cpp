#include "eye_sizer_normal.h"

EyeSizerNormal::EyeSizerNormal()
{

}

void EyeSizerNormal::SetSizeData(EyeSizeDataStruct *pEyeSizeData)
{
  pSizeData=pEyeSizeData;
}
bool EyeSizerNormal::Process()
{
  if ( pSizeData->sCurrentSizes== pSizeData->sNewSizes) return false;
  pSizeData->sCurrentSizes=pSizeData->sNewSizes;
  return true;
}

void EyeSizerNormal::Start()
{
  bIsStopt=false;
}

void EyeSizerNormal::Stop()
{
  bIsStopt=true;
}
bool EyeSizerNormal::isStoped()
{
  return bIsStopt;
}
