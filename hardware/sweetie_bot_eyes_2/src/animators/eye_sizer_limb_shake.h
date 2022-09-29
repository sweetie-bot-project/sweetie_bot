#ifndef EYESIZERLIMBSHAKE_H
#define EYESIZERLIMBSHAKE_H

#include "eye_animator_interface.h"

#define ESL_MAX_DEV 0.02  //  увеличение зрачка
#define ESL_MIN_DEV -0.02 //  уменьшение зрачка
#define ESL_SHAKE_DURATION  2 // количество кадров в одном дрожании
#define ESL_SHAKE_STEP ((ESL_MAX_DEV)-(ESL_MIN_DEV))/ESL_SHAKE_DURATION


class EyeSizerLimbShake: public ISizeAnimatorInterface
{
public:
  EyeSizerLimbShake();
  bool Process();
  void Start();
  void Stop();
  bool isStoped();
  void SetSizeData(EyeSizeDataStruct* pEyeSizeData);
 private:
  qreal qrLimbShakeProgress=0;
  qreal qrShakeStep=ESL_SHAKE_STEP;
  bool bNeedStop=false;
  EyeSizeDataStruct* pSizeData;
  bool bIsStopt=false;
};
#endif // EYESIZERLIMBSHAKE_H
