#ifndef EYESIZERNORMAL_H
#define EYESIZERNORMAL_H
#include "eye_animator_interface.h"

class EyeSizerNormal: public ISizeAnimatorInterface
{
public:
  EyeSizerNormal();
  bool Process();
  void Start();
  void Stop();
  bool isStoped();
  void SetSizeData(EyeSizeDataStruct* pEyeSizeData);
 private:
  EyeSizeDataStruct* pSizeData;
  bool bIsStopt=false;
};

#endif // EYESIZERNORMAL_H
