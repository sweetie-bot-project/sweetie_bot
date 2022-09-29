#ifndef LIDANIMATORDEFAULT_H
#define LIDANIMATORDEFAULT_H

#include <iostream>

#include "eye_animator_interface.h"

class LidAnimatorDefault:public ILidAnimatorInterface
{
public:
  LidAnimatorDefault();
  bool Process();
  void Start();
  void Stop();
  bool isStoped();
  void SetLidData(EyeLidDataStruct* pEyeLidData);
private:
  bool bIsStopFlag=false;
  EyeLidDataStruct* pLidData;
};

#endif // LIDANIMATORDEFAULT_H
