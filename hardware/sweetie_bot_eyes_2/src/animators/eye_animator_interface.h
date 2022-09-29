#ifndef EYE_ANIMATOR_INTERFACE_H
#define EYE_ANIMATOR_INTERFACE_H

#include <QPointF>
#include "my_types.h"

struct EyeMoveDataStruct
{
  QPointF qpNewPoint;
  QPointF qpCurrentPoint;
};

struct EyeSizeDataStruct
{
  EyeSizesStruct sNewSizes;
  EyeSizesStruct sCurrentSizes;
};

struct EyeLidDataStruct
{
  eLidEmotionType eCurrentEmotionState;
  eLidEmotionType eNewEmotionState;

  LidDisplayStateStruct sRigthLidState;
  LidDisplayStateStruct sLeftLidState;
};

class IEyeAnimatorInterface
{
public:
  virtual void Init(){}
  virtual void Stop()=0;
  virtual void Start()=0;
  virtual bool isStoped()=0;
  virtual bool Process()=0;
};

class IMoveAnimatorInterface :public IEyeAnimatorInterface
{
public:
    virtual void SetMoveData(EyeMoveDataStruct* pEyeMoveData)=0;
};

class ISizeAnimatorInterface:public IEyeAnimatorInterface
{
public:
    virtual void SetSizeData(EyeSizeDataStruct* pEyeSizeData)=0;
};


class ILidAnimatorInterface:public IEyeAnimatorInterface
{
public:
    virtual void SetLidData(EyeLidDataStruct* pEyeLidData)=0;
};

#endif // EYE_ANIMATOR_INTERFACE_H
