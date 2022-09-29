#ifndef EYE_ANIMATION_MANAGER_H
#define EYE_ANIMATION_MANAGER_H

#include <QTimer>
#include "eye_mover_normal.h"
#include "eye_mover_roll.h"
#include "lid_animator_default.h"
#include "lid_animation_blink.h"
#include "lid_amimation_roll.h"
#include "lid_animation_close.h"
#include "eye_sizer_normal.h"
#include "eye_sizer_limb_shake.h"
#include "my_types.h"
#include <iostream>

// Из ROS приходят координаты от pi/2 до -pi/2,
// а экранные координаты от 1 до -1.
// TRANSFORM_COEF - пересчитывает приходящие координаты в экранные
#define EAM_TRANSFORM_COEF  (1/1.57)

struct AnimatorToggleStruct
{
  IEyeAnimatorInterface* pNew;
  IEyeAnimatorInterface* pCurrent;
  IEyeAnimatorInterface* pDefault;
  bool bNeedStartNew=false;
};

class EyeAnimationManager : public QObject
{
  Q_OBJECT
public:
  ~EyeAnimationManager();
  explicit EyeAnimationManager(QObject *parent = nullptr);
  void SetNewCommand(QString* sNewCommand);
  // void SetEyeLid(bool bIsLeft, IEyeLidInterface* pEyeLid);

private:
  EyeMoverNormal* pEyeNormalMover;
  EyeMoverRoll* pEyeRollMover;
  LidAnimatorDefault* pLidAnimDefault;
  LidAnimationBlink* pLidAnimBlink;
  LidAmimationRoll* pLidAnimRoll;
  LidAnimationClose* pLidAnimClose;
  EyeSizerNormal* pEyeSizerNormal;
  EyeSizerLimbShake* pEyeSizerLimbShake;

  bool bGetNewAnimator=false;

  AnimatorToggleStruct aMove;
  AnimatorToggleStruct aSize;
  AnimatorToggleStruct aLid;
  void PrepareNewAnimator(AnimatorToggleStruct *pAnimatorStruct, IEyeAnimatorInterface *pNewAnimator);
  bool ProccesAnimator(AnimatorToggleStruct *pAnimatorStruct);
  void StartNewAnimator(AnimatorToggleStruct *pAnimatorStruct);


  QTimer* pTimer;
  EyeMoveDataStruct sMoveData;
  EyeSizeDataStruct sSizeData;
  EyeLidDataStruct sLidData;

  EyeParametrStruct sEyeParam;
  //  IEyeLidInterface* pLeftEyeLid;
  //  IEyeLidInterface* pRigthEyeLid;
  //  bool bNeedLidStart;

signals:
  void EyeParamSend(EyeParametrStruct* pEyeParametrs);
  void EyeFigureName(QString* sFigName);
  void EyeLeftLidStateSend(LidDisplayStateStruct* pLidDisplayState);
  void EyeRigthLidStateSend(LidDisplayStateStruct* pLidDisplayState);


public slots:
  void NewPitch(double p);
  void NewYaw(double y);
  void NewSetParamCommand (QString* psCommand);
  void NewAnimationMode (QString* psAnimationMode);
  void NewEmotionCommand(QString* psCommand);
  void timer_clk();

private slots:

};

#endif // EYE_ANIMATION_MANAGER_H
