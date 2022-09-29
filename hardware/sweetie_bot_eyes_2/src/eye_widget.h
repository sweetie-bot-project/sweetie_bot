#ifndef EYEWIDGET_H
#define EYEWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShader>
#include <QOpenGLFramebufferObject>
#include <QScreen>
#include <QPixmap>

#include "eye_painter.h"
#include "eye_lid_painter.h"
#include "eye_lid_param_preparer.h"
#include "eye_animator_interface.h"


class EyeWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
  Q_OBJECT
public:
  explicit EyeWidget(QWidget *parent = nullptr, bool bIsLeftEye_=false, bool bShowOnDesktop=false, bool bSendImage=false);
  ~EyeWidget();
  void initializeGL();
  void paintGL();

private:
  int iFrameCounter=0;
  bool bDesktopMode;
  QOpenGLFunctions *m_F;
  bool bIsLeftEye;

  EyePainter* pEyePainter;
  EyeLidPainter* pEyeLidPainter;
  EyeLidParamPreparer *pEyeLidParamPreparer;


  QOpenGLShader *ver;
  QOpenGLShader *frg;
  QOpenGLShaderProgram *prog;

  QColor myColor;
  QMatrix4x4 pmvMatrix;
  QMatrix4x4 tx_matrix;
  QMatrix4x4 tranformMatrix;
  QMatrix4x4 rotateMatrix;

  bool bNeedRepaintFlag=false;
  bool bNeedSendImage=false;
  EyeParametrStruct sCurEyeParam;  // хранит размер и положение глаза

signals:
  void SendEyeImage(QImage* pImage);
public slots:
  void SetEyePeram(EyeParametrStruct* pParam);
  void SetEyeName(QString* sName);
  void SetLidState(LidDisplayStateStruct* pLidDisplayState);
  //  void SetEffect(QString* sEffectName);
  void NewSetLidParamCommand (QString* psCommand); // нужен только для настройки век
  void timer_clk();

};

#endif // EYEWIDGET_H
