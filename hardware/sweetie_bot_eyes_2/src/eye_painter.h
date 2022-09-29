#ifndef EYEPAINTER_H
#define EYEPAINTER_H
#include <QString>
#include <QOpenGLTexture>
#include <QOpenGLShader>
#include <QOpenGLFunctions>

#include "eye_image_loader.h"
#include "my_types.h"

struct EyeTextureStruct
{
  QOpenGLTexture*  pTexture;
  eEyePartType ePartType;
  QPointF qpOffset;
  QMatrix4x4 mPartTranformMat;
};

class EyePainter: protected QOpenGLFunctions
{
public:
  EyePainter(bool bEyeIsLeft=true);
  void Init(QString *sName);
  void LoadTextureList(QString *sName);
  void Paint(QOpenGLShaderProgram* pProg);
  void RefreshTranformMat(EyeParametrStruct *pParam);
private:
  bool bIsLeftEye;
  EyeImageLoader cImageLoader;

  QList<EyeTextureStruct> qTextList;
  QVector<QVector2D> qTexVertex;
  QVector<QVector2D> qPosVertex;

};

#endif // EYEPAINTER_H
