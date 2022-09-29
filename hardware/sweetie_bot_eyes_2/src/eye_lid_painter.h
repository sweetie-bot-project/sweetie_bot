#ifndef EYELIDPAINTER_H
#define EYELIDPAINTER_H
#include <QString>
#include <QOpenGLTexture>
#include <QOpenGLShader>
#include <QOpenGLFunctions>

#include <ros/ros.h>
#include <ros/package.h>

#include "my_types.h"




struct sLidElementStruct
{
  QOpenGLTexture*  pTexture;
  QMatrix4x4 mLidBaseMat;
  QMatrix4x4 mLidTransMat;
};

class EyeLidPainter: protected QOpenGLFunctions
{
  // Q_OBJECT
public:
  EyeLidPainter(bool bLidIsLeft=true);
  void Init();
  void Paint(QOpenGLShaderProgram* pProg);
  void RecalculateTranformMarix(const LidPaintPropertyStruct* pLidPaintProperty);

private:
  // LidPaintPropertyStruct sLidDefaultProperty;
  bool bIsLeftEye;

  void CreateElementFromFile(QString *sFileName);
  QList<sLidElementStruct> qElementList;
  QVector<QVector2D> qTexVertex;
  QVector<QVector2D> qPosVertex;


};

#endif // EYELIDPAINTER_H
