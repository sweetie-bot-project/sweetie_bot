#include "eye_lid_painter.h"

EyeLidPainter::EyeLidPainter(bool bLidIsLeft)
{
  bIsLeftEye=bLidIsLeft;
}
void EyeLidPainter::Init()
{
  initializeOpenGLFunctions();
  QString path_,sFileName;
  path_ =  QString::fromStdString( ros::package::getPath("sweetie_bot_eyes_2") );
  // -----  верхнее веко ------------
  sFileName=path_+"/images/UpperEyelid.png";
  CreateElementFromFile(&sFileName);
  //-------- нижнее веко ---------------
  sFileName=path_+"/images/LowerEyelid.png";
  CreateElementFromFile(&sFileName);
  //RecalculateTranformMarix();
}

void EyeLidPainter::CreateElementFromFile(QString *sFileName)
{
  QImage* pImage= new QImage(*sFileName);
  sLidElementStruct sEffectElement;

  sEffectElement.pTexture = new QOpenGLTexture(QOpenGLTexture::Target2D);
  sEffectElement.pTexture->setFormat(QOpenGLTexture::RGBAFormat);

  if (bIsLeftEye) sEffectElement.pTexture->setData(pImage->mirrored(false,true),QOpenGLTexture::DontGenerateMipMaps);
  else sEffectElement.pTexture->setData(pImage->mirrored(true,true),QOpenGLTexture::DontGenerateMipMaps);
  sEffectElement.pTexture->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
  sEffectElement.pTexture->setMagnificationFilter(QOpenGLTexture::Linear);

  QVector2D vNewVertex;
  //---- Текстурные координаты------
  vNewVertex=QVector2D(0,0);
  qTexVertex.push_back(vNewVertex);
  vNewVertex=QVector2D(0,1);
  qTexVertex.push_back(vNewVertex);
  vNewVertex=QVector2D(1,0);
  qTexVertex.push_back(vNewVertex);
  vNewVertex=QVector2D(1,1);
  qTexVertex.push_back(vNewVertex);
  //------ координаты вершин полигона ---------
  float w_im=pImage->width();
  float h_im=pImage->height();

  float fX=(-1)*w_im/EW_SCREEN_WIDTH;
  float fY=(-1)*w_im/EW_SCREEN_HEIGHT;
  float fW=w_im/EW_SCREEN_WIDTH;
  float fH=h_im/EW_SCREEN_HEIGHT;

  vNewVertex=QVector2D(fX,fY);
  qPosVertex.push_back(vNewVertex);
  vNewVertex=QVector2D(fX,fH);
  qPosVertex.push_back(vNewVertex);
  vNewVertex=QVector2D(fW,fY);
  qPosVertex.push_back(vNewVertex);
  vNewVertex=QVector2D(fW,fH);
  qPosVertex.push_back(vNewVertex);
  sEffectElement.mLidBaseMat.setToIdentity();
  sEffectElement.mLidBaseMat.scale(1);
  //sEffectElement.mLidBaseMat.rotate(10,0,0,1);
  qElementList.push_back(sEffectElement);
}



void EyeLidPainter::RecalculateTranformMarix(const LidPaintPropertyStruct* pLidPaintProperty)
{
  if (bIsLeftEye)
  {
    // 0 - верхнее веко
    qElementList[0].mLidTransMat= qElementList[0].mLidBaseMat;
    qElementList[0].mLidTransMat.rotate(pLidPaintProperty->sUpLid.fAngle,0,0,1);
    qElementList[0].mLidTransMat.translate(pLidPaintProperty->sUpLid.fPos.x(),pLidPaintProperty->sUpLid.fPos.y());
    // 1- нижнее веко
    qElementList[1].mLidTransMat= qElementList[1].mLidBaseMat;
    qElementList[1].mLidTransMat.rotate(pLidPaintProperty->sDownLid.fAngle,0,0,1);
    qElementList[1].mLidTransMat.translate(pLidPaintProperty->sDownLid.fPos.x(),pLidPaintProperty->sDownLid.fPos.y());
  }
  else // глаз правый
  {
    // 0 - верхнее веко
    qElementList[0].mLidTransMat= qElementList[0].mLidBaseMat;
    qElementList[0].mLidTransMat.rotate(pLidPaintProperty->sUpLid.fAngle*(-1),0,0,1);
    qElementList[0].mLidTransMat.translate(pLidPaintProperty->sUpLid.fPos.x()*(-1),pLidPaintProperty->sUpLid.fPos.y());
    // 1- нижнее веко
    qElementList[1].mLidTransMat= qElementList[1].mLidBaseMat;
    qElementList[1].mLidTransMat.rotate((pLidPaintProperty->sDownLid.fAngle)*(-1),0,0,1);
    qElementList[1].mLidTransMat.translate(pLidPaintProperty->sDownLid.fPos.x()*(-1),pLidPaintProperty->sDownLid.fPos.y());
  }
}

void EyeLidPainter::Paint(QOpenGLShaderProgram* pProg)
{
  // загружаем координаты текстур и полигонов
  int positionLocation = pProg->attributeLocation("position");
  int tex_cordLocation = pProg->attributeLocation("tex_cord");
  pProg->setAttributeArray(positionLocation, qPosVertex.data());
  pProg->setAttributeArray(tex_cordLocation, qTexVertex.data());
  //------------------------------------------------------
  int trans_matrixLocation = pProg->uniformLocation("trans_matrix");
  for (int i = 0; i < qElementList.size(); ++i) {
    pProg->setUniformValue (trans_matrixLocation, qElementList[i].mLidTransMat);
    qElementList[i].pTexture->bind();
    glDrawArrays(GL_TRIANGLE_STRIP, i*4, 4);
    qElementList[i].pTexture->release();
  }

}
