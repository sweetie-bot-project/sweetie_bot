#include "eye_painter.h"

EyePainter::EyePainter(bool bEyeIsLeft)
{
  bIsLeftEye=bEyeIsLeft;
}

void EyePainter::Init(QString *sName)
{
  initializeOpenGLFunctions();
  LoadTextureList(sName);
}

void EyePainter::Paint(QOpenGLShaderProgram* pProg)
{
  // загружаем координаты текстур и полигонов
  int positionLocation = pProg->attributeLocation("position");
  int tex_cordLocation = pProg->attributeLocation("tex_cord");
  pProg->setAttributeArray(positionLocation, qPosVertex.data());
  pProg->setAttributeArray(tex_cordLocation, qTexVertex.data());
  //------------------------------------------------------

  int trans_matrixLocation = pProg->uniformLocation("trans_matrix");
  for (int i = 0; i < qTextList.size(); ++i)
  {
    qTextList[i].pTexture->bind();
    pProg->setUniformValue (trans_matrixLocation, qTextList[i].mPartTranformMat);

    glDrawArrays(GL_TRIANGLE_STRIP, i*4, 4);

    qTextList[i].pTexture->release();
  }
}

void EyePainter::LoadTextureList(QString *sName)
{
  // Удаляем старые текстуры
  EyeTextureStruct myTestSruct;
  while (!qTextList.isEmpty())
  {
    myTestSruct=qTextList.front();
    myTestSruct.pTexture->release();
    myTestSruct.pTexture->destroy();
    delete myTestSruct.pTexture;
    qTextList.pop_front();
  }
  qTexVertex.clear();
  qPosVertex.clear();

  // загружаем картинки из файла
  cImageLoader.SetEyeFigureName(sName);
  QList<EyeImageStruct> *imList=cImageLoader.GetPartsStructList();
  std::sort(imList->begin(),imList->end(),Im_Struct_Compare);

  // создаем текстуры
  EyeImageStruct* pImageStruct;
  QOpenGLTexture *texture;
  QVector2D vNewVertex;
  for (int i = 0; i < imList->size(); ++i)
  {
    pImageStruct=&(*imList)[i];
    texture = new QOpenGLTexture(QOpenGLTexture::Target2D);
    texture->setFormat(QOpenGLTexture::RGBAFormat);
    if((pImageStruct->eyePartType==EP_BACKGROUNG)&(bIsLeftEye)) // фон для левого глаза зеркалим
    {
      texture->setData(pImageStruct->eyeImage.mirrored(true,true),QOpenGLTexture::DontGenerateMipMaps);
    }
    else  texture->setData(pImageStruct->eyeImage.mirrored(false,true),QOpenGLTexture::DontGenerateMipMaps);
    texture->setMagnificationFilter(QOpenGLTexture::Linear);
    myTestSruct.pTexture=texture;
    myTestSruct.ePartType=pImageStruct->eyePartType;
    myTestSruct.qpOffset=pImageStruct->qpOffset;
    myTestSruct.mPartTranformMat.setToIdentity();
    qTextList.push_back(myTestSruct);
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
    float w_im=pImageStruct->eyeImage.width();
    float h_im=pImageStruct->eyeImage.height();


    float x_offset=pImageStruct->eyeImage.width()-pImageStruct->qpOffset.x()*2;
    float y_offset=pImageStruct->eyeImage.height()-pImageStruct->qpOffset.y()*2;


    float x_pos=(-1)*w_im;
    float y_pos=(-1)*h_im;
    x_pos=x_pos+x_offset;
    y_pos=y_pos-y_offset;

    float fX=x_pos/EW_SCREEN_WIDTH;
    float fY=y_pos/EW_SCREEN_HEIGHT;
    float fW=(w_im+x_offset)/EW_SCREEN_WIDTH;
    float fH=(h_im-y_offset)/EW_SCREEN_HEIGHT;
    vNewVertex=QVector2D(fX,fY);
    qPosVertex.push_back(vNewVertex);
    vNewVertex=QVector2D(fX,fH);
    qPosVertex.push_back(vNewVertex);
    vNewVertex=QVector2D(fW,fY);
    qPosVertex.push_back(vNewVertex);
    vNewVertex=QVector2D(fW,fH);
    qPosVertex.push_back(vNewVertex);
  }
}

/********************* RefreshTranformMat *********************
 * Обновляет матрицы перемещения и размера для каждого элемента
 * ************************************************************/
void EyePainter::RefreshTranformMat(EyeParametrStruct *pParam)
{
  QPointF pointIris;
  QPointF pointLimbAndLigth; //смещение зрачка и бликов относительно радужки при взгляде вбок вверх/вниз
  pointIris=pParam->qpEyePos;
  pointIris.setY(pointIris.y()*(-1));
  pointLimbAndLigth=pointIris*(1+(pParam->tEyeSizes.qrIrisSize/5));
  // размер зрачка зависит от размера радужки
  qreal qrLimbSize=pParam->tEyeSizes.qrLimbSize*(pParam->tEyeSizes.qrIrisSize);
  for (int i = 0; i < qTextList.size(); ++i)
  {
    switch (qTextList[i].ePartType) {
    case EP_LIMB:
      qTextList[i].mPartTranformMat.setToIdentity();
      qTextList[i].mPartTranformMat.translate(pointLimbAndLigth.x(),pointLimbAndLigth.y());
      qTextList[i].mPartTranformMat.scale(qrLimbSize);
      break;
    case EP_IRIS:
      qTextList[i].mPartTranformMat.setToIdentity();
      qTextList[i].mPartTranformMat.translate(pointIris.x(),pointIris.y());
      qTextList[i].mPartTranformMat.scale(pParam->tEyeSizes.qrIrisSize);
      break;
    case EP_LIGTH:
      qTextList[i].mPartTranformMat.setToIdentity();
      qTextList[i].mPartTranformMat.translate(pointLimbAndLigth.x(),pointLimbAndLigth.y());
      qTextList[i].mPartTranformMat.scale(pParam->tEyeSizes.qrLigtSize);
      break;
    }
  }
}

// сортировка списка текстур по типу. сначала фон, в конце блики и остальное
static bool Im_Struct_Compare(const EyeImageStruct& first,const EyeImageStruct& second)
{
  return (first.eyePartType<second.eyePartType);
}
