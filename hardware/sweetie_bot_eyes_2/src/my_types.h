#ifndef MY_TYPES_H
#define MY_TYPES_H

#include <QString>
#include <QPoint>
#include <QPixmap>

#define EW_SCREEN_WIDTH 800
#define EW_SCREEN_HEIGHT 800
//#define FRAME_TIME      30     // Длительность одного кадра в милисекундах. Задает интервал таймера в main


enum eEyePartType
{
  EP_BACKGROUNG,
  EP_IRIS,
  EP_LIMB,
  EP_LIGTH,
  EP_NONE,
};



struct EyePartStruct
{
  QPixmap  imPixmap;
  eEyePartType ePartType;
  //QString sPartName;
  QPointF qpOffset;
  uint iMaxWidth;
};
struct EyeImageStruct
{
  QImage eyeImage;
  eEyePartType eyePartType;
  QPointF qpOffset;
};



#define EYE_FIG_DEFAULT_NAME "Default"

struct EyeSizesStruct
{
  qreal qrIrisSize;
  qreal qrLimbSize;
  qreal qrLigtSize;
  bool operator != (const EyeSizesStruct &sizes) const
  {
    return this->qrIrisSize != sizes.qrIrisSize || this->qrLimbSize != sizes.qrLimbSize || this->qrLigtSize != sizes.qrLigtSize;
  }
  bool operator ==(const EyeSizesStruct &sizes) const
  {
    return this->qrIrisSize == sizes.qrIrisSize && this->qrLimbSize == sizes.qrLimbSize && this->qrLigtSize == sizes.qrLigtSize;
  }
};



struct EyeParametrStruct
{
  QPointF qpEyePos;
  EyeSizesStruct tEyeSizes;
};

enum eLidEmotionType
{
  LF_NORMAL,
  LF_SHY,
  LF_ANGRY,
  LF_TIRED,
  LF_SUSPECT,
  LF_NONE,
  LF_UNKNOW,
};


struct LidDisplayStateStruct
{
  eLidEmotionType eEmotionCondition;
  float fClosePercent;
};

struct LidPaintParamStruct{
  float fAngle;
  QPointF fPos;
};

struct LidPaintPropertyStruct
{
  LidPaintParamStruct sUpLid;
  LidPaintParamStruct sDownLid;
};


#endif // MY_TYPES_H

