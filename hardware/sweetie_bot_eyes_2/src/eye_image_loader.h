#ifndef EYEIMAGELOADER_H
#define EYEIMAGELOADER_H

#include <QPixmap>
#include <QPainter>
#include <QImage>
#include <QFileInfo>

#include <ros/ros.h>
#include <ros/package.h>
#include <QSettings>

#include "my_types.h"

#define EYE_PART_FILE_NAME "FileName"
#define EYE_PART_MAX_WIDTH "MaxWidth"
#define EYE_PART_MIN_WIDTH "MinWidth"
#define EYE_PART_CENTER_X "CenterX"
#define EYE_PART_CENTER_Y "CenterY"

#define EYE_PART_BACKGROUNG "Background"
#define EYE_PART_LIMB "Limb"
#define EYE_PART_IRIS "Iris"
#define EYE_PART_LIGTH "Ligth"

#define EYE_SETTING_FILE "/images/eye_figure.ini"

class EyeImageLoader
{
public:
  EyeImageLoader();
  ~EyeImageLoader();
  QList<EyeImageStruct>* GetPartsStructList();
  bool SetEyeFigureName(QString *psFigName);
private:
  void FillDefaultImageStruct(EyeImageStruct* s);
  void LoadImageFromFile();
  eEyePartType GetEyeTypeFromString(QString sName);
  void PrintMsg(const QString &s);
  QString sEyeFigureName;
  QList<EyeImageStruct> qlOrigImage;
  QSettings *pSettings;
  QString sSettingsFileName;
};

static bool Im_Struct_Compare(const EyeImageStruct& first,const EyeImageStruct& second);

#endif // EYEIMAGELOADER_H
