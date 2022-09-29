#include "eye_image_loader.h"

EyeImageLoader::EyeImageLoader()
{
  sSettingsFileName =  QString::fromStdString( ros::package::getPath("sweetie_bot_eyes_2") );
  sSettingsFileName+=EYE_SETTING_FILE;
  pSettings=new QSettings(sSettingsFileName,QSettings::IniFormat);
  sEyeFigureName="";
  /* sEyeFigureName=EYE_FIG_DEFAULT_NAME;
 LoadImageFromFile();
 ResizeAll();*/
}
EyeImageLoader::~EyeImageLoader()
{
  delete pSettings;

}


void EyeImageLoader::LoadImageFromFile()
{
  //if (pSettings==nullptr)return;
  QString s;
  //-----------------------------------------------------------------
  // проверка наличия файла настроек, его доступности и формата
  if (!(QFileInfo::exists(sSettingsFileName))) // file not found
  {
    PrintMsg(sSettingsFileName+" file not found");
    return;
  }
  if (pSettings->status()!=QSettings::NoError)//AccessError or FormatError
  {
    s=EYE_SETTING_FILE;
    if (pSettings->status()==QSettings::AccessError) PrintMsg(s+" AccessError");
    if (pSettings->status()==QSettings::FormatError) PrintMsg(s+" FormatError");
    return;
  }

  qlOrigImage.clear();
  EyeImageStruct strEyeImage;
  QString sFileName;
  QStringList slParts=pSettings->childGroups();
  //------------------------------------------------------------------
  // Загружаем настройки. Если имя конфигурации sEyeFigureName на найдено
  // то берем по умолчанию. Если нет по умолчанию - загружается дефолтная
  // картинка с крестом
  if (slParts.contains(sEyeFigureName))
  {
    pSettings->beginGroup(sEyeFigureName);
  }
  else {
    PrintMsg("Eye fig "+ sEyeFigureName+" not found, load default");
    if (slParts.contains(EYE_FIG_DEFAULT_NAME))
    {
      pSettings->beginGroup(EYE_FIG_DEFAULT_NAME);
    }
    else {
      s=EYE_FIG_DEFAULT_NAME;
      PrintMsg("Eye fig "+ s+" not found");
      FillDefaultImageStruct(&strEyeImage);
      qlOrigImage.push_back(strEyeImage);
      return;
    }
  }
  //-------------------------------------------------------
  //Настройки найдены, загружаем данные и картинку
  slParts=pSettings->childGroups();
  QString path_;
  QImage* pImage;
  int iMaxWidth;
  path_ =  QString::fromStdString( ros::package::getPath("sweetie_bot_eyes_2") );
  for (int i = 0; i < slParts.size(); ++i)
  {
    pSettings->beginGroup(slParts[i]);
    sFileName=pSettings->value(EYE_PART_FILE_NAME).toString();
    if (QFileInfo::exists(path_+sFileName)) // file found
    {
      pImage=new QImage(path_+sFileName);
      if (pImage==nullptr){
        PrintMsg(path_+sFileName+" file problem");
        return;
      }
      iMaxWidth=pSettings->value(EYE_PART_MAX_WIDTH,"0").toInt();
      strEyeImage.eyeImage=pImage->scaledToWidth(iMaxWidth,Qt::SmoothTransformation);
      strEyeImage.eyePartType=GetEyeTypeFromString(slParts[i]);
      strEyeImage.qpOffset.setX(pSettings->value(EYE_PART_CENTER_X,"0").toInt());
      strEyeImage.qpOffset.setY(pSettings->value(EYE_PART_CENTER_Y,"0").toInt());
      delete pImage;
    }
    else {    // file not found
      PrintMsg(path_+sFileName+" file not found");
      FillDefaultImageStruct(&strEyeImage);
    }
    qlOrigImage.push_back(strEyeImage);
    pSettings->endGroup();
  }
  pSettings->endGroup();
}

/****************************************************************
 * FillDefaultImageStruct - В качестве элемента глаза используется
 *  картинка - черный крест 200х200 пикс
 * ********************************************************/
void EyeImageLoader::FillDefaultImageStruct(EyeImageStruct *s)
{
  QImage* myPix=new QImage (200,200,QImage::Format_ARGB32);
  myPix->fill(qRgba(255, 255, 255, 50));

  QPainter* pPainter=new QPainter(myPix);
  QPen myPen(Qt::darkRed, 5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
  pPainter->setPen(myPen);
  pPainter->drawLine(0,0,200,200);
  pPainter->drawLine(0,200,200,0);

  s->eyeImage=*myPix;
  s->eyePartType=EP_NONE; //"Image not found";
  delete pPainter;
  delete myPix;
}

bool EyeImageLoader::SetEyeFigureName(QString *psFigName)
{
  //if (sEyeFigureName==*psFigName) return false;
  sEyeFigureName=*psFigName;
  LoadImageFromFile();
  return true;
}

QList<EyeImageStruct>* EyeImageLoader::GetPartsStructList()
{
  return &qlOrigImage;
}

eEyePartType EyeImageLoader::GetEyeTypeFromString(QString sName)
{
  if (sName==EYE_PART_BACKGROUNG)return EP_BACKGROUNG;
  if (sName==EYE_PART_LIMB)return EP_LIMB;
  if (sName==EYE_PART_IRIS)return EP_IRIS;
  if (sName==EYE_PART_LIGTH)return EP_LIGTH;
  return EP_NONE;
}

void EyeImageLoader::PrintMsg(const QString &s)
{
  std::cout << s.toStdString()<<  std::endl;
}


