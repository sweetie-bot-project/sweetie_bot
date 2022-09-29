#include "eye_widget.h"

EyeWidget::EyeWidget(QWidget *parent,bool bIsLeftEye_, bool bShowOnDesktop, bool bSendImage)
  : QOpenGLWidget{parent}
{
  bIsLeftEye=bIsLeftEye_;
  bDesktopMode=bShowOnDesktop;
  bNeedSendImage=bSendImage;
  if (bDesktopMode)setFixedSize(300,300);
  else setFixedSize(800,800);
  pEyePainter=new EyePainter();
  pEyeLidPainter=new EyeLidPainter(bIsLeftEye);
  pEyeLidParamPreparer=new EyeLidParamPreparer();
}

EyeWidget::~EyeWidget()
{

}

void EyeWidget::initializeGL()
{
  this->makeCurrent();
  initializeOpenGLFunctions();

  // glViewport(-150,-150,1100,1100);
  glEnable(GL_BLEND);
  glEnable(GL_TEXTURE_2D);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBlendEquation(GL_FUNC_ADD);

  /********* В зависимости от версии OpenGl нужны разные шейдеры**********/
  // определяем версию OpenGL
  const GLubyte* gl_version = glGetString(GL_VERSION);
  const GLubyte* gl_SL_version = glGetString(GL_SHADING_LANGUAGE_VERSION);

  if (bIsLeftEye)std::cout << "GL_VERSION= "<<gl_version<<  std::endl;
  if (bIsLeftEye) std::cout << "GL_SHADING_LANGUAGE_VERSION= "<<gl_SL_version<<  std::endl;

  QString sVersion=(const char*)gl_version;
  int comPos = sVersion.indexOf("OpenGL ES");

  QString sVertShaderName, sFragShaderName;

  // выбираем шейдеры
  if (comPos==(-1)) // версия не содержит "OpenGL ES"
  {
    if (bIsLeftEye) std::cout << "Use PC shaders "<< std::endl;
    sVertShaderName="/shaders/vert_shader_PC.vert";
    sFragShaderName="/shaders/frag_shader_PC.frag";
  }
  else // в названии версии есть подстрока "OpenGL ES"
  {
    if (bIsLeftEye) std::cout << "Use RasPi shaders "<< std::endl;
    sVertShaderName="/shaders/vert_shader_RasPi.vert";
    sFragShaderName="/shaders/frag_shader_RasPi.frag";
  }

  /******************************************************************************/

  // Создаем шейдеры и линкуем их в программу
  QString sShaderFileName =  QString::fromStdString( ros::package::getPath("sweetie_bot_eyes_2") );
  sShaderFileName+=sVertShaderName;
  ver=new QOpenGLShader (QOpenGLShader::Vertex);
  ver->compileSourceFile(sShaderFileName);
  sShaderFileName =  QString::fromStdString( ros::package::getPath("sweetie_bot_eyes_2") );
  sShaderFileName+=sFragShaderName;
  frg=new QOpenGLShader (QOpenGLShader::Fragment);
  frg->compileSourceFile(sShaderFileName);
  prog=new QOpenGLShaderProgram();
  if (prog->addShader(ver)==false) std::cout << " prog->addShader(ver)==false" <<  std::endl;
  if (prog->addShader(frg)==false) std::cout << " prog->addShader(frg)==false" <<  std::endl;
  if (prog->link()==false) std::cout << " prog->link==false" <<  std::endl;
  if (prog->bind()==false) std::cout << " prog->bind==false" <<  std::endl;

  // включаем массивы атрибутов
  int positionLocation = prog->attributeLocation("position");
  int tex_cordLocation = prog->attributeLocation("tex_cord");
  prog->enableAttributeArray(positionLocation);
  prog->enableAttributeArray(tex_cordLocation);


  // rotateMatrix - поворачивает глаз в соответствии с поворотом экранов.
  // при повороте получаются черные полосы в углах,
  //поэтому изображение немного увеличивается.
  int rot_matrixLocation = prog->uniformLocation("rot_matrix");
  rotateMatrix.setToIdentity();
  float_t fScaleFactor=1.4;
  rotateMatrix.scale(fScaleFactor,fScaleFactor);

  float_t fDisplayRotateAngle=-30;
  if (bIsLeftEye)fDisplayRotateAngle=30;
  if (bDesktopMode) fDisplayRotateAngle=0;
  rotateMatrix.rotate(fDisplayRotateAngle,0,0,1);

  prog->setUniformValue    (rot_matrixLocation, rotateMatrix);

  // загружаются текстуры
  QString sEyeName="Default";
  pEyePainter->Init(&sEyeName);
  pEyeLidPainter->Init();

}


void EyeWidget::paintGL()
{
  pEyePainter->Paint(prog);
  pEyeLidPainter->Paint(prog);
}

void EyeWidget::SetEyeName(QString *sName)
{
  this->makeCurrent();
  pEyePainter->LoadTextureList(sName);
  pEyePainter->RefreshTranformMat(&sCurEyeParam);
  this->update();
}


void EyeWidget::SetEyePeram(EyeParametrStruct *pParam)
{
  //TODO: Проверка тут не нужна. если данные пришли, то они точно изменились.
  // но это нужно протестить
  if (sCurEyeParam.tEyeSizes!=pParam->tEyeSizes) // изменился размер
  {
    bNeedRepaintFlag=true;
    sCurEyeParam.tEyeSizes=pParam->tEyeSizes;
  }
  if (sCurEyeParam.qpEyePos!=pParam->qpEyePos)// изменилось положение
  {
    bNeedRepaintFlag=true;
    sCurEyeParam.qpEyePos=pParam->qpEyePos;
    // std::cout << " sCurEyeParam.qpEyePos= " << pParam->qpEyePos.x()<< " ; "<< pParam->qpEyePos.y() <<  std::endl;
  }
}

//  SetLidState - слот, получает данные век от аниматора, отправлят на подготовку к рисованию
void EyeWidget::SetLidState(LidDisplayStateStruct *pLidDisplayState)
{
  pEyeLidParamPreparer->SetLidEmotion(pLidDisplayState->eEmotionCondition);
  pEyeLidParamPreparer->SetLidBlinkState(pLidDisplayState->fClosePercent);
}

void EyeWidget::timer_clk()
{
  if (bNeedRepaintFlag) // пересчитать положение и размер глаза
  {
    pEyePainter->RefreshTranformMat(&sCurEyeParam);
  }
  if (pEyeLidParamPreparer->Process())// и положение век тоже изменилось
  {
    bNeedRepaintFlag=true;
    pEyeLidPainter->RecalculateTranformMarix(pEyeLidParamPreparer->GetLidPaintProperty());
  }

  if (bNeedRepaintFlag)
  {
    this->update(); // перерисовывается всё
    bNeedRepaintFlag=false;
  }
  // отправка картинки для отображения в RViz
  if (bNeedSendImage)
  {
    iFrameCounter++;
    if (iFrameCounter>3)
    {
      iFrameCounter=0;
      QPixmap my_pixmap=this->grab();
      QImage myImage=my_pixmap.toImage().convertToFormat(QImage::Format_RGBA8888);
      emit SendEyeImage(&myImage);
    }
  }
}

// для настройки век
void EyeWidget::NewSetLidParamCommand(QString *psCommand)
{
  pEyeLidParamPreparer->NewSetLidParamCommand(psCommand);
}


