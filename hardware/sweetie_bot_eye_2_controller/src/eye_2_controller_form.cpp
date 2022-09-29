#include "eye_2_controller_form.h"

Eye2ControllerForm::Eye2ControllerForm(int argc, char *argv[], QWidget *parent) :
  QMainWindow(parent)
{
  // setup GUI
  ui.setupUi(this);

  ConnectWidget *pCF1=new ConnectWidget(argc, argv,nullptr);
  ros_subscriber_widget *pRSW=new ros_subscriber_widget(this);
  ros_publisher_widget *pRPW=new ros_publisher_widget(this);


  connect(pCF1, SIGNAL(RosStateChanged(int)), pRSW, SLOT(RosStateChanged(int)));
  connect(pCF1, SIGNAL(RosStateChanged(int)), pRPW, SLOT(RosStateChanged(int)));
  connect(this,SIGNAL(SendCommand(std::string, std::string)),pRPW,SLOT(PublishCommand(std::string, std::string)));

  MouseEyeMoverScene* pScene=new MouseEyeMoverScene(this);
  pScene->setSceneRect(0,0,EYE_VIEW_WIDTH,EYE_VIEW_WIDTH);
  pixmap_item=new QGraphicsPixmapItem;
  pixmap_item->setPos(100,100);
  ui.graphicsView->setScene(pScene);
  pScene->addItem(pixmap_item);

  connect(pScene,SIGNAL(SendPoint(QPointF)),this,SLOT(EyePos(QPointF)));
  connect(pRSW,SIGNAL(GetNewImage(QImage*)),this,SLOT(GetEyeImage(QImage*)));

  statusBar()->addWidget(pCF1);
  statusBar()->addWidget(pRSW);
  statusBar()->addWidget(pRPW);

  /**************************************************************************/
  /**************  виджеты для управления веками ***********************************/
  pUpLidControl=new LidManualControl(this);
  pUpLidControl->SetLidUpOrDown(true);
  pDownLidControl=new LidManualControl(this);
  pDownLidControl->SetLidUpOrDown(false);
  ui.vlUpLidControl->addWidget(pUpLidControl);
  ui.vlUpLidControl->addWidget(pDownLidControl);
  connect(pUpLidControl,SIGNAL(SendCommand(std::string, std::string)),pRPW,SLOT(PublishCommand(std::string, std::string)));
  connect(pDownLidControl,SIGNAL(SendCommand(std::string, std::string)),pRPW,SLOT(PublishCommand(std::string, std::string)));


  /************************************************************************/
  /************  таймер используется только в этом объекте *****/
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(timer_clk()));
  timer->start(30);
}

void Eye2ControllerForm::GetEyeImage(QImage *pImage)
{
  QImage myImage=pImage->scaled(600,600);
  pixmap_item->setPixmap(QPixmap::fromImage(myImage));
}

void Eye2ControllerForm::logAddString(QString s)
{
  slLog.push_back(s);
  if (slLog.size()>1000) slLog.removeFirst();
  ui.textEdit->clear();
  ui.textEdit->append(slLog.join("\n"));
}
void Eye2ControllerForm::timer_clk()
{
  //ui.graphicsView->fitInView(qGScene.sceneRect(),Qt::AspectRatioMode::KeepAspectRatio);
  bTimerElapsedFlag=true;
}

Eye2ControllerForm::~Eye2ControllerForm()
{
  delete timer;
}

void Eye2ControllerForm::on_pushButton_3_clicked()
{
  SendCommand("eyes/set_param", "LimbSize=50");
}

void Eye2ControllerForm::on_horizontalSlider_sliderMoved(int position)
{

}

void Eye2ControllerForm::on_horizontalSlider_valueChanged(int value)
{
  if (!bTimerElapsedFlag) return;
  bTimerElapsedFlag=false;
  QString s="LimbSize=";
  double qrVal=value;
  qrVal=qrVal/100;
  s=s+QString::number(qrVal);
  ui.lbLimbSize->setText(s);
  SendCommand("eyes/set_param", s.toStdString());
}


void Eye2ControllerForm::on_horizontalSlider_2_valueChanged(int value)
{
  if (!bTimerElapsedFlag) return;
  bTimerElapsedFlag=false;
  QString s="IrisSize=";
  double qrVal=value;
  qrVal=qrVal/100;
  s=s+QString::number(qrVal);
  ui.lbIrisSize->setText(s);
  SendCommand("eyes/set_param", s.toStdString());
}


void Eye2ControllerForm::on_pushButton_18_clicked()
{
  QString sType="eyes/set_param";
  QString sCommand="EyeFigureName=Default";
  SendCommand(sType.toStdString(),sCommand.toStdString());
  logAddString(sType+"    "+sCommand);
}


void Eye2ControllerForm::on_pushButton_19_clicked()
{
  QString sType="eyes/set_param";
  QString sCommand="EyeFigureName=Test";
  SendCommand(sType.toStdString(),sCommand.toStdString());
  logAddString(sType+"    "+sCommand);
}


void Eye2ControllerForm::on_pushButton_17_clicked()
{
  QString sType="eyes/set_param";
  QString sCommand="EyeFigureName=TestEff";
  SendCommand(sType.toStdString(),sCommand.toStdString());
  logAddString(sType+"    "+sCommand);
}

void Eye2ControllerForm::on_pbAnimNormal_clicked()
{
  QString sType="eyes/animation_mode";
  QString sCommand="Normal";
  SendCommand(sType.toStdString(),sCommand.toStdString());
  logAddString(sType+"    "+sCommand);
}



void Eye2ControllerForm::on_pbAnimSpin_clicked()
{
  QString sType="eyes/animation_mode";
  QString sCommand="Spin";
  SendCommand(sType.toStdString(),sCommand.toStdString());
  logAddString(sType+"    "+sCommand);
}


void Eye2ControllerForm::on_pbAnimShake_clicked()
{
  QString sType="eyes/animation_mode";
  QString sCommand="Shake";
  SendCommand(sType.toStdString(),sCommand.toStdString());
  logAddString(sType+"    "+sCommand);
}

void Eye2ControllerForm::EyePos(QPointF pos)
{
  if (!bTimerElapsedFlag) return;
  bTimerElapsedFlag=false;
  QString sType="eyes/set_param";
  QString sCommand;
  ui.lbXpos->setText(QString::number(pos.x()));
  ui.lbYpos->setText(QString::number(pos.y()));

  sCommand="Pitch="+QString::number(pos.x());
  SendCommand(sType.toStdString(),sCommand.toStdString());
  sCommand="Yaw="+QString::number(pos.y());
  SendCommand(sType.toStdString(),sCommand.toStdString());
}

void Eye2ControllerForm::on_pbBlink_clicked()
{
  QString sType="eyes/animation_mode";
  QString sCommand="Blink";
  SendCommand(sType.toStdString(),sCommand.toStdString());
  logAddString(sType+"    "+sCommand);
}


void Eye2ControllerForm::on_pbCloseEye_clicked()
{
  QString sType="eyes/animation_mode";
  QString sCommand="CloseEye";
  SendCommand(sType.toStdString(),sCommand.toStdString());
  logAddString(sType+"    "+sCommand);
}



void Eye2ControllerForm::on_pushButton_4_clicked()
{
  pUpLidControl->SendAllData();
  pDownLidControl->SendAllData();
}


void Eye2ControllerForm::on_pbEmNormal_clicked()
{
  QString sType="eyes/emotion";
  QString sCommand="Normal";
  SendCommand(sType.toStdString(),sCommand.toStdString());
  logAddString(sType+"    "+sCommand);
}


void Eye2ControllerForm::on_pbEmShy_clicked()
{
  QString sType="eyes/emotion";
  QString sCommand="Shy";
  SendCommand(sType.toStdString(),sCommand.toStdString());
  logAddString(sType+"    "+sCommand);
}


void Eye2ControllerForm::on_pbEmAngry_clicked()
{
  QString sType="eyes/emotion";
  QString sCommand="Angry";
  SendCommand(sType.toStdString(),sCommand.toStdString());
  logAddString(sType+"    "+sCommand);
}


void Eye2ControllerForm::on_pbEmTired_clicked()
{
  QString sType="eyes/emotion";
  QString sCommand="Tired";
  SendCommand(sType.toStdString(),sCommand.toStdString());
  logAddString(sType+"    "+sCommand);
}


void Eye2ControllerForm::on_pbEmSuspect_clicked()
{
  QString sType="eyes/emotion";
  QString sCommand="Suspect";
  SendCommand(sType.toStdString(),sCommand.toStdString());
  logAddString(sType+"    "+sCommand);
}


void Eye2ControllerForm::on_hsCloseSlider_valueChanged(int value)
{
  QString s="LidClosePercent=";
  double qrVal=value;
  qrVal=qrVal/100;
  s=s+QString::number(qrVal);
  ui.lbClosePercent->setText("ClosePercent="+QString::number(qrVal));


  SendCommand("eyes/set_lid_param",s.toStdString());
}


void Eye2ControllerForm::on_pbAnimNormalSize_clicked()
{
  QString sType="eyes/animation_mode";
  QString sCommand="Size_normal";
  SendCommand(sType.toStdString(),sCommand.toStdString());
  logAddString(sType+"    "+sCommand);
}


void Eye2ControllerForm::on_pbLidNormal_clicked()
{
  QString sType="eyes/animation_mode";
  QString sCommand="LidNormal";
  SendCommand(sType.toStdString(),sCommand.toStdString());
  logAddString(sType+"    "+sCommand);
}


void Eye2ControllerForm::on_pushButton_2_clicked()
{
  QString sType="eyes/emotion";
  QString sCommand="None";
  SendCommand(sType.toStdString(),sCommand.toStdString());
  logAddString(sType+"    "+sCommand);
}

