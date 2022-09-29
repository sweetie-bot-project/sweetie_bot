#include "lid_manual_control.h"
#include "ui_lid_manual_control.h"

LidManualControl::LidManualControl(QWidget *parent):
  QWidget(parent),
  ui(new Ui::LidManualControl)
{
  ui->setupUi(this);
}

void LidManualControl::SetLidUpOrDown(bool bIsUp)
{
  bLidIsUp=bIsUp;
}

void LidManualControl::PrepareCommand(std::string sType, std::string sCommand)
{
  if (bLidIsUp) sCommand=sCommand+"=up";
  else sCommand=sCommand+"=down";
  SendCommand(sType,sCommand);
}
void LidManualControl::SendAllData()
{
  on_vsStartA_valueChanged(ui->vsStartA->value());
  on_vsStartX_valueChanged(ui->vsStartX->value());
  on_vsStartY_valueChanged(ui->vsStartY->value());
  on_vsEndA_valueChanged(ui->vsEndA->value());
  on_vsEndX_valueChanged(ui->vsEndX->value());
  on_vsEndY_valueChanged(ui->vsEndY->value());
}

void LidManualControl::on_vsStartA_valueChanged(int value)
{
  QString s="LidStartA=";
  double qrVal=value;
  qrVal=qrVal/10;
  s=s+QString::number(qrVal);
  ui->lbStartA->setText(QString::number(qrVal));
  PrepareCommand("eyes/set_lid_param", s.toStdString());
}


void LidManualControl::on_vsStartX_valueChanged(int value)
{
  QString s="LidStartX=";
  double qrVal=value;
  qrVal=qrVal/100;
  s=s+QString::number(qrVal);
  ui->lbStartX->setText(QString::number(qrVal));
  PrepareCommand("eyes/set_lid_param", s.toStdString());
}


void LidManualControl::on_vsStartY_valueChanged(int value)
{
  QString s="LidStartY=";
  double qrVal=value;
  qrVal=qrVal/100;
  s=s+QString::number(qrVal);
  ui->lbStartY->setText(QString::number(qrVal));
  PrepareCommand("eyes/set_lid_param", s.toStdString());
}


void LidManualControl::on_vsEndA_valueChanged(int value)
{
  QString s="LidEndA=";
  double qrVal=value;
  qrVal=qrVal/10;
  s=s+QString::number(qrVal);
  ui->lbEndA->setText(QString::number(qrVal));
  PrepareCommand("eyes/set_lid_param", s.toStdString());
}


void LidManualControl::on_vsEndX_valueChanged(int value)
{
  QString s="LidEndX=";
  double qrVal=value;
  qrVal=qrVal/100;
  s=s+QString::number(qrVal);
  ui->lbEndX->setText(QString::number(qrVal));
  PrepareCommand("eyes/set_lid_param", s.toStdString());
}


void LidManualControl::on_vsEndY_valueChanged(int value)
{
  QString s="LidEndY=";
  double qrVal=value;
  qrVal=qrVal/100;
  s=s+QString::number(qrVal);
  ui->lbEndY->setText(QString::number(qrVal));
  PrepareCommand("eyes/set_lid_param", s.toStdString());
}

