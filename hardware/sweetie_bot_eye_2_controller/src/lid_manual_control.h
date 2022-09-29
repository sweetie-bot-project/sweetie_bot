#ifndef LIDMANUALCONTROL_H
#define LIDMANUALCONTROL_H

#include <QWidget>



namespace Ui {
class LidManualControl;
}

class LidManualControl : public QWidget
{
  Q_OBJECT
public:
  explicit LidManualControl(QWidget *parent = nullptr);
  // ~LidManualControl();
  void SetLidUpOrDown(bool bIsUp);
  void SendAllData();
private:
  bool bLidIsUp=true;
  Ui::LidManualControl *ui;
  void PrepareCommand(std::string sType, std::string sCommand);


signals:

  void SendCommand(std::string sType, std::string sCommand);

private slots:
  void on_vsStartA_valueChanged(int value);
  void on_vsStartX_valueChanged(int value);
  void on_vsStartY_valueChanged(int value);
  void on_vsEndA_valueChanged(int value);
  void on_vsEndX_valueChanged(int value);
  void on_vsEndY_valueChanged(int value);
};

#endif // LIDMANUALCONTROL_H
