#include "eye_2_controller_form.h"
#include <QApplication>
#include "ros_connect_widget.h"
#include "cleanexit.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  Eye2ControllerForm *w = new Eye2ControllerForm(argc, argv);
  w->show();
  CleanExit cleanExit;
  return a.exec();

}
