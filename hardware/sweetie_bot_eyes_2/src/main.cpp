#include "mainwindow.h"
#include <QApplication>
#include <QCommandLineParser>

#include <cleanexit.h>
#include "ros/ros.h"


int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  QApplication::setApplicationVersion(QString("ver 0.1b Build at %1 %2").arg(__DATE__).arg(__TIME__));

  QCommandLineParser parser;
  parser.setApplicationDescription("Eye2 helper");
  parser.addHelpOption();
  parser.addVersionOption();
  // A boolean option with a single name (-d) (-s)
  QCommandLineOption qDisplayOption("d", QCoreApplication::translate("main", "For desktop, small size"));
  parser.addOption(qDisplayOption);
  QCommandLineOption qSendImageOption("s", QCoreApplication::translate("main", "Send eye image to topic eye_image_left/eye_image_rigth"));
  parser.addOption(qSendImageOption);

  // Process the actual command line arguments given by the user
  parser.process(a);
  bool bShowOnDesktop = parser.isSet(qDisplayOption);
  bool bSendImage = parser.isSet(qSendImageOption);

  MainWindow *w=new MainWindow(argc, argv,bShowOnDesktop,bSendImage);
  w->show();
  CleanExit cleanExit;


  return a.exec();
}
