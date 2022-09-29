#include "cleanexit.h"

static CleanExit *pCleanExit;

CleanExit::CleanExit(QObject *parent ) : QObject(parent)
{
  pCleanExit = this;
  signal(SIGINT, &(CleanExit::exitQt));
  signal(SIGTERM,&(CleanExit::exitQt));
  signal(SIGTSTP,&(CleanExit::exitQt));
  signal(SIGSTOP,&(CleanExit::exitQt));
  signal(SIGABRT,&(CleanExit::exitQt));
  signal(SIGKILL,&(CleanExit::exitQt));
  signal(SIGQUIT,&(CleanExit::exitQt));
  signal(SIGHUP ,&(CleanExit::exitQt));
}

void CleanExit::exitQt(int sig)
{
  //  qDebug() << "TEST TEST";
  switch (sig) {
  case SIGINT:
    qDebug() << "Shutdown application CTRL+C.";
    break;
  case SIGTERM:
    qDebug() << "Shutdown application SIGTERM.";
    break;
  case SIGTSTP:
    qDebug() << "Shutdown application CTRL+Z.";
    break;
  case SIGSTOP:
    qDebug() << "Shutdown application SIGSTOP.";
    break;
  default:
    qDebug() << "Shutdown application unknow reason";
    break;
  }
  QCoreApplication::exit(0);
}
