#ifndef CLEANEXIT_H
#define CLEANEXIT_H

#include <QObject>
#include <QCoreApplication>
#include <QDebug>
#include <csignal>

class CleanExit : public QObject
{
  Q_OBJECT

public:

  explicit CleanExit(QObject *parent = 0);

private:

  static void exitQt(int sig);
};

#endif // CLEANEXIT_H
