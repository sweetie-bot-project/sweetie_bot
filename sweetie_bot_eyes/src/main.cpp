#include "mainwindow.h"
#include <QApplication>

#include <QDebug>

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    QStringList args = a.arguments();

    bool isRightEye = false;
    if(args.count() >= 2) {
        if(args.at(1) == "-r") {
            isRightEye = true;
        }
    }
	MainWindow *w = new MainWindow(argc, argv, isRightEye);
    w->show();

    return a.exec();
}
