#include "mainwindow.h"
#include <QApplication>

#include <QDebug>

#include "ros/ros.h"

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    QStringList args = a.arguments();

    bool isLeftEye = false;
    if(args.count() >= 2) {
        if(args.at(1) == "-l") {
            isLeftEye = true;
        }
    }

    ros::init(argc, argv, "eye");

    MainWindow w(isLeftEye);
    w.show();

    return a.exec();
}
