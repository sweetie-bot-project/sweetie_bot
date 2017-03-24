#include "mainwindow.h"
#include <QApplication>

#include "ros/ros.h"

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    QStringList args = a.arguments();

    bool isRightEye = false;
    if(args.count() >= 2) {
        if(args.at(1) == "-r") {
            isRightEye = true;
        }
    }
    ros::init(argc, argv, "eye");

	MainWindow *w = new MainWindow(isRightEye);
    w->show();

    return a.exec();
}
