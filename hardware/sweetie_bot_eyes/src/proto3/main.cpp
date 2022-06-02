#include "mainwindow.h"
#include <QApplication>

#include "ros/ros.h"

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);

    ros::init(argc, argv, "eyes");

    // @Note: I've decided to put everything related to both eyes into
    //        window class, as right now I'm not going to take the job of
    //        separating their implementation from Qt interface. As behaviour
    //        entangled with EyeWindow, it'll require redo of whole system structure
    MainWindow w;
    w.show();

    return a.exec();
}

