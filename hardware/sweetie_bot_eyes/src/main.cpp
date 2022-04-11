#include "mainwindow.h"
#include <QApplication>
#include <QHBoxLayout>

#include "consts.h"

#include "ros/ros.h"

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);

    ros::init(argc, argv, "eyes");

    QWidget w;
    auto right_eye = new MainWindow(false, &w);
    auto left_eye = new MainWindow(true, &w);

    w.setAttribute(Qt::WA_Hover);
    right_eye->setAttribute(Qt::WA_Hover);
    left_eye->setAttribute(Qt::WA_Hover);

    auto hlay = new QHBoxLayout(&w);
    hlay->addWidget(right_eye);
    hlay->addWidget(left_eye);
    hlay->setMargin(0);
    hlay->setSpacing(0);

    w.resize(2 * WIDTH, HEIGHT);
    // w.resize(640,240);
    w.show();

    return a.exec();
}
