#include "trajectoryeditor.h"
#include <QApplication>
#include <QLocale>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "trajectory_editor");
    ros::NodeHandle node;

    QLocale::setDefault(QLocale::English);

    QApplication a(argc, argv);
    TrajectoryEditor *w = new TrajectoryEditor(argc, argv, node);
    w->show();

    return a.exec();
}
