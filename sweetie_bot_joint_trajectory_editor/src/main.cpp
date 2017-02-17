#include "trajectoryeditor.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    TrajectoryEditor *w = new TrajectoryEditor(argc, argv);
    w->show();

    return a.exec();
}
