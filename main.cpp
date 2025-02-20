#include "guiMainWindow.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    guiMainWindow w;
    w.show();
    return a.exec();
}
