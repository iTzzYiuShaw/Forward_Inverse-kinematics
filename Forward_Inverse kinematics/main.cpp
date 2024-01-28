#include "Forward_Inversekinematics.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Forward_Inversekinematics w;
    w.show();
    return a.exec();
}
