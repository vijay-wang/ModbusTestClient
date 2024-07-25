#include "modbus.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Modbus w;
    w.show();
    return a.exec();
}
