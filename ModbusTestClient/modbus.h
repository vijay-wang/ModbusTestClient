#ifndef MODBUS_H
#define MODBUS_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui {
class Modbus;
}
QT_END_NAMESPACE

class Modbus : public QMainWindow
{
    Q_OBJECT

public:
    Modbus(QWidget *parent = nullptr);
    ~Modbus();

private:
    Ui::Modbus *ui;
};
#endif // MODBUS_H
