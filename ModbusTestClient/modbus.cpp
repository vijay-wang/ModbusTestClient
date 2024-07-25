#include "modbus.h"
#include "./ui_modbus.h"

Modbus::Modbus(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::Modbus)
{
    ui->setupUi(this);
}

Modbus::~Modbus()
{
    delete ui;
}
