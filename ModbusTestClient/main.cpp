#include "modbustest.h"
#include "modelselectordialog.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Modbus *w;
    int r;

    QMap<QString, enum model> modelMap;
    modelMap.insert("others", OTHERS);
    modelMap.insert("uti589a", UTI589A);

    // 型号列表
    QStringList models = {"uti589a", "others"};

    // 创建并显示对话框
    QString machineModel;
    ModelSelectorDialog dialog(models);
    if (dialog.exec() == QDialog::Accepted) {
        machineModel = dialog.selectedModel();
        if (machineModel.isEmpty()) {
            return -1;
        }
    } else
        return -1;

    w = new Modbus(modelMap[machineModel]);
    w->show();
    r = a.exec();
    delete w;
    return  r;
}
