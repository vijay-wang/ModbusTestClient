// modelselectordialog.cpp
#include "modelselectordialog.h"

ModelSelectorDialog::ModelSelectorDialog(const QStringList &models, QWidget *parent)
    : QDialog(parent), selectedModel_("")
{
    // 设置对话框属性
    setWindowTitle("Select Model");
    setModal(true);  // 设置为模态对话框

    // 创建控件
    modelList = new QListWidget(this);
    modelList->addItems(models);  // 添加型号列表

    QPushButton *okButton = new QPushButton("Ok", this);
    QPushButton *cancelButton = new QPushButton("Cancel", this);

    // 布局
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(modelList);
    layout->addWidget(okButton);
    layout->addWidget(cancelButton);
    setLayout(layout);

    // 连接信号与槽
    connect(okButton, &QPushButton::clicked, this, &QDialog::accept);
    connect(cancelButton, &QPushButton::clicked, this, &QDialog::reject);
    connect(modelList, &QListWidget::itemDoubleClicked, this, &ModelSelectorDialog::onItemDoubleClicked);
}

void ModelSelectorDialog::onItemDoubleClicked(QListWidgetItem *item)
{
    selectedModel_ = item->text();
    accept();  // 双击直接确认
}

QString ModelSelectorDialog::selectedModel()
{
    if (modelList->currentItem()) {
        selectedModel_ = modelList->currentItem()->text();
    }
    return selectedModel_;
}
