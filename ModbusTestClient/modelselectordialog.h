// modelselectordialog.h
#ifndef MODELSELECTORDIALOG_H
#define MODELSELECTORDIALOG_H

#include <QDialog>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <qmessagebox.h>

enum model {
    OTHERS,
    UTI589A,
};

class ModelSelectorDialog : public QDialog {
    Q_OBJECT
public:
    explicit ModelSelectorDialog(const QStringList &models, QWidget *parent = nullptr);
    QString selectedModel();  // 返回用户选择的型号

private slots:
    void onItemDoubleClicked(QListWidgetItem *item);  // 双击选择

private:
    QListWidget *modelList;
    QString selectedModel_;
};

#endif // MODELSELECTORDIALOG_H
