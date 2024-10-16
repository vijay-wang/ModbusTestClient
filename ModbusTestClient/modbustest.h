#ifndef MODBUSTEST_H
#define MODBUSTEST_H

#include <QMainWindow>
#include <QSettings>
//#define QT_NO_DEBUG_OUTPUT
#include <QDebug>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QSettings>
#include <QComboBox>
#include <QStandardItemModel>
#include <QAbstractItemView>
#include <pthread.h>

#define CONFIG_FILE "C:\\Users\\ww107\\Desktop\\share\\ModbusTestClient\\ModbusTestClient\\config.ini"
//#define CONFIG_FILE "config.ini"
#define DEFAULT_SERIAL_SECTION_NAME "/SerialConfig/"
#define MODBUS_PROTOCOL_SECTION_NAME "/Protocol/"
#define IP_SECTION_NAME "/IP/"

enum {
	TCP,
	TCP_PI,
	RTU
};

enum {
	STOP,
	START
};

using namespace Qt;

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
	int setComboxDefalutIndex(QComboBox *combox, const QString &str);
	void setSerialParameters(void);
	void setComboBoxList(void);
	void addLinemessage(QStringList list,int line);
	static void *work_thread_cb(void *arg);

private slots:
	void on_btnApplyConfig_clicked();

	void on_btnStart_clicked();

	void on_radioButtonRtu_clicked();

	void on_radioButtonTcp_clicked();

	void on_btnScanID_clicked();

	void on_btnModifyId_clicked();

private:
	Ui::Modbus *ui;
	QSerialPort *serial;
	QSettings *configFile;
	pthread_t work_thread;
	QStandardItemModel *model;
	int scanSalveId(int id);
	int modifySalveId(int oldId, int newId);
	int status;
};
#endif // MODBUSTEST_H
