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
#include <modbus.h>
#include "modelselectordialog.h"

#ifdef QT_DEBUG
#define CONFIG_FILE "config.ini"
#else
#define CONFIG_FILE "config.ini"
#endif
#define DEFAULT_SERIAL_SECTION_NAME "/SerialConfig/"
#define MODBUS_PROTOCOL_SECTION_NAME "/Protocol/"
#define IP_SECTION_NAME "/IP/"
#define SLAVE_SECTION_NAME "/SLAVE/"
#define MISC_SECTION_NAME "/MISC/"
#define SLAVE_ID_MIN 1
#define SLAVE_ID_MAX 247
#define SCAN_ID_TIMOUT 30000 //us

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
    Modbus(enum model model, QWidget *parent = nullptr);
	~Modbus();
	int setComboxDefalutIndex(QComboBox *combox, const QString &str);
	void setSerialParameters(void);
	void setComboBoxList(void);
	void addLinemessage(QStringList list,int line);
	static void *work_thread_cb(void *arg);
	static void *scan_thread_cb(void *arg);
	int worker(int id, int (*cb)(modbus_t *ctx, uint16_t *tab_rp_registers, void *data, void *out), void *data, void *out);
	int need_additional_data(void);
	void set_additional_data(int state);

private slots:
	void on_btnApplyConfig_clicked();

	void on_btnStart_clicked();

	void on_radioButtonRtu_clicked();

	void on_radioButtonTcp_clicked();

	void on_btnScanID_clicked();

	void on_btnModifyId_clicked();

	void on_btnSetRefTime_clicked();

	void on_btnIdToDisplay_clicked();

	void on_checkBox_stateChanged(int arg1);

private:
    int machineModel;
	Ui::Modbus *ui;
	QSerialPort *serial;
	QSettings *configFile;
	pthread_t work_thread;
	pthread_t scan_thread;
	QStandardItemModel *model;
	int scanSalveId(int id);
	int status;
	int refTime = 1000; // ms
	int idToDisplay = 0;
	modbus_t *ctx = NULL;
	float errorRate = 0.0;
	int additional_data = 0;

};

typedef enum {
    TOOL_TYPE_NONE = -1, // 全图
    TOOL_TYPE_POINT= 0,
    TOOL_TYPE_LINE,
    TOOL_TYPE_RECTANGLE,
    TOOL_TYPE_CIRCLE,
    TOOL_TYPE_SHIELDING,
    TOOL_TYPE_IRREGUALR,
    TOOL_TYPE_AUTO_TRACING,
} tool_type;

#endif // MODBUSTEST_H
