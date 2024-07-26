#include "modbustest.h"
#include "./ui_modbustest.h"
#include "modbus.h"
#include "modbus-rtu.h"
#include "unit-test.h"
#include "modbus-tcp.h"
#include "modbus-version.h"

int Modbus::setComboxDefalutIndex(QComboBox *combox, const QString &str)
{
	int index;
	index = combox->findText(str);
	if (index != -1) { // 确保找到了文本
		combox->setCurrentIndex(index);
		qDebug() << "already find the text:" << str << endl;
	} else {
		// 文本未找到的处理逻辑
		qDebug() << "Given text:" << str << "not found in the ComboBox.";
		return -1;
	}
	return 0;
}

void Modbus::setComboBoxList(void)
{
	// Set data bits list
	for(int i = 0; i < 4; i++) {
		ui->comboBoxDataBits->addItem(QString::number(5 + i));
	}
	QString dataBits = configFile->value(DEFAULT_SERIAL_SECTION_NAME"dataBits").toString();
	setComboxDefalutIndex(ui->comboBoxDataBits, dataBits);

	// Set parity list
	QString parityList[] = {"None", "Odd", "Even", "Mark", "Space"};
	for (int i = 0; i < 5; ++i) {
		ui->comboBoxParity->addItem(parityList[i]);
	}
	QString parity = configFile->value(DEFAULT_SERIAL_SECTION_NAME"parity").toString();
	setComboxDefalutIndex(ui->comboBoxParity, parity);

	// Set stop bits list
	ui->comboBoxStopBits->addItem("1");
	ui->comboBoxStopBits->addItem("2");
	QString stopBits = configFile->value(DEFAULT_SERIAL_SECTION_NAME"stopBits").toString();
	setComboxDefalutIndex(ui->comboBoxStopBits, stopBits);

	// Set flow control list
	QString flowControlList[] = {"None", "Xon/Xoff", "Rts/Cts", "Dsr/Dtr"};
	for (int i = 0; i < 4; ++i) {
		ui->comboBoxFlowControl->addItem(flowControlList[i]);
	}
	QString flowControl = configFile->value(DEFAULT_SERIAL_SECTION_NAME"flowControl").toString();
	setComboxDefalutIndex(ui->comboBoxFlowControl,flowControl );

	// Set speed list
	QString speedList[] = {"1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200", "460800", "921600", "230400"};
	for (int i = 0; i < 11; ++i) {
		ui->comboBoxSpeed->addItem(speedList[i]);
	}
	QString baudRate = configFile->value(DEFAULT_SERIAL_SECTION_NAME"baudRate").toString();
	setComboxDefalutIndex(ui->comboBoxSpeed, baudRate);
}

void Modbus::setSerialParameters()
{

	/* Set serial parameters from comboBox curent text */
	// port
	serial->setPort(QSerialPortInfo(ui->comboBoxPort->currentText()));

	// dataBits
	if(ui->comboBoxDataBits->currentText() == "5") {
		serial->setDataBits(QSerialPort::Data5);
	} else if (ui->comboBoxDataBits->currentText() == "6") {
		serial->setDataBits(QSerialPort::Data6);
	} else if (ui->comboBoxDataBits->currentText() == "7") {
		serial->setDataBits(QSerialPort::Data7);
	} else if (ui->comboBoxDataBits->currentText() == "8") {
		serial->setDataBits(QSerialPort::Data8);
	}

	// parity
	if(ui->comboBoxParity->currentText() == "None") {
		serial->setParity(QSerialPort::NoParity);
	} else if (ui->comboBoxParity->currentText() == "Odd") {
		serial->setParity(QSerialPort::OddParity);
	} else if (ui->comboBoxParity->currentText() == "Even") {
		serial->setParity(QSerialPort::EvenParity);
	} else if (ui->comboBoxParity->currentText() == "Mark") {
		serial->setParity(QSerialPort::MarkParity);
	} else if (ui->comboBoxParity->currentText() == "Space") {
		serial->setParity(QSerialPort::SpaceParity);
	}

	// stopBits
	if(ui->comboBoxStopBits->currentText() == "1") {
		serial->setStopBits(QSerialPort::OneStop);
	} else if (ui->comboBoxStopBits->currentText() == "2") {
		serial->setStopBits(QSerialPort::TwoStop);
	}

	// flowControl
	if(ui->comboBoxFlowControl->currentText() == "None") {
		serial->setFlowControl(QSerialPort::NoFlowControl);
	} else if (ui->comboBoxFlowControl->currentText() == "Xon/Xoff") {
		serial->setFlowControl(QSerialPort::SoftwareControl);
	} else if (ui->comboBoxFlowControl->currentText() == "Rts/Sts") {
		serial->setFlowControl(QSerialPort::HardwareControl);
	} else if (ui->comboBoxFlowControl->currentText() == "Dsr/Dtr") {
		serial->setFlowControl(QSerialPort::HardwareControl);
	}

	// Set baud rate
	if(ui->comboBoxSpeed->currentText() == "1200") {
		serial->setBaudRate(QSerialPort::Baud1200);
	} else if (ui->comboBoxSpeed->currentText() == "2400") {
		serial->setBaudRate(QSerialPort::Baud2400);
	} else if (ui->comboBoxSpeed->currentText() == "4800") {
		serial->setBaudRate(QSerialPort::Baud4800);
	} else if (ui->comboBoxSpeed->currentText() == "9600") {
		serial->setBaudRate(QSerialPort::Baud9600);
	} else if (ui->comboBoxSpeed->currentText() == "19200") {
		serial->setBaudRate(QSerialPort::Baud19200);
	} else if (ui->comboBoxSpeed->currentText() == "38400") {
		serial->setBaudRate(QSerialPort::Baud38400);
	} else if (ui->comboBoxSpeed->currentText() == "57600") {
		serial->setBaudRate(QSerialPort::Baud57600);
	} else if (ui->comboBoxSpeed->currentText() == "115200") {
		serial->setBaudRate(QSerialPort::Baud115200);
	} else if (ui->comboBoxSpeed->currentText() == "460800") {
		serial->setBaudRate(460800);
	} else if (ui->comboBoxSpeed->currentText() == "921600") {
		serial->setBaudRate(921600);
	} else if (ui->comboBoxSpeed->currentText() == "230400") {
		serial->setBaudRate(230400);
	}

	qDebug() << "dataBits:" << serial->dataBits();
	qDebug() << "parity:" << serial->parity();
	qDebug() << "stopBits:" << serial->stopBits();
	qDebug() << "flowControl:" << serial->flowControl();
	qDebug() << "baudRate:" << serial->baudRate() << endl;
}

Modbus::Modbus(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::Modbus)
{
	ui->setupUi(this);

	/* Set comboBox value list */
	QList<QSerialPortInfo> list = QSerialPortInfo::availablePorts();
	for(int i = 0; i < list.size(); i++) {
		ui->comboBoxPort->addItem(list.at(i).portName());
	}

	/* Read config files and set defaut parameters for serial */
	serial = new QSerialPort;
	configFile = new QSettings(CONFIG_FILE, QSettings::IniFormat);
	setComboBoxList();

	/* Set modbus protocol to radio button */
	QString protocol = configFile->value(MODBUS_PROTOCOL"Protocol").toString();
	if (protocol == "RTU")
		ui->radioButtonRtu->setChecked(true);
	else if (protocol == "TCP")
		ui->radioButtonTcp->setChecked(true);
}

Modbus::~Modbus()
{
	delete configFile;
	delete serial;
	delete ui;
}

void Modbus::on_btnApplyConfig_clicked()
{
	QString strDataBit =ui->comboBoxDataBits->currentText();
	QString strParit =ui->comboBoxParity->currentText();
	QString strStopBit =ui->comboBoxStopBits->currentText();
	QString strFlowControl =ui->comboBoxFlowControl->currentText();
	QString strSpeed =ui->comboBoxSpeed->currentText();
	qDebug() << "set DataBit:" << strDataBit;
	qDebug() << "set Parit:" << strParit;
	qDebug() << "set StopBits:" << strStopBit;
	qDebug() << "set FlowControl:" << strFlowControl;
	qDebug() << "set boadRate:" << strSpeed << endl;
	configFile->setValue(DEFAULT_SERIAL_SECTION_NAME"dataBits", strDataBit);
	configFile->setValue(DEFAULT_SERIAL_SECTION_NAME"parity", strParit);
	configFile->setValue(DEFAULT_SERIAL_SECTION_NAME"stopBits", strStopBit);
	configFile->setValue(DEFAULT_SERIAL_SECTION_NAME"flowControl", strFlowControl);
	configFile->setValue(DEFAULT_SERIAL_SECTION_NAME"baudRate", strSpeed);
}

#define BUG_REPORT(_cond, _format, _args...) \
    printf(                                  \
	"\nLine %d: assertion error for '%s': " _format "\n", __LINE__, #_cond, ##_args)

#define ASSERT_TRUE(_cond, _format, __args...)    \
    {                                             \
	if (_cond) {                              \
	    printf("OK\n");                       \
	} else {                                  \
	    BUG_REPORT(_cond, _format, ##__args); \
	    goto close;                           \
	}                                         \
    };

void Modbus::on_btnStart_clicked()
{
	setSerialParameters();

	modbus_new_rtu("COM4", 115200, 'N', 8, 1);
	/* Length of report slave ID response slave ID + ON/OFF + 'LMB' + version */
	const int NB_REPORT_SLAVE_ID = 2 + 3 + strlen(LIBMODBUS_VERSION_STRING);
	uint8_t *tab_rp_bits = NULL;
	uint16_t *tab_rp_registers = NULL;
	uint16_t *tab_rp_registers_bad = NULL;
	modbus_t *ctx = NULL;
	int i;
	uint8_t value;
	int nb_points;
	int rc;
	float real;
	uint32_t old_response_to_sec;
	uint32_t old_response_to_usec;
	uint32_t new_response_to_sec;
	uint32_t new_response_to_usec;
	uint32_t old_byte_to_sec;
	uint32_t old_byte_to_usec;
	int use_backend;
	int success = FALSE;
	int old_slave;
	char *ip_or_device;

	if (argc > 1) {
		if (strcmp(argv[1], "tcp") == 0) {
			use_backend = TCP;
		} else if (strcmp(argv[1], "tcppi") == 0) {
			use_backend = TCP_PI;
		} else if (strcmp(argv[1], "rtu") == 0) {
			use_backend = RTU;
		} else {
			printf("Modbus client for unit testing\n");
			printf("Usage:\n  %s [tcp|tcppi|rtu]\n", argv[0]);
			printf("Eg. tcp 127.0.0.1 or rtu /dev/ttyUSB1\n\n");
			exit(1);
		}
	} else {
		/* By default */
		use_backend = TCP;
	}

	//modbus_set_slave(ctx, 0x11);
	//modbus_set_debug(ctx, true);
}

void Modbus::on_radioButtonRtu_clicked()
{
	configFile->setValue(MODBUS_PROTOCOL"Protocol", "RTU");
}

void Modbus::on_radioButtonTcp_clicked()
{
	configFile->setValue(MODBUS_PROTOCOL"Protocol", "TCP");
}

