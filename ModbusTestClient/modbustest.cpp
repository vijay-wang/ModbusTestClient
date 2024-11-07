#include "modbustest.h"
#include "./ui_modbustest.h"
#include "modbus.h"
#include "modbus-rtu.h"
#include "unit-test.h"
#include "modbus-tcp.h"
#include "modbus-version.h"
#include <unistd.h>
#include <QMessageBox>

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
	QString speedList[] = {"600", "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200", "460800", "921600", "230400"};
	for (int i = 0; i < 12; ++i) {
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

static void toggleAllWidgetsInLayout(QLayout *layout, bool _switch) {
	// 遍历布局中的所有项
	for (int i = 0; i < layout->count(); ++i) {
		QLayoutItem *item = layout->itemAt(i);

		// 检查是否是 QWidget
		if (QWidget *widget = item->widget()) {
			widget->setEnabled(_switch);  // 禁用控件
		}

		// 检查是否是子布局
		if (QLayout *childLayout = item->layout()) {
			toggleAllWidgetsInLayout(childLayout, _switch);  // 递归禁用子布局中的控件
		}
	}
}

Modbus::Modbus(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::Modbus)
{
	ui->setupUi(this);
	//setFixedSize(779, 629);
	status = STOP;

	serial = new QSerialPort;
	configFile = new QSettings(CONFIG_FILE, QSettings::IniFormat);

	/* Set comboBox value list */
	QList<QSerialPortInfo> list = QSerialPortInfo::availablePorts();
	for(int i = 0; i < list.size(); i++) {
		ui->comboBoxPort->addItem(list.at(i).portName());
		if(list.at(i).portName() == configFile->value(DEFAULT_SERIAL_SECTION_NAME"port").toString())
			setComboxDefalutIndex(ui->comboBoxPort, list.at(i).portName());
	}

	/* Read config files and set defaut parameters for serial */
	setComboBoxList();

	/* Set modbus protocol to radio button */
	QString protocol = configFile->value(MODBUS_PROTOCOL_SECTION_NAME"Protocol").toString();
	if (protocol == "RTU") {
		ui->radioButtonRtu->setChecked(true);
		toggleAllWidgetsInLayout(ui->vLayoutRTU, true);
		toggleAllWidgetsInLayout(ui->vLayoutIP, false);
	} else if (protocol == "TCP") {
		ui->radioButtonTcp->setChecked(true);
		toggleAllWidgetsInLayout(ui->vLayoutRTU, false);
		toggleAllWidgetsInLayout(ui->vLayoutIP, true);
	}

	/* Set default IP */
	QString ip = configFile->value(IP_SECTION_NAME"IP").toString();
	ui->lineEditIp->setText(ip);

	ui->lineEditOldId->setFixedWidth(55);
	ui->lineEditNewId->setFixedWidth(55);

	ui->lineEditInterval->setText(configFile->value(MISC_SECTION_NAME"refresh_interval").toString());
	ui->lineEditIdToDisplay->setText(configFile->value(SLAVE_SECTION_NAME"id").toString());
}

Modbus::~Modbus()
{
	delete configFile;
	delete serial;
	delete ui;
}

void Modbus::on_btnApplyConfig_clicked()
{
	QString strDataBit = ui->comboBoxDataBits->currentText();
	QString strParit = ui->comboBoxParity->currentText();
	QString strStopBit = ui->comboBoxStopBits->currentText();
	QString strFlowControl = ui->comboBoxFlowControl->currentText();
	QString strSpeed = ui->comboBoxSpeed->currentText();
	QString port = ui->comboBoxPort->currentText();
	QString strIp = ui->lineEditIp->text();
	configFile->setValue(DEFAULT_SERIAL_SECTION_NAME"dataBits", strDataBit);
	configFile->setValue(DEFAULT_SERIAL_SECTION_NAME"parity", strParit);
	configFile->setValue(DEFAULT_SERIAL_SECTION_NAME"stopBits", strStopBit);
	configFile->setValue(DEFAULT_SERIAL_SECTION_NAME"flowControl", strFlowControl);
	configFile->setValue(DEFAULT_SERIAL_SECTION_NAME"baudRate", strSpeed);
	configFile->setValue(DEFAULT_SERIAL_SECTION_NAME"port", port);
	configFile->setValue(IP_SECTION_NAME"IP", strIp);
	qDebug() << "set DataBit:" << strDataBit;
	qDebug() << "set Parit:" << strParit;
	qDebug() << "set StopBits:" << strStopBit;
	qDebug() << "set FlowControl:" << strFlowControl;
	qDebug() << "set boadRate:" << strSpeed << endl;
	qDebug() << "set ip:" << strIp << endl;
}

#define BUG_REPORT(_cond, _format, _args...) \
    qDebug(                                  \
	"\nLine %d: assertion error for '%s': " _format "\n", __LINE__, #_cond, ##_args)

#define ASSERT_TRUE(_cond, _format, __args...)    \
    {                                             \
	if (_cond) {                              \
	    qDebug("OK\n");                       \
	} else {                                  \
	    BUG_REPORT(_cond, _format, ##__args); \
	    goto close;                           \
	}                                         \
    };


void Modbus::addLinemessage(QStringList list,int line)
{
	for(int i=0; i<list.size(); i++)
	{
		model->setItem(line,i,new QStandardItem(list.at(i)));
		model->item(line, i)->setTextAlignment(Qt::AlignCenter);
	}
}

int is_valid_ip(const char *ip) {
	// Check for null pointer
	if (ip == NULL) {
		return 0;
	}

	// Copy of the IP to avoid modifying the original string
	char ip_copy[16];
	strncpy(ip_copy, ip, sizeof(ip_copy) - 1);
	ip_copy[sizeof(ip_copy) - 1] = '\0';

	int segments = 0;  // Segment count
	char *token = strtok(ip_copy, ".");  // Split by dot

	while (token != NULL) {
		// Check if the segment is a number
		for (int i = 0; token[i]; i++) {
			if (!isdigit((unsigned char) token[i])) {
				return 0;
			}
		}

		// Convert segment to integer
		int num = atoi(token);
		if (num < 0 || num > 255) {
			return 0;
		}

		// Move to the next segment
		segments++;
		token = strtok(NULL, ".");
	}

	// A valid IPv4 address has exactly 4 segments
	return segments == 4;
}

void *Modbus::work_thread_cb(void *arg)
{
	setbuf(stdout, nullptr);
	setbuf(stdin, nullptr);
	class Modbus *pthis = (class Modbus *)arg;
	/* Length of report slave ID response slave ID + ON/OFF + 'LMB' + version */
	const int NB_REPORT_SLAVE_ID = 2 + 3 + strlen(LIBMODBUS_VERSION_STRING);
	uint16_t *tab_rp_registers = NULL;
	pthis->ctx = NULL;
	// modbus_t *ctx = NULL;
	int i;
	int nb_points;
	int rc;
	uint32_t old_response_to_sec;
	uint32_t old_response_to_usec;
	uint32_t new_response_to_sec;
	uint32_t new_response_to_usec;
	int use_backend;
	char *ip_or_device;
	unsigned short input_register_num = 128;
	int tmp_num;
	int read_num = 17 * 4 + 1;
	int group = 0;
	int baudRate;
	int baudRates[] = {115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 600};
	int intervals[9]; // us

	for (int i = 0; i < 9; ++i)
		intervals[i] = 3625 * 2^i + 20000;

	QString protocol = pthis->configFile->value(MODBUS_PROTOCOL_SECTION_NAME"Protocol").toString();
	QString ip = pthis->ui->lineEditIp->text();
	QString serialCom = pthis->ui->comboBoxPort->currentText();
	if (protocol == "TCP") {
		use_backend = TCP;
		ip_or_device = ip.toLatin1().data();
	} else if (protocol == "RTU") {
		use_backend = RTU;
		ip_or_device = serialCom.toLatin1().data();
	} else {
		return NULL;
	}

	if (ip_or_device[0] == 0)
		QMessageBox::warning(pthis, "Warning", "Device or ip cannot be null");
	qDebug("device or ip is: %s\n", ip_or_device);

	if (use_backend == TCP) {
		pthis->ctx = modbus_new_tcp(ip_or_device, 1502);
	} else if (use_backend == RTU) {
		baudRate = pthis->ui->comboBoxSpeed->currentText().toInt();
		pthis->ctx = modbus_new_rtu(ip_or_device, baudRate, 'N', 8, 1);
		if (pthis->ctx == NULL) {
			qDebug() << "Unable to allocate libmodbus context\n" << endl;
			return NULL;
		}
		modbus_rtu_set_serial_mode(pthis->ctx, MODBUS_RTU_RS485);
		modbus_rtu_set_rts(pthis->ctx, MODBUS_RTU_RTS_DOWN);
	}


	// modbus_set_debug(pthis->ctx, TRUE);
	modbus_set_error_recovery(
	    pthis->ctx, (modbus_error_recovery_mode)(MODBUS_ERROR_RECOVERY_LINK | MODBUS_ERROR_RECOVERY_PROTOCOL));

	if (use_backend == RTU) {
		modbus_set_slave(pthis->ctx, pthis->idToDisplay);
	}

	modbus_get_response_timeout(pthis->ctx, &old_response_to_sec, &old_response_to_usec);
	if (modbus_connect(pthis->ctx) == -1) {
		qDebug("Connection failed: %s\n", modbus_strerror(errno));
		modbus_free(pthis->ctx);
		return NULL;
	}

	nb_points = input_register_num;
	tab_rp_registers = (uint16_t *) malloc(nb_points * sizeof(uint16_t));
	memset(tab_rp_registers, 0, nb_points * sizeof(uint16_t));

	/* Set tab header */
	pthis->model = new QStandardItemModel();
	pthis->ui->tableView->setModel(pthis->model);
	pthis->ui->tableView->setEditTriggers(QAbstractItemView::NoEditTriggers);
	QStringList list;
	list<< "num" << "min" << "avg" << "max";
	for(int i=0;i<list.size();i++)
	{
		pthis->model->setHorizontalHeaderItem(i,new QStandardItem(list.at(i)));
		//pthis->ui->tableView->setColumnWidth(i,100);
	}
	pthis->ui->tableView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	//pthis->ui->tableView->horizontalHeader()->setStretchLastSection(false);

	qDebug("** UNIT TESTING **\n");
	qDebug("1/1 No response timeout modification on connect: ");
	modbus_get_response_timeout(pthis->ctx, &new_response_to_sec, &new_response_to_usec);
	ASSERT_TRUE(old_response_to_sec == new_response_to_sec &&
			old_response_to_usec == new_response_to_usec,
		    "");

	for (int i = 0; i < 9; ++i) {
		if (baudRate == baudRates[i])
			modbus_set_response_timeout(pthis->ctx, 0, baudRates[i]);
	}

	while (pthis->status == START) {

		rc = modbus_read_input_registers(
		    pthis->ctx, 0, read_num, tab_rp_registers);

		group = tab_rp_registers[0];
		read_num = group * 4 + 1;

		if (rc == -1) {
			qDebug("===============read data failed: %s==================\n",modbus_strerror(errno) );
			continue;
		}
		for (int i = 0; i < group; i++) {
			QStringList list;
			QString num;
			QString min;
			QString avg;
			QString max;
			if ((tab_rp_registers[i * 4 + 1] >> 8) == 0) {
				num = QString("P%1").arg(tab_rp_registers[i * 4 + 1] & 0x00ff);
			} else if ((tab_rp_registers[i * 4 + 1] >> 8) == 1)
				num = QString("L%1").arg(tab_rp_registers[i * 4 + 1] & 0x00ff);
				//qDebug("[L%d]\n", tab_rp_registers[i * 4] & 0x0f);
			else if ((tab_rp_registers[i * 4 + 1] >> 8) == 2)
				num = QString("R%1").arg(tab_rp_registers[i * 4 + 1] & 0x00ff);
				//qDebug("[R%d]\n", tab_rp_registers[i * 4] & 0x0f);
			else if ((tab_rp_registers[i * 4 + 1] >> 8) == 3)
				num = QString("C%1").arg(tab_rp_registers[i * 4 + 1] & 0x00ff);
				//qDebug("[C%d]\n", tab_rp_registers[i * 4] & 0x0f);
			else if ((tab_rp_registers[i * 4 + 1] >> 8) == 0xff)
				num = QString("full graph");
				//qDebug("[full graph]\n");
			min = QString("%1.%2").arg(tab_rp_registers[i * 4 + 1 + 1] / 10).arg(tab_rp_registers[i * 4 + 1 + 1] % 10);
			avg = QString("%1.%2").arg(tab_rp_registers[i * 4 + 2 + 1] / 10).arg(tab_rp_registers[i * 4 + 2 + 1] % 10);
			max = QString("%1.%2").arg(tab_rp_registers[i * 4 + 3 + 1] / 10).arg(tab_rp_registers[i * 4 + 3 + 1] % 10);
			/* remove the surplus row */
			if (tmp_num > group)
				pthis->model->removeRows(group, tmp_num - group);
			list << num << min << avg << max;
			pthis->addLinemessage(list, i);
		}
		tmp_num = group;
		usleep(1000 * pthis->refTime);
	}

close:
	/* Free the memory */
	free(tab_rp_registers);

	/* Close the connection */
	modbus_close(pthis->ctx);
	modbus_free(pthis->ctx);

	return NULL;
}

void Modbus::on_btnStart_clicked()
{
	QString protocol = this->configFile->value(MODBUS_PROTOCOL_SECTION_NAME"Protocol").toString();
	idToDisplay = this->configFile->value(SLAVE_SECTION_NAME"id").toInt();
	refTime = this->configFile->value(MISC_SECTION_NAME"refresh_interval").toInt();

	if (protocol == "RTU") {
		if (idToDisplay < 1 || idToDisplay > 247) {
			QMessageBox::warning(this, "Warning", "Please input ID, the ID must be \n less than 248 and greater than 0");
			return;
		}
	}

	QString ip = this->ui->lineEditIp->text();
	if (protocol == "TCP") {
		if (!is_valid_ip(ip.toUtf8().constData())) {
			QMessageBox::warning(this, "Warning", "Ip is illegal");
			return;
		}
	}

	status = (status == START) ? STOP:START;
	(status == START) ?
		ui->btnStart->setText("Stop"),
		ui->radioButtonRtu->setEnabled(false),
		ui->radioButtonTcp->setEnabled(false),
		(void)pthread_create(&work_thread, NULL, work_thread_cb, this):
	(ui->btnStart->setText("Start"),
	ui->radioButtonRtu->setEnabled(true),
	ui->radioButtonTcp->setEnabled(true));
	setSerialParameters();
}

void Modbus::on_radioButtonRtu_clicked()
{
	configFile->setValue(MODBUS_PROTOCOL_SECTION_NAME"Protocol", "RTU");
	toggleAllWidgetsInLayout(ui->vLayoutRTU, true);
	toggleAllWidgetsInLayout(ui->vLayoutIP, false);
}

void Modbus::on_radioButtonTcp_clicked()
{
	configFile->setValue(MODBUS_PROTOCOL_SECTION_NAME"Protocol", "TCP");
	toggleAllWidgetsInLayout(ui->vLayoutRTU, false);
	toggleAllWidgetsInLayout(ui->vLayoutIP, true);
}

int read_slave_id(modbus_t *ctx, uint16_t *tab_rp_registers)
{
	return modbus_read_registers( ctx, 0, 1, tab_rp_registers);
}

/* The function shall return the number of salves scaned. Otherwise it shall return -1 on no Slave */
int cb_scan_id(modbus_t *ctx, uint16_t *tab_rp_registers, void *data, void *out)
{
	uint8_t *idNum = (uint8_t *)out; *idNum = 0;
	uint8_t *idSet = (uint8_t *)data;
	for (int i = SLAVE_ID_MIN; i <= SLAVE_ID_MAX; ++i) {
		modbus_set_slave(ctx, i);
		if (read_slave_id(ctx, &tab_rp_registers[0]) > 0) {
			idSet[*idNum] = tab_rp_registers[0];
			(*idNum)++;
		}
	}
	return *idNum;
}

int cb_modify_id(modbus_t *ctx, uint16_t *tab_rp_registers, void *data, void * out)
{
	return modbus_write_register(ctx, 0, *(uint16_t *)data);
}

/* The return code is up to cb function exceipt -100, the -100 is a common failed code */
int Modbus::worker(int id, int (*cb)(modbus_t *ctx, uint16_t *tab_rp_registers, void *data, void *out), void *data, void *out)
{
	/* Length of report slave ID response slave ID + ON/OFF + 'LMB' + version */
	const int NB_REPORT_SLAVE_ID = 2 + 3 + strlen(LIBMODBUS_VERSION_STRING);
	uint16_t *tab_rp_registers = NULL;
	modbus_t *ctx = NULL;
	int i;
	int nb_points;
	int rc;
	uint32_t old_response_to_sec;
	uint32_t old_response_to_usec;
	uint32_t new_response_to_sec;
	uint32_t new_response_to_usec;
	int use_backend;
	char *ip_or_device;
	unsigned short input_register_num = 128;
	int tmp_num;
	int read_num = 17 * 4 + 1;
	int group = 0;

	QString protocol = this->configFile->value(MODBUS_PROTOCOL_SECTION_NAME"Protocol").toString();
	QString serialCom = this->ui->comboBoxPort->currentText();
	if (protocol == "TCP") {
		QMessageBox::warning(this, "Warning", "Please select RTU protocol rather than TCP");
		return -100;
	} else if (protocol == "RTU") {
		ip_or_device = serialCom.toLatin1().data();
	} else {
		QMessageBox::warning(this, "Warning", "Unkown protocl");
		return -100;
	}

	if (ip_or_device[0] == 0)
		QMessageBox::warning(this, "Warning", "Device or ip cannot be null");
	qDebug("device or ip is: %s\n", ip_or_device);

	ctx = modbus_new_rtu(ip_or_device, 115200, 'N', 8, 1);
	if (ctx == NULL) {
		qDebug() << "Unable to allocate libmodbus context\n" << endl;
		modbus_free(ctx);
		return -100;
	}
	modbus_rtu_set_serial_mode(ctx, MODBUS_RTU_RS485);
	modbus_rtu_set_rts(ctx, MODBUS_RTU_RTS_DOWN);
	modbus_set_response_timeout(ctx, 0, SCAN_ID_TIMOUT);

	modbus_set_debug(ctx, TRUE);
	modbus_set_error_recovery(
	    ctx, (modbus_error_recovery_mode)(MODBUS_ERROR_RECOVERY_LINK | MODBUS_ERROR_RECOVERY_PROTOCOL));

	modbus_get_response_timeout(ctx, &old_response_to_sec, &old_response_to_usec);
	if (modbus_connect(ctx) == -1) {
		qDebug("Connection failed: %s\n", modbus_strerror(errno));
		rc = -100;
		goto free_modbus;
	}
	modbus_set_slave(ctx, id);

	nb_points = input_register_num;
	tab_rp_registers = (uint16_t *) malloc(nb_points * sizeof(uint16_t));
	memset(tab_rp_registers, 0, nb_points * sizeof(uint16_t));

	qDebug("** Modify slave id **\n");
	qDebug("No response timeout modification on connect: ");
	modbus_get_response_timeout(ctx, &new_response_to_sec, &new_response_to_usec);
	ASSERT_TRUE(old_response_to_sec == new_response_to_sec &&
			old_response_to_usec == new_response_to_usec,
		    "");
	qDebug("Response time: %ds, %dus\n", new_response_to_sec, new_response_to_usec);

	rc = cb(ctx, tab_rp_registers, data, out);

close:
	/* Free the memory */
	free(tab_rp_registers);
free_modbus:
	/* Close the connection */
	modbus_close(ctx);
	modbus_free(ctx);
	return rc;
}

void Modbus::on_btnModifyId_clicked()
{
	int oldId = ui->lineEditOldId->text().toInt();
	int newId = ui->lineEditNewId->text().toInt();
	int ret = worker(oldId, cb_modify_id, &newId, NULL);

	ret < 0 ?
		(ret == -100 ?
		QMessageBox::warning(this, "Warning", "Open serial failed")
		: QMessageBox::warning(this, "Warning", "Modify slave ID failed"))
	: QMessageBox::information(this, "Info", "Modify slave ID successfully");
}

void Modbus::on_btnScanID_clicked()
{
	char ids[512] = { 0 };
	uint8_t idNum = 0;
	uint8_t idSet[SLAVE_ID_MAX - SLAVE_ID_MIN] = { 0 };
	int ret = worker(SLAVE_ID_MIN, cb_scan_id, idSet, &idNum);


	if (ret == -100)
		QMessageBox::warning(this, "Warning", "Open serial failed");
	else {
		for (int i = 0; i < idNum; i++)
			sprintf(ids + i * 5, "[%3d]", idSet[i]);
		QMessageBox::information(this, "Info", ids);
		qDebug("Found id: %s", ids);
	}
}

void Modbus::on_btnSetRefTime_clicked()
{
	refTime = ui->lineEditInterval->text().toInt();
	configFile->setValue(MISC_SECTION_NAME"refresh_interval", refTime);
}


void Modbus::on_btnIdToDisplay_clicked()
{
	int id = ui->lineEditIdToDisplay->text().toInt();
	if (id == 0)
		QMessageBox::warning(this, "Warning", "Please select which device to display\n Input device ID");
	if (id < 1 || id > 247)
		QMessageBox::warning(this, "Warning", "ID must be less than 248 and greater than 0");
	else {
		idToDisplay = id;
		modbus_set_slave(ctx, idToDisplay);
	}

	configFile->setValue(SLAVE_SECTION_NAME"id", id);
}

