#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtNetwork>
#include <QNetworkInterface>
#include <QDesktopServices>
#include <QUrl>

#define RED 0
#define GREEN 1
#define ACC 0
#define ROT 1
#define ELUR 2

QLineSeries *series_accx;
QLineSeries *series_accy;
QLineSeries *series_accz;
QLineSeries *series_rotx;
QLineSeries *series_roty;
QLineSeries *series_rotz;
QLineSeries *series_roll;
QLineSeries *series_pitch;
QLineSeries *series_yaw;

QString accx_str;
QString accy_str;
QString accz_str;
QString rotx_str;
QString roty_str;
QString rotz_str;
QString elurx_str;
QString elury_str;
QString elurz_str;

QString acc_num_str;
QString rot_num_str;
QString elur_num_str;

static float acc_exp[30] = {0};
static float rot_exp[30] = {0};
static float elur_exp[30] = {0};

static uint32_t acc_num = 0;
static uint32_t rot_num = 0;
static uint32_t elur_num = 0;

static float acc_test[3] = {0};
static float rot_test[3] = {0};
static float elur_test[3] = {0};

static float peak_acc_x = 0;
static float peak_acc_y = 0;
static float peak_acc_z = 0;

static int battery_value[2] = {0};
static int battery_raw[2] = {0};
static float battery = 0;

static qreal  t_acc = 0 ,intv_acc = 0.002;
static qreal  t_rot = 0 ,intv_rot = 0.002;
static qreal  t_elur = 0 ,intv_elur = 0.01;

//实验模式
// static const char * mode_exp_start = "0105000001d8cc";
static const char* mode_exp_start_ack = "010501000048cc";
// static const char * mode_exp_stop  = "0105000100189c";
static const char* mode_exp_stop_ack  = "010500100014cc";

//测试模式
// static const char* mode_test_start  = "010500000298cd";
static const char* mode_test_start_ack = "0105020000b8cc";
// static const char* mode_test_stop   = "0105000200186c";
static const char* mode_test_stop_ack  = "010500200000cc";

//数据导出模式
// static const char* mode_data_export_start  = "0105000003590d";
static const char* mode_data_export_start_ack = "0105030000e90c";
// static const char* mode_data_export_stop   = "010500030019fc";
static const char* mode_data_export_stop_ack  = "01050030000d0c";
// static const char * mode_data_export_receive_ack = "01010101010101";

//建立连接
static const char * mode_connect_start = "010500000418cf";
static const char* mode_connect_ack    = "010504000058cd";
// static const char * mode_connect_stop  = "01050004001bcc";
static const char* mode_connect_stop_ack = "010500400028cc";

//falsh擦除模式
// static const char* mode_flash_erease_start  = "0105000005d90f";
static const char* mode_flash_erease_start_ack = "0105050000090d";
// static const char* mode_flash_erease_stop   = "01050005001a5c";
static const char* mode_flash_erease_stop_ack  = "0105005000250c";

//连接检测
// static const char* connect_test  = "0105000006990e";
static const char* connect_test_ack = "0105060000f90d";

//数据接收应答
// static const char * data_receive_ack = "10101010101010";

bool flag_connect = false;
bool flag_exp     = false;
bool flag_test    = false;
bool flag_export  = false;
bool flag_erease  = false;
uint8_t flag_connect_ok = 0; //是否连接标志
uint16_t flag_number = 0;     //启动模式后接收数据量是否正常标志

static const int MaxDataSize    = 256;
static const int DataStart      = 2;
static const int DataStart_test = 3;
static const int DataLen        = 18;
static const int DataNum        = 14;
static const int MODE_LEN       = 15;

uint8_t dataArray[26];
uint8_t dataArray_test[MaxDataSize];
char modeArray[MODE_LEN];

uint16_t start_sector  = 0;
uint16_t end_sector    = 0;
uint16_t big_exp_num   = 0;
uint16_t small_exp_num = 0;
uint16_t big_exp_num_compare   = 0;
uint16_t small_exp_num_compare = 0;

qint64 maxBytesToRead = 256; // 你想要读取的最大字节数

QString fullPath;

QString fullPath_export;
QString fullPath_export_acc;
QString fullPath_export_rot;
QString fullPath_export_elur;

QString fullPath_exp;
QString fullPath_exp_acc;
QString fullPath_exp_rot;
QString fullPath_exp_elur;

QMessageBox *warningBox;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //设置状态栏
    labSocketState = new QLabel("Socket status: ");
    labSocketState->setMinimumWidth(200);
    ui->statusbar->addWidget(labSocketState);

    ui->local_ip->clear();  // 清除所有选项
    QList<QString> localIP_list = getAllLocalIPv4Addresses(); //获取本机IP
    for (int i = 0; i < localIP_list.size(); ++i)
    {
        ui->local_ip->addItem(localIP_list.at(i));
    }
    connect(ui->local_ip, SIGNAL(clicked()), this, SLOT(getIP()));

    udpSocket = new QUdpSocket(this);                     //创建socket
    connect(udpSocket,&QUdpSocket::stateChanged,this,&MainWindow::do_socketStateChange);
    do_socketStateChange(udpSocket->state());           //执行一次，显示当前状态
    connect(udpSocket,SIGNAL(readyRead()), this,SLOT(do_socketReadyRead()));

    setLED(ui->led_1, RED, 30);
    setLED(ui->led_2, RED, 30);
    setLED(ui->led_3, RED, 30);
    createChart_acc();
    createChart_rot();
    createChart_elur();
    prepareData(ACC);
    prepareData(ROT);
    prepareData(ELUR);

    m_timer_connect = new QTimer(this);               //创建定时器
    m_timer_connect->stop();                          //先停止定时器
    m_timer_connect->setTimerType(Qt::PreciseTimer);  //定时器精度等级
    m_timer_connect->setInterval(4000);               //设置定时器的周期
    m_timer_connect->setSingleShot(false);            //连续定时
    connect(m_timer_connect,SIGNAL(timeout()),this,SLOT(do_timer_connect_timeout())); //关联定时器的信号与槽


    m_timer_number = new QTimer(this);               //创建定时器
    m_timer_number->stop();                          //先停止定时器
    m_timer_number->setTimerType(Qt::PreciseTimer);  //定时器精度等级
    m_timer_number->setInterval(1000);               //设置定时器的周期
    m_timer_number->setSingleShot(false);            //连续定时
    connect(m_timer_number,SIGNAL(timeout()),this,SLOT(do_timer_number_timeout())); //关联定时器的信号与槽

    m_timer_redraw = new QTimer(this);               //图表批量刷新定时器
    m_timer_redraw->setTimerType(Qt::PreciseTimer);  //定时器精度等级
    m_timer_redraw->setInterval(33);                 //约30fps，降低重绘频率
    m_timer_redraw->setSingleShot(false);            //连续定时
    connect(m_timer_redraw,SIGNAL(timeout()),this,SLOT(do_timer_redraw_timeout())); //关联定时器的信号与槽
    m_timer_redraw->start();                         //一直运行，缓冲为空时开销可忽略

    // m_timer->start();     //启动定时器
    // m_timer->stop();    //定时器停止

    ui->erease_flash->setEnabled(false);
    ui->test_mode->setEnabled(false);
    ui->exp_mode->setEnabled(false);
    ui->export_data_2->setEnabled(false);

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if (udpSocket && udpSocket->state() != QAbstractSocket::UnconnectedState) {
        QString targetIP = ui->device_ip->text();
        QHostAddress targetAddr(targetIP);
        bool ok;
        quint16 targetPort = ui->device_port->text().toUShort(&ok);

        if (!ok || targetIP.isEmpty())
        {
            udpSocket->abort();
        }
        else
        {
            QString  msg= "01050004001bcc";
            QByteArray  str=msg.toUtf8();
            udpSocket->writeDatagram(str,targetAddr,targetPort); //发出数据报
        }
    }
    // 关闭所有可能仍打开的数据文件
    m_file_exp_acc.close();
    m_file_exp_rot.close();
    m_file_exp_elur.close();
    m_file_export_acc.close();
    m_file_export_rot.close();
    m_file_export_elur.close();

    event->accept();
}

MainWindow::~MainWindow() {
    if (udpSocket) {
        udpSocket->close(); // 或 abort()
        delete udpSocket;  // 如果不是智能指针管理
    }
    delete ui;
}

void MainWindow::setLED(QLabel* label, int color, int size) //0：红 1：绿
{
    label->setText("");

    QString min_width = QString("min-width: %1px;").arg(size);
    QString min_height = QString("min-height: %1px;").arg(size);
    QString max_width = QString("max-width: %1px;").arg(size);
    QString max_height = QString("max-height: %1px;").arg(size);
    QString border_radius = QString("border-radius: %2px;").arg(size/2);
    QString border = QString("border:1px solid black;");
    QString background = "background-color:";
    switch (color) {
    case 0:
        // 红色
        background += "rgb(255,0,0)";
        break;
    case 1:
        // 绿色
        background += "rgb(0,255,0)";
        break;
    default:
        break;
    }

    const QString SheetStyle = min_width + min_height + max_width + max_height + border_radius + border + background;
    label->setStyleSheet(SheetStyle);
}

void MainWindow::updatePeakAccDisplay(float x, float y, float z)
{
    if (qAbs(x) > qAbs(peak_acc_x)) peak_acc_x = x;
    if (qAbs(y) > qAbs(peak_acc_y)) peak_acc_y = y;
    if (qAbs(z) > qAbs(peak_acc_z)) peak_acc_z = z;

    ui->acc_x_peak->setText(QString::number(peak_acc_x, 'f', 3));
    ui->acc_y_peak->setText(QString::number(peak_acc_y, 'f', 3));
    ui->acc_z_peak->setText(QString::number(peak_acc_z, 'f', 3));
}

void MainWindow::createChart_acc()
{
    //创建图表
    chart_acc = new QChart();

    //设置标题-加粗-字体11
    chart_acc->setTitle(tr("ACC (500Hz)"));
    QFont currentFont = chart_acc->titleFont();
    currentFont.setBold(true);
    currentFont.setPointSize(11);   // 设置字号（根据需求调整）
    chart_acc->setTitleFont(currentFont);

    //设置边界间距
    QMargins mgs;
    mgs.setLeft(0);
    mgs.setRight(0);
    mgs.setTop(0);
    mgs.setBottom(0);
    chart_acc->setMargins(mgs);

    //设置图例在右侧
    chart_acc->legend()->setAlignment(Qt::AlignRight);

    ui->chart_acc->setRenderHint(QPainter::Antialiasing);
    ui->chart_acc->setChart(chart_acc);

    //生成三条折线
    series_accx = new QLineSeries();
    series_accy = new QLineSeries();
    series_accz = new QLineSeries();
    series_accx->setName("Acc-X");
    series_accy->setName("Acc-Y");
    series_accz->setName("Acc-Z");

    //设置折线属性
    QPen pen;
    pen.setStyle(Qt::SolidLine);  //Qt::SolidLine, Qt::DashLine, Qt::DotLine, Qt::DashDotLine
    pen.setWidth(2);
    pen.setColor(Qt::red);
    series_accx->setPen(pen);           //序列series0的线条设置

    pen.setStyle(Qt::SolidLine);//Qt::SolidLine, Qt::DashLine, Qt::DotLine, Qt::DashDotLine
    pen.setWidth(2);
    pen.setColor(Qt::green);
    series_accy->setPen(pen);           //序列series1的线条设置

    pen.setStyle(Qt::SolidLine);//Qt::SolidLine, Qt::DashLine, Qt::DotLine, Qt::DashDotLine
    pen.setWidth(2);
    pen.setColor(Qt::blue);
    series_accz->setPen(pen);           //序列series2的线条设置

    //启动openGL硬件加速
    series_accx->setUseOpenGL(true);
    series_accy->setUseOpenGL(true);
    series_accz->setUseOpenGL(true);

    //将序列添加到图表
    chart_acc->addSeries(series_accx);
    chart_acc->addSeries(series_accy);
    chart_acc->addSeries(series_accz);

    //设置横坐标轴属性
    QValueAxis *axisX = new QValueAxis;
    axisX->setRange(0, 3);              //设置坐标轴范围
    axisX->setLabelFormat("%.1f");      //标签格式
    axisX->setTickCount(4);             //主刻度个数
    axisX->setMinorTickCount(1);        //次刻度个数
    axisX->setTitleText("Time(s)");     //轴标题

    //设置纵坐标轴属性
    QValueAxis *axisY = new QValueAxis;
    axisY->setRange(-32, 32);
    axisY->setLabelFormat("%.2f");  //标签格式
    axisY->setTickCount(5);
    axisY->setMinorTickCount(0);
    axisY->setTitleText("Acc(m/s²)");

    //为chart和序列设置坐标轴
    chart_acc->addAxis(axisX,Qt::AlignBottom);
    chart_acc->addAxis(axisY,Qt::AlignLeft);

    series_accx->attachAxis(axisX); //序列series0 附加坐标轴
    series_accx->attachAxis(axisY);

    series_accy->attachAxis(axisX); //序列series1 附加坐标轴
    series_accy->attachAxis(axisY);

    series_accz->attachAxis(axisX); //序列series2 附加坐标轴
    series_accz->attachAxis(axisY);
}

void MainWindow::createChart_rot()
{
    //创建图表
    chart_rot = new QChart();

    //设置标题-加粗-字体11
    chart_rot->setTitle(tr("ROT (500Hz)"));
    QFont currentFont = chart_rot->titleFont();
    currentFont.setBold(true);
    currentFont.setPointSize(11);   // 设置字号（根据需求调整）
    chart_rot->setTitleFont(currentFont);

    //设置边界间距
    QMargins mgs;
    mgs.setLeft(0);
    mgs.setRight(0);
    mgs.setTop(0);
    mgs.setBottom(0);
    chart_rot->setMargins(mgs);

    //设置图例在右侧
    chart_rot->legend()->setAlignment(Qt::AlignRight);

    ui->chart_rot->setRenderHint(QPainter::Antialiasing);
    ui->chart_rot->setChart(chart_rot);

    //生成三条折线
    series_rotx = new QLineSeries();
    series_roty = new QLineSeries();
    series_rotz = new QLineSeries();

    series_rotx->setName("Rot-X");
    series_roty->setName("Rot-Y");
    series_rotz->setName("Rot-Z");

    //设置折线属性
    QPen  pen;
    pen.setStyle(Qt::SolidLine);  //Qt::SolidLine, Qt::DashLine, Qt::DotLine, Qt::DashDotLine
    pen.setWidth(2);
    pen.setColor(Qt::red);
    series_rotx->setPen(pen);           //序列series0的线条设置

    pen.setStyle(Qt::SolidLine);//Qt::SolidLine, Qt::DashLine, Qt::DotLine, Qt::DashDotLine
    pen.setWidth(2);
    pen.setColor(Qt::green);
    series_roty->setPen(pen);           //序列series1的线条设置

    pen.setStyle(Qt::SolidLine);//Qt::SolidLine, Qt::DashLine, Qt::DotLine, Qt::DashDotLine
    pen.setWidth(2);
    pen.setColor(Qt::blue);
    series_rotz->setPen(pen);           //序列series2的线条设置

    //启动openGL硬件加速
    series_rotx->setUseOpenGL(true);
    series_roty->setUseOpenGL(true);
    series_rotz->setUseOpenGL(true);

    //将序列添加到图表
    chart_rot->addSeries(series_rotx);  //将序列添加到图表
    chart_rot->addSeries(series_roty);
    chart_rot->addSeries(series_rotz);

    //设置横坐标轴属性
    QValueAxis *axisX = new QValueAxis;
    axisX->setRange(0, 3);             //设置坐标轴范围
    axisX->setLabelFormat("%.1f");      //标签格式
    axisX->setTickCount(4);            //主刻度个数
    axisX->setMinorTickCount(1);        //次刻度个数
    axisX->setTitleText("Time(s)");  //轴标题

    QValueAxis *axisY = new QValueAxis;
    axisY->setRange(-15, 15);
    axisY->setLabelFormat("%.2f");  //标签格式
    axisY->setTickCount(5);
    axisY->setMinorTickCount(0);
    axisY->setTitleText("Rot(rad/s)");

    //为chart和序列设置坐标轴
    chart_rot->addAxis(axisX,Qt::AlignBottom); //坐标轴添加到图表，并指定方向
    chart_rot->addAxis(axisY,Qt::AlignLeft);

    series_rotx->attachAxis(axisX); //序列series0 附加坐标轴
    series_rotx->attachAxis(axisY);

    series_roty->attachAxis(axisX); //序列series1 附加坐标轴
    series_roty->attachAxis(axisY);

    series_rotz->attachAxis(axisX); //序列series2 附加坐标轴
    series_rotz->attachAxis(axisY);
}

void MainWindow::createChart_elur()
{
    //创建图表
    chart_elur = new QChart();

    //设置标题-加粗-字体11
    chart_elur->setTitle(tr("ELUR (100Hz)"));
    QFont currentFont = chart_elur->titleFont();
    currentFont.setBold(true);
    currentFont.setPointSize(11);   // 设置字号（根据需求调整）
    chart_elur->setTitleFont(currentFont);

    //设置边界间距
    QMargins mgs;
    mgs.setLeft(0);
    mgs.setRight(0);
    mgs.setTop(0);
    mgs.setBottom(0);
    chart_elur->setMargins(mgs);

    //设置图例在右侧
    chart_elur->legend()->setAlignment(Qt::AlignRight);

    ui->chart_elur->setRenderHint(QPainter::Antialiasing);
    ui->chart_elur->setChart(chart_elur);

    //生成三条折线
    series_roll = new QLineSeries();
    series_pitch = new QLineSeries();
    series_yaw = new QLineSeries();

    series_roll->setName("Roll");
    series_pitch->setName("Pitch");
    series_yaw->setName("Yaw");

    //设置折线属性
    QPen  pen;
    pen.setStyle(Qt::SolidLine);  //Qt::SolidLine, Qt::DashLine, Qt::DotLine, Qt::DashDotLine
    pen.setWidth(2);
    pen.setColor(Qt::red);
    series_roll->setPen(pen);           //序列series0的线条设置

    pen.setStyle(Qt::SolidLine);//Qt::SolidLine, Qt::DashLine, Qt::DotLine, Qt::DashDotLine
    pen.setWidth(2);
    pen.setColor(Qt::green);
    series_pitch->setPen(pen);           //序列series1的线条设置

    pen.setStyle(Qt::SolidLine);//Qt::SolidLine, Qt::DashLine, Qt::DotLine, Qt::DashDotLine
    pen.setWidth(2);

    pen.setColor(Qt::blue);
    series_yaw->setPen(pen);           //序列series2的线条设置

    //启动openGL硬件加速
    series_roll->setUseOpenGL(true);
    series_pitch->setUseOpenGL(true);
    series_yaw->setUseOpenGL(true);

    //将序列添加到图表
    chart_elur->addSeries(series_roll);  //将序列添加到图表
    chart_elur->addSeries(series_pitch);
    chart_elur->addSeries(series_yaw);

    //设置横坐标轴属性
    QValueAxis *axisX = new QValueAxis;
    axisX->setRange(0, 3);             //设置坐标轴范围
    axisX->setLabelFormat("%.1f");      //标签格式
    axisX->setTickCount(4);            //主刻度个数
    axisX->setMinorTickCount(1);        //次刻度个数
    axisX->setTitleText("Time(s)");  //轴标题

    QValueAxis *axisY = new QValueAxis;
    axisY->setRange(-200, 200);
    axisY->setLabelFormat("%.2f");  //标签格式
    axisY->setTickCount(5);
    axisY->setMinorTickCount(0);
    axisY->setTitleText("Elur(deg)");

    //为chart和序列设置坐标轴
    chart_elur->addAxis(axisX,Qt::AlignBottom); //坐标轴添加到图表，并指定方向
    chart_elur->addAxis(axisY,Qt::AlignLeft);

    series_roll->attachAxis(axisX); //序列series0 附加坐标轴
    series_roll->attachAxis(axisY);

    series_pitch->attachAxis(axisX); //序列series1 附加坐标轴
    series_pitch->attachAxis(axisY);

    series_yaw->attachAxis(axisX); //序列series2 附加坐标轴
    series_yaw->attachAxis(axisY);
}

QString MainWindow::getLocalIP()
{
    //获取本机IPv4地址
    QString   hostName = QHostInfo::localHostName();    //本地主机名
    QHostInfo hostInfo = QHostInfo::fromName(hostName);
    QString   localIP = "";

    QList<QHostAddress> addList = hostInfo.addresses();  //本机IP地址列表
    if (addList.isEmpty())
        return localIP;

    foreach(QHostAddress aHost, addList)
        if (QAbstractSocket::IPv4Protocol == aHost.protocol())
        {
            localIP = aHost.toString();

            break;
        }
    return localIP;
}

QList<QString> MainWindow::getAllLocalIPv4Addresses()
{
    QList<QString> ipList;

    // 获取所有网络接口
    QList<QNetworkInterface> interfaces = QNetworkInterface::allInterfaces();

    // 遍历所有网络接口（使用普通 for 循环 + at() 避免 detach）
    for (int i = 0; i < interfaces.size(); ++i)
    {
        const QNetworkInterface &interface = interfaces.at(i);  // 使用 at() 避免 detach

        // 跳过非活动接口和回环接口
        if (!(interface.flags() & QNetworkInterface::IsUp) ||
            (interface.flags() & QNetworkInterface::IsLoopBack))
        {
            continue;
        }

        // 获取接口的所有地址
        QList<QNetworkAddressEntry> entries = interface.addressEntries();
        for (int j = 0; j < entries.size(); ++j)
        {
            const QNetworkAddressEntry &entry = entries.at(j);  // 使用 at() 避免 detach
            QHostAddress addr = entry.ip();

            // 只处理 IPv4 地址，并排除回环地址
            if (addr.protocol() == QAbstractSocket::IPv4Protocol &&
                !addr.isLoopback())
            {
                ipList.append(addr.toString());
            }
        }
    }

    return ipList;
}

void MainWindow::getIP()
{
    QString currentIP = ui->local_ip->currentText();
    ui->local_ip->clear();  // 清除所有选项
    QList<QString> localIP_list = getAllLocalIPv4Addresses(); //获取本机IP
    for (int i = 0; i < localIP_list.size(); ++i)
    {
        ui->local_ip->addItem(localIP_list.at(i));
    }
    ui->local_ip->setCurrentText(currentIP);
}

void MainWindow::do_socketStateChange(QAbstractSocket::SocketState socketState)
{
    // socket状态变化时
    //A[UnconnectedState] --> B[BoundState]
    //B --> A (调用 close()/abort())
    switch(socketState)
    {
    case QAbstractSocket::UnconnectedState:
        labSocketState->setText("Socket status：UnconnectedState"); //A 未连接状态
        break;
    case QAbstractSocket::HostLookupState:
        labSocketState->setText("Socket status：HostLookupState"); //B 主机查找状态
        break;
    case QAbstractSocket::ConnectingState:
        labSocketState->setText("Socket status：ConnectingState"); //C 连接中状态
        break;
    case QAbstractSocket::ConnectedState:
        labSocketState->setText("Socket status：ConnectedState"); //D 已连接状态
        break;
    case QAbstractSocket::BoundState:
        labSocketState->setText("Socket status：BoundState");    //绑定状态
        break;
    case QAbstractSocket::ClosingState:
        labSocketState->setText("Socket status：ClosingState"); //E 关闭中状态
        break;
    case QAbstractSocket::ListeningState:
        labSocketState->setText("Socket status：ListeningState"); //监听状态
    }
}

void MainWindow::prepareData(int flag)
{
    qreal   y1, y2, y3;

    if(0 == flag) //更新加速度
    {
        //为序列生成数据
        series_accx = static_cast<QLineSeries *>(chart_acc->series().at(0));
        series_accy = static_cast<QLineSeries *>(chart_acc->series().at(1));
        series_accz = static_cast<QLineSeries *>(chart_acc->series().at(2));

        y1 = acc_test[0];
        y2 = acc_test[1];
        y3 = acc_test[2];

        series_accx->append(t_acc, y1);  //序列添加数据点
        series_accy->append(t_acc, y2);
        series_accz->append(t_acc, y3);
        t_acc += intv_acc;
        if(t_acc >= 10)
        {
            t_acc = 0;
            series_accx->clear();
            series_accy->clear();
            series_accz->clear();
        }

    }
    else if(1 == flag)//更新角速度
    {
        //为序列生成数据
        series_rotx = static_cast<QLineSeries *>(chart_rot->series().at(0));
        series_roty = static_cast<QLineSeries *>(chart_rot->series().at(1));
        series_rotz = static_cast<QLineSeries *>(chart_rot->series().at(2));

        y1 = rot_test[0];
        y2 = rot_test[1];
        y3 = rot_test[2];

        series_rotx->append(t_rot, y1);  //序列添加数据点
        series_roty->append(t_rot, y2);
        series_rotz->append(t_rot, y3);
        t_rot += intv_rot;
        if(t_rot >= 10)
        {
            t_rot = 0;
            series_rotx->clear();
            series_roty->clear();
            series_rotz->clear();
        }
    }
    else if(2 == flag)//更新欧拉角
    {
        //为序列生成数据
        series_roll  = static_cast<QLineSeries *>(chart_elur->series().at(0));
        series_pitch = static_cast<QLineSeries *>(chart_elur->series().at(1));
        series_yaw   = static_cast<QLineSeries *>(chart_elur->series().at(2));

        y1 = elur_test[0];
        y2 = elur_test[1];
        y3 = elur_test[2];

        series_roll->append(t_elur, y1);  //序列添加数据点
        series_pitch->append(t_elur, y2);
        series_yaw->append(t_elur, y3);
        t_elur += intv_elur;
        if(t_elur >= 10)
        {
            t_elur = 0;
            series_roll->clear();
            series_pitch->clear();
            series_yaw->clear();
        }
    }
}

void MainWindow::do_socketReadyRead()   //读消息及处理
{
    // 读取收到的数据报
    while (udpSocket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(udpSocket->pendingDatagramSize());

        QHostAddress peerAddr;
        quint16      peerPort;
        QString      pureIPv4Addr;
        QHostAddress sendAddr;                                          //用于发送的QHostAddress
        //读数据
        memset(dataArray_test, 0, sizeof(dataArray_test));
        udpSocket->readDatagram(datagram.data(), datagram.size(), &peerAddr, &peerPort);
        QString str = QString::fromUtf8(datagram.data());               //使用fromUtf8以确保正确解码UTF-8字符串
        int copySize = qMin(datagram.size(), static_cast<int>(sizeof(dataArray_test)));
        memcpy(dataArray_test, datagram.data(), copySize);

        if (str == QString::fromUtf8(mode_connect_start))               //建立连接
        {
            //处理接收到设备的IP和端口号
            QString peerAddrStr = peerAddr.toString();

            // 检查是否为IPv4映射的IPv6地址，并提取IPv4部分
            if (peerAddrStr.startsWith("::ffff:"))
            {
                pureIPv4Addr = peerAddrStr.mid(7);          // 跳过"::ffff:"前缀
                sendAddr.setAddress(pureIPv4Addr);          // 将纯IPv4地址字符串转换为QHostAddress
            }
            else
            {
                pureIPv4Addr = peerAddrStr;                 // 如果不是映射地址，则直接使用原地址（理论上这里也可以是IPv6地址）
                sendAddr = peerAddr;                        // 直接使用接收到的QHostAddress（可能是IPv4或IPv6）
            }

            std::string addrStr = pureIPv4Addr.toStdString();
            //将目标的IP和port写到lineEdit_device_ip lineEdit_device_port
            ui->device_ip->setText(pureIPv4Addr);  // 直接使用 QString
            ui->device_port->setText(QString::number(peerPort));  // 数值转字符串

            QByteArray ackData = QByteArray(mode_connect_ack);  // 直接构造QByteArray

            udpSocket->writeDatagram(ackData, peerAddr, peerPort);
            // 发送数据报回给发送者（使用提取或原始的QHostAddress）
            flag_connect = true;

            m_timer_connect->start(); //启动定时器

            setLED(ui->led_1, GREEN, 30);
            ui->erease_flash->setEnabled(true);
            ui->test_mode->setEnabled(true);
            ui->exp_mode->setEnabled(true);
            ui->export_data_2->setEnabled(true);
        }
        else if(str == QString::fromUtf8(mode_connect_stop_ack))        //停止连接成功
        {
            flag_connect = false;
            flag_exp     = false;
            flag_test    = false;
            flag_export  = false;
            flag_erease  = false;
            flag_connect_ok = 0;

            udpSocket->abort();
            ui->device_ip->clear();    // 清空 IP 输入框
            ui->device_port->clear();  // 清空端口输入框
            ui->erease_flash->setEnabled(false);
            ui->test_mode->setEnabled(false);
            ui->exp_mode->setEnabled(false);
            ui->export_data_2->setEnabled(false);
            ui->pind_port->setText("监听端口");
            setLED(ui->led_1, RED, 30);

            m_timer_connect->stop(); //停止定时器
        }
        else if (str == QString::fromUtf8(mode_test_start_ack))         //启动测试模式成功
        {
            flag_test = true;
            flag_exp = false;
            flag_erease = false;
            flag_export = false;

            t_acc = 0;
            t_rot = 0;
            t_elur = 0;
            peak_acc_x = peak_acc_y = peak_acc_z = 0;
            series_accx->clear();
            series_accy->clear();
            series_accz->clear();
            series_rotx->clear();
            series_roty->clear();
            series_rotz->clear();
            series_roll->clear();
            series_pitch->clear();
            series_yaw->clear();

            m_buf_accx.clear();
            m_buf_accy.clear();
            m_buf_accz.clear();
            m_buf_rotx.clear();
            m_buf_roty.clear();
            m_buf_rotz.clear();
            m_buf_roll.clear();
            m_buf_pitch.clear();
            m_buf_yaw.clear();
            m_acc_clear = false;
            m_rot_clear = false;
            m_elur_clear = false;

            m_timer_number->start();     //启动定时器

            ui->pind_port->setEnabled(false);
            ui->erease_flash->setEnabled(false);
            ui->exp_mode->setEnabled(false);
            ui->export_data_2->setEnabled(false);

            ui->test_mode->setText("停止测试模式");
        }
        else if(flag_test == true)
        {
            if (str == QString::fromUtf8(mode_test_stop_ack))           //停止测试模式成功
            {
                flag_test = false;

                m_timer_number->stop();     //停止定时器
                flag_number = 0;

                ui->pind_port->setEnabled(true);
                ui->erease_flash->setEnabled(true);
                ui->exp_mode->setEnabled(true);
                if (!ui->data_path->text().isEmpty() && !ui->data_name->text().isEmpty())
                {
                    ui->export_data_2->setEnabled(true);
                }

                ui->test_mode->setText("启动测试模式");
            }
            else
            {
                uint32_t combined = 0;

                if ( (dataArray_test[0] == 0xfa) && (dataArray_test[1] == 0xff) )
                {
                    flag_number += 1;

                    if(dataArray_test[2] == 0x01)
                    {
                        combined = ((uint32_t)dataArray_test[DataStart_test]     << 24) |
                                   ((uint32_t)dataArray_test[DataStart_test + 1] << 16) |
                                   ((uint32_t)dataArray_test[DataStart_test + 2] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart_test + 3]);
                        memcpy(&acc_test[0], &combined, sizeof(float));
                        combined = ((uint32_t)dataArray_test[DataStart_test + 4] << 24) |
                                   ((uint32_t)dataArray_test[DataStart_test + 5] << 16) |
                                   ((uint32_t)dataArray_test[DataStart_test + 6] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart_test + 7]);
                        memcpy(&acc_test[1], &combined, sizeof(float));
                        combined = ((uint32_t)dataArray_test[DataStart_test + 8]  << 24) |
                                   ((uint32_t)dataArray_test[DataStart_test + 9]  << 16) |
                                   ((uint32_t)dataArray_test[DataStart_test + 10] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart_test + 11]);
                        memcpy(&acc_test[2], &combined, sizeof(float));

                        updatePeakAccDisplay(acc_test[0], acc_test[1], acc_test[2]);

                        //为序列生成数据（先入缓冲，定时器统一刷新）
                        m_buf_accx.append(QPointF(t_acc, acc_test[0]));
                        m_buf_accy.append(QPointF(t_acc, acc_test[1]));
                        m_buf_accz.append(QPointF(t_acc, acc_test[2]));

                        t_acc += intv_acc;
                        if(t_acc >= 3)
                        {
                            t_acc = 0;
                            peak_acc_x = peak_acc_y = peak_acc_z = 0;
                            m_buf_accx.clear();
                            m_buf_accy.clear();
                            m_buf_accz.clear();
                            m_acc_clear = true;
                        }
                    }
                    else if(dataArray_test[2] == 0x02)
                    {
                        combined = ((uint32_t)dataArray_test[DataStart_test]     << 24) |
                                   ((uint32_t)dataArray_test[DataStart_test + 1] << 16) |
                                   ((uint32_t)dataArray_test[DataStart_test + 2] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart_test + 3]);
                        memcpy(&rot_test[0], &combined, sizeof(float));
                        combined = ((uint32_t)dataArray_test[DataStart_test + 4] << 24) |
                                   ((uint32_t)dataArray_test[DataStart_test + 5] << 16) |
                                   ((uint32_t)dataArray_test[DataStart_test + 6] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart_test + 7]);
                        memcpy(&rot_test[1], &combined, sizeof(float));
                        combined = ((uint32_t)dataArray_test[DataStart_test + 8]  << 24) |
                                   ((uint32_t)dataArray_test[DataStart_test + 9]  << 16) |
                                   ((uint32_t)dataArray_test[DataStart_test + 10] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart_test + 11]);
                        memcpy(&rot_test[2], &combined, sizeof(float));

                        //为序列生成数据（先入缓冲，定时器统一刷新）
                        m_buf_rotx.append(QPointF(t_rot, rot_test[0]));
                        m_buf_roty.append(QPointF(t_rot, rot_test[1]));
                        m_buf_rotz.append(QPointF(t_rot, rot_test[2]));

                        t_rot += intv_rot;
                        if(t_rot >= 3)
                        {
                            t_rot = 0;
                            m_buf_rotx.clear();
                            m_buf_roty.clear();
                            m_buf_rotz.clear();
                            m_rot_clear = true;
                        }
                    }
                    else if(dataArray_test[2] == 0x03)
                    {
                        combined = ((uint32_t)dataArray_test[DataStart_test]     << 24) |
                                   ((uint32_t)dataArray_test[DataStart_test + 1] << 16) |
                                   ((uint32_t)dataArray_test[DataStart_test + 2] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart_test + 3]);
                        memcpy(&elur_test[0], &combined, sizeof(float));
                        combined = ((uint32_t)dataArray_test[DataStart_test + 4] << 24) |
                                   ((uint32_t)dataArray_test[DataStart_test + 5] << 16) |
                                   ((uint32_t)dataArray_test[DataStart_test + 6] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart_test + 7]);
                        memcpy(&elur_test[1], &combined, sizeof(float));
                        combined = ((uint32_t)dataArray_test[DataStart_test + 8]  << 24) |
                                   ((uint32_t)dataArray_test[DataStart_test + 9]  << 16) |
                                   ((uint32_t)dataArray_test[DataStart_test + 10] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart_test + 11]);
                        memcpy(&elur_test[2], &combined, sizeof(float));

                        //为序列生成数据（先入缓冲，定时器统一刷新）
                        m_buf_roll.append(QPointF(t_elur, elur_test[0]));
                        m_buf_pitch.append(QPointF(t_elur, elur_test[1]));
                        m_buf_yaw.append(QPointF(t_elur, elur_test[2]));

                        t_elur += intv_elur;
                        if(t_elur >= 3)
                        {
                            t_elur = 0;
                            m_buf_roll.clear();
                            m_buf_pitch.clear();
                            m_buf_yaw.clear();
                            m_elur_clear = true;
                        }
                    }
                }
            }
        }
        else if (str == QString::fromUtf8(mode_flash_erease_start_ack)) //擦除flash启动成功
        {
            flag_exp = false;
            flag_test = false;
            flag_erease = true;
            flag_export = false;

            showWarning();

            ui->test_mode->setEnabled(false);
            ui->exp_mode->setEnabled(false);
            ui->erease_flash->setEnabled(false);
            ui->export_data_2->setEnabled(false);
        }
        else if (flag_erease == true)
        {
            if (str == QString::fromUtf8(mode_flash_erease_stop_ack))   //停止擦除模式成功
            {
                flag_erease = false;

                if (warningBox && !warningBox->isHidden())
                {  // 检查窗口是否存在且未隐藏
                    warningBox->close();
                }

                setLED(ui->led_2, GREEN, 30);

                ui->test_mode->setEnabled(true);
                ui->exp_mode->setEnabled(true);
                ui->erease_flash->setEnabled(true);
                ui->export_data_2->setEnabled(true);
            }
        }
        else if (str == QString::fromUtf8(mode_exp_start_ack))          //启动实验模式成功
        {
            flag_exp = true;
            flag_test = false;
            flag_erease = false;
            flag_export = false;

            t_acc = 0;
            t_rot = 0;
            t_elur = 0;
            peak_acc_x = peak_acc_y = peak_acc_z = 0;

            series_accx->clear();
            series_accy->clear();
            series_accz->clear();
            series_rotx->clear();
            series_roty->clear();
            series_rotz->clear();
            series_roll->clear();
            series_pitch->clear();
            series_yaw->clear();

            m_buf_accx.clear();
            m_buf_accy.clear();
            m_buf_accz.clear();
            m_buf_rotx.clear();
            m_buf_roty.clear();
            m_buf_rotz.clear();
            m_buf_roll.clear();
            m_buf_pitch.clear();
            m_buf_yaw.clear();
            m_acc_clear = false;
            m_rot_clear = false;
            m_elur_clear = false;

            m_timer_number->start();     //启动定时器

            ui->pind_port->setEnabled(false);
            ui->erease_flash->setEnabled(false);
            ui->test_mode->setEnabled(false);
            ui->export_data_2->setEnabled(false);

            ui->exp_mode->setText("停止实验模式");
        }
        else if (flag_exp == true)
        {
            if (str == QString::fromUtf8(mode_exp_stop_ack))         //停止实验模式成功
            {
                flag_exp = false;

                m_timer_number->stop();     //停止定时器
                flag_number = 0;

                ui->pind_port->setEnabled(true);
                ui->erease_flash->setEnabled(true);
                ui->test_mode->setEnabled(true);
                if (!ui->data_path->text().isEmpty() && !ui->data_name->text().isEmpty())
                {
                    ui->export_data_2->setEnabled(true);
                }

                // 关闭常开的数据文件，确保数据落盘后再合并
                m_file_exp_acc.close();
                m_file_exp_rot.close();
                m_file_exp_elur.close();

                merge_csv_files_horizontally(fullPath_exp, fullPath_exp_acc, fullPath_exp_rot, fullPath_exp_elur);
                setLED(ui->led_2, RED, 30);
                ui->exp_mode->setText("启动实验模式");
            }
            else
            {
                uint32_t combined = 0;

                if ( dataArray_test[0] == 0xaa )
                {
                    flag_number += 1;

                    if(dataArray_test[1] == 0x01)
                    {
                        combined = ((uint32_t)dataArray_test[DataStart]     << 24) |
                                   ((uint32_t)dataArray_test[DataStart + 1] << 16) |
                                   ((uint32_t)dataArray_test[DataStart + 2] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart + 3]);
                        memcpy(&acc_exp[0], &combined, sizeof(float));
                        accx_str = QString::number(acc_exp[0]);
                        combined = ((uint32_t)dataArray_test[DataStart + 4] << 24) |
                                   ((uint32_t)dataArray_test[DataStart + 5] << 16) |
                                   ((uint32_t)dataArray_test[DataStart + 6] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart + 7]);
                        memcpy(&acc_exp[1], &combined, sizeof(float));
                        accy_str = QString::number(acc_exp[1]);
                        combined = ((uint32_t)dataArray_test[DataStart + 8]  << 24) |
                                   ((uint32_t)dataArray_test[DataStart + 9]  << 16) |
                                   ((uint32_t)dataArray_test[DataStart + 10] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart + 11]);
                        memcpy(&acc_exp[2], &combined, sizeof(float));
                        accz_str = QString::number(acc_exp[2]);

                        updatePeakAccDisplay(acc_exp[0], acc_exp[1], acc_exp[2]);

                        combined = ((uint32_t)dataArray_test[DataStart + 12]  << 24) |
                                   ((uint32_t)dataArray_test[DataStart + 13]  << 16) |
                                   ((uint32_t)dataArray_test[DataStart + 14]  << 8)  |
                                   ((uint32_t)dataArray_test[DataStart + 15]);
                        memcpy(&acc_num, &combined, sizeof(uint32_t));
                        acc_num_str = QString::number(acc_num);

                        //为序列生成数据（先入缓冲，定时器统一刷新）
                        m_buf_accx.append(QPointF(t_acc, acc_exp[0]));
                        m_buf_accy.append(QPointF(t_acc, acc_exp[1]));
                        m_buf_accz.append(QPointF(t_acc, acc_exp[2]));

                        t_acc += intv_acc;
                        if(t_acc >= 3)
                        {
                            t_acc = 0;
                            peak_acc_x = peak_acc_y = peak_acc_z = 0;
                            m_buf_accx.clear();
                            m_buf_accy.clear();
                            m_buf_accz.clear();
                            m_acc_clear = true;
                        }

                        if (m_file_exp_acc.isOpen())
                        {
                            QTextStream out(&m_file_exp_acc);

                            out << acc_num_str << ',' << QString::number(m_elapsed.nsecsElapsed() / 1e6, 'f', 3) << ',' << accx_str <<','<< accy_str <<','<< accz_str <<'\n';
                        }
                    }
                    else if(dataArray_test[1] == 0x02)
                    {
                        combined = ((uint32_t)dataArray_test[DataStart]     << 24) |
                                   ((uint32_t)dataArray_test[DataStart + 1] << 16) |
                                   ((uint32_t)dataArray_test[DataStart + 2] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart + 3]);
                        memcpy(&rot_exp[0], &combined, sizeof(float));
                        rotx_str = QString::number(rot_exp[0]);
                        combined = ((uint32_t)dataArray_test[DataStart + 4] << 24) |
                                   ((uint32_t)dataArray_test[DataStart + 5] << 16) |
                                   ((uint32_t)dataArray_test[DataStart + 6] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart + 7]);
                        memcpy(&rot_exp[1], &combined, sizeof(float));
                        roty_str = QString::number(rot_exp[1]);
                        combined = ((uint32_t)dataArray_test[DataStart + 8]  << 24) |
                                   ((uint32_t)dataArray_test[DataStart + 9]  << 16) |
                                   ((uint32_t)dataArray_test[DataStart + 10] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart + 11]);
                        memcpy(&rot_exp[2], &combined, sizeof(float));
                        rotz_str = QString::number(rot_exp[2]);
                        combined = ((uint32_t)dataArray_test[DataStart + 12]  << 24) |
                                   ((uint32_t)dataArray_test[DataStart + 13]  << 16) |
                                   ((uint32_t)dataArray_test[DataStart + 14]  << 8)  |
                                   ((uint32_t)dataArray_test[DataStart + 15]);
                        memcpy(&rot_num, &combined, sizeof(uint32_t));
                        rot_num_str = QString::number(rot_num);

                        //为序列生成数据（先入缓冲，定时器统一刷新）
                        m_buf_rotx.append(QPointF(t_rot, rot_exp[0]));
                        m_buf_roty.append(QPointF(t_rot, rot_exp[1]));
                        m_buf_rotz.append(QPointF(t_rot, rot_exp[2]));

                        t_rot += intv_rot;
                        if(t_rot >= 3)
                        {
                            t_rot = 0;
                            m_buf_rotx.clear();
                            m_buf_roty.clear();
                            m_buf_rotz.clear();
                            m_rot_clear = true;
                        }

                        if (m_file_exp_rot.isOpen())
                        {
                            QTextStream out(&m_file_exp_rot);

                            out << rot_num_str << ',' << QString::number(m_elapsed.nsecsElapsed() / 1e6, 'f', 3) << ',' << rotx_str <<','<< roty_str <<','<< rotz_str <<'\n';
                        }
                    }
                    else if(dataArray_test[1] == 0x03)
                    {
                        combined = ((uint32_t)dataArray_test[DataStart]     << 24) |
                                   ((uint32_t)dataArray_test[DataStart + 1] << 16) |
                                   ((uint32_t)dataArray_test[DataStart + 2] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart + 3]);
                        memcpy(&elur_exp[0], &combined, sizeof(float));
                        elurx_str = QString::number(elur_exp[0]);
                        combined = ((uint32_t)dataArray_test[DataStart + 4] << 24) |
                                   ((uint32_t)dataArray_test[DataStart + 5] << 16) |
                                   ((uint32_t)dataArray_test[DataStart + 6] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart + 7]);
                        memcpy(&elur_exp[1], &combined, sizeof(float));
                        elury_str = QString::number(elur_exp[1]);
                        combined = ((uint32_t)dataArray_test[DataStart + 8]  << 24) |
                                   ((uint32_t)dataArray_test[DataStart + 9]  << 16) |
                                   ((uint32_t)dataArray_test[DataStart + 10] << 8)  |
                                   ((uint32_t)dataArray_test[DataStart + 11]);
                        memcpy(&elur_exp[2], &combined, sizeof(float));
                        elurz_str = QString::number(elur_exp[2]);
                        combined = ((uint32_t)dataArray_test[DataStart + 12]  << 24) |
                                   ((uint32_t)dataArray_test[DataStart + 13]  << 16) |
                                   ((uint32_t)dataArray_test[DataStart + 14]  << 8)  |
                                   ((uint32_t)dataArray_test[DataStart + 15]);
                        memcpy(&elur_num, &combined, sizeof(uint32_t));
                        elur_num_str = QString::number(elur_num);

                        //为序列生成数据（先入缓冲，定时器统一刷新）
                        m_buf_roll.append(QPointF(t_elur, elur_exp[0]));
                        m_buf_pitch.append(QPointF(t_elur, elur_exp[1]));
                        m_buf_yaw.append(QPointF(t_elur, elur_exp[2]));

                        t_elur += intv_elur;
                        if(t_elur >= 3)
                        {
                            t_elur = 0;
                            m_buf_roll.clear();
                            m_buf_pitch.clear();
                            m_buf_yaw.clear();
                            m_elur_clear = true;
                        }

                        if (m_file_exp_elur.isOpen())
                        {
                            QTextStream out(&m_file_exp_elur);

                            out << elur_num_str << ',' << QString::number(m_elapsed.nsecsElapsed() / 1e6, 'f', 3) << ',' << elurx_str <<','<< elury_str <<','<< elurz_str <<'\n';
                        }
                    }
                }
            }
        }
        else if(str == QString::fromUtf8(mode_data_export_start_ack))   //启动数据导出模式
        {
            flag_exp = false;
            flag_test = false;
            flag_erease = false;
            flag_export = true;

            ui->pind_port->setEnabled(false);
            ui->erease_flash->setEnabled(false);
            ui->test_mode->setEnabled(false);
            ui->exp_mode->setEnabled(false);
            ui->export_data_2->setEnabled(false);

            showWarning2();

            setLED(ui->led_3, GREEN, 30);

        }
        else if (flag_export == true)
        {
            if (str == QString::fromUtf8(mode_data_export_stop_ack)) //停止导出模式成功
            {
                flag_export = false;

                //改变模式标志
                ui->pind_port->setEnabled(true);
                ui->erease_flash->setEnabled(true);
                ui->test_mode->setEnabled(true);
                ui->exp_mode->setEnabled(true);
                ui->export_data_2->setEnabled(true);

                m_file_export_acc.close();
                m_file_export_rot.close();
                m_file_export_elur.close();

                merge_csv_files_horizontally(fullPath_export, fullPath_export_acc, fullPath_export_rot, fullPath_export_elur);

                if (warningBox && !warningBox->isHidden())
                {  // 检查窗口是否存在且未隐藏
                    warningBox->close();
                }

                setLED(ui->led_3, RED, 30);
            }
            else
            {
                qDebug() << "copySize: " << copySize;

                QString hexOutput;
                for (int i = 0; i < copySize; ++i) {
                    hexOutput.append(QString("%1 ").arg(static_cast<int>(dataArray_test[i]), 2, 16, QChar('0')));
                }
                qDebug() << "Hex output: " << hexOutput;

                for(int i = 0; i < DataNum; i++)
                {
                    uint32_t combined = 0;

                    if( dataArray_test[i * DataLen] == 0xaa )
                    {
                        if( dataArray_test[i * DataLen + 1] == 0x01 )
                        {
                            combined = ((uint32_t)dataArray_test[DataStart + i * DataLen]     << 24) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 1] << 16) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 2] << 8)  |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 3]);
                            memcpy(&acc_exp[0], &combined, sizeof(float));
                            accx_str = QString::number(acc_exp[0]);
                            combined = ((uint32_t)dataArray_test[DataStart + i * DataLen + 4] << 24) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 5] << 16) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 6] << 8)  |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 7]);
                            memcpy(&acc_exp[1], &combined, sizeof(float));
                            accy_str = QString::number(acc_exp[1]);
                            combined = ((uint32_t)dataArray_test[DataStart + i * DataLen + 8]  << 24) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 9]  << 16) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 10] << 8)  |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 11]);
                            memcpy(&acc_exp[2], &combined, sizeof(float));
                            accz_str = QString::number(acc_exp[2]);
                            combined = ((uint32_t)dataArray_test[DataStart + i * DataLen + 12]  << 24) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 13]  << 16) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 14]  << 8)  |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 15]);
                            memcpy(&acc_num, &combined, sizeof(uint32_t));
                            acc_num_str = QString::number(acc_num);

                            if (m_file_export_acc.isOpen())
                            {
                                QTextStream out(&m_file_export_acc);

                                out << acc_num_str << ',' << QString::number(m_elapsed.nsecsElapsed() / 1e6, 'f', 3) << ',' << accx_str <<','<< accy_str <<','<< accz_str <<'\n';
                            }
                        }
                        else if ( dataArray_test[i * DataLen + 1] == 0x02 )
                        {
                            combined = ((uint32_t)dataArray_test[DataStart + i * DataLen]     << 24) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 1] << 16) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 2] << 8)  |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 3]);
                            memcpy(&rot_exp[0], &combined, sizeof(float));
                            rotx_str = QString::number(rot_exp[0]);
                            combined = ((uint32_t)dataArray_test[DataStart + i * DataLen + 4] << 24) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 5] << 16) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 6] << 8)  |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 7]);
                            memcpy(&rot_exp[1], &combined, sizeof(float));
                            roty_str = QString::number(rot_exp[1]);
                            combined = ((uint32_t)dataArray_test[DataStart + i * DataLen + 8]  << 24) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 9]  << 16) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 10] << 8)  |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 11]);
                            memcpy(&rot_exp[2], &combined, sizeof(float));
                            rotz_str = QString::number(rot_exp[2]);
                            combined = ((uint32_t)dataArray_test[DataStart + i * DataLen+ 12]  << 24) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen+ 13]  << 16) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen+ 14]  << 8)  |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen+ 15]);
                            memcpy(&rot_num, &combined, sizeof(uint32_t));
                            rot_num_str = QString::number(rot_num);
                            if (m_file_export_rot.isOpen())
                            {
                                QTextStream out(&m_file_export_rot);

                                out << rot_num_str << ',' << QString::number(m_elapsed.nsecsElapsed() / 1e6, 'f', 3) << ',' << rotx_str <<','<< roty_str <<','<< rotz_str <<'\n';
                            }
                        }
                        else if ( dataArray_test[i * DataLen + 1] == 0x03 )
                        {
                            combined = ((uint32_t)dataArray_test[DataStart + i * DataLen]     << 24) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 1] << 16) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 2] << 8)  |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 3]);
                            memcpy(&elur_exp[0], &combined, sizeof(float));
                            elurx_str = QString::number(elur_exp[0]);
                            combined = ((uint32_t)dataArray_test[DataStart + i * DataLen + 4] << 24) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 5] << 16) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 6] << 8)  |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 7]);
                            memcpy(&elur_exp[1], &combined, sizeof(float));
                            elury_str = QString::number(elur_exp[1]);
                            combined = ((uint32_t)dataArray_test[DataStart + i * DataLen + 8]  << 24) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 9]  << 16) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 10] << 8)  |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 11]);
                            memcpy(&elur_exp[2], &combined, sizeof(float));
                            elurz_str = QString::number(elur_exp[2]);
                            combined = ((uint32_t)dataArray_test[DataStart + i * DataLen + 12]  << 24) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 13]  << 16) |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 14]  << 8)  |
                                       ((uint32_t)dataArray_test[DataStart + i * DataLen + 15]);
                            memcpy(&elur_num, &combined, sizeof(uint32_t));
                            elur_num_str = QString::number(elur_num);

                            if (m_file_export_elur.isOpen())
                            {
                                QTextStream out(&m_file_export_elur);

                                out << elur_num_str << ',' << QString::number(m_elapsed.nsecsElapsed() / 1e6, 'f', 3) << ',' << elurx_str <<','<< elury_str <<','<< elurz_str <<'\n';
                            }
                        }
                    }
                }

                //发送继续命令
                QString  targetIP=ui->device_ip->text();  //目标IP
                QHostAddress  targetAddr(targetIP);

                QString targetPort_str = ui->device_port->text();//目标port
                quint16 targetPort = targetPort_str.toUShort();  // 使用 toUShort 转换

                QString  msg= "01010101010101";
                QByteArray  str=msg.toUtf8();
                udpSocket->writeDatagram(str,targetAddr,targetPort); //发出数据报
            }
        }
        else if(str == QString::fromUtf8(connect_test_ack))             //连接状态正常
        {
            flag_connect_ok = 0;
            qDebug() << "connect_test_ack";
        }

        if ( (dataArray_test[0] == 0xfa) && (dataArray_test[1] == 0xaf) ) //电量
        {
            battery_value[0] = ((int)dataArray_test[3] << 8) | ((int)dataArray_test[2]);
            battery_value[1] = ((int)dataArray_test[5] << 8) | ((int)dataArray_test[4]);
            battery_raw[0] = ((int)dataArray_test[7] << 8) | ((int)dataArray_test[6]);
            battery_raw[1] = ((int)dataArray_test[9] << 8) | ((int)dataArray_test[8]);

            battery = battery_value[0] * (2.0 * battery_raw[1] / battery_raw[0]);

            QString formattedBattery = QString::number(battery / 1000, 'f', 3) + " V";

            QLabel* batteryLabel = this->findChild<QLabel*>("battery");
            if (batteryLabel)
            {
                batteryLabel->setText(formattedBattery);
            }

            qDebug() << "battery_value[0]: " << battery_value[0];
            qDebug() << "battery_value[1]: " << battery_value[1];
            qDebug() << "battery_raw[0]: " << battery_raw[0];
            qDebug() << "battery_raw[1]: " << battery_raw[1];
            qDebug() << "battery: " << battery;

        }
    }
}

void MainWindow::showWarning()
{
    QString dlgTitle = "warning";
    QString strInfo = "正在擦除存储器！！！";

    warningBox = new QMessageBox(QMessageBox::Warning, dlgTitle, strInfo,
                                 QMessageBox::NoButton, this);
    // warningBox->setAttribute(Qt::WA_DeleteOnClose); // 关闭时自动删除

    warningBox->setFixedSize(400, 200);

    warningBox->show(); // 非模态显示
}

void MainWindow::showWarning1()
{
    QString dlgTitle = "warning";
    QString strInfo = "请检查保存路径和文件名！！！";

    warningBox = new QMessageBox(QMessageBox::Warning, dlgTitle, strInfo,
                                 QMessageBox::NoButton, this);
    warningBox->setAttribute(Qt::WA_DeleteOnClose); // 关闭时自动删除

    warningBox->setFixedSize(400, 200);

    warningBox->show(); // 非模态显示
}

void MainWindow::showWarning2()
{
    QString dlgTitle = "warning";
    QString strInfo = "正在导出数据，请等待！！！";

    warningBox = new QMessageBox(QMessageBox::Warning, dlgTitle, strInfo,
                                 QMessageBox::NoButton, this);
    // warningBox->setAttribute(Qt::WA_DeleteOnClose); // 关闭时自动删除

    warningBox->setFixedSize(400, 200);

    warningBox->show(); // 非模态显示
}

void MainWindow::showWarning3()
{
    QString dlgTitle = "warning";
    QString strInfo = "连接异常，请检查设备状态！";

    warningBox = new QMessageBox(QMessageBox::Warning, dlgTitle, strInfo,
                                 QMessageBox::NoButton, this);
    warningBox->setAttribute(Qt::WA_DeleteOnClose); // 关闭时自动删除

    warningBox->setFixedSize(400, 200);

    warningBox->show(); // 非模态显示
}

void MainWindow::showWarning4()
{
    QString dlgTitle = "warning";
    QString strInfo = "网络连接异常，请检查网络状态！";

    warningBox = new QMessageBox(QMessageBox::Warning, dlgTitle, strInfo,
                                 QMessageBox::NoButton, this);
    warningBox->setAttribute(Qt::WA_DeleteOnClose); // 关闭时自动删除

    warningBox->setFixedSize(400, 200);

    warningBox->show(); // 非模态显示
}

void MainWindow::on_pind_port_clicked()//监听和解除监听
{
    // 当前按钮文本决定操作类型
    bool isBinding = (ui->pind_port->text() == "监听端口");

    if (isBinding)
    {
        // 绑定端口逻辑
        QString portText = ui->local_port->text();
        bool conversionOk;
        quint16 port = portText.toUShort(&conversionOk);

        if (!conversionOk || port == 0)
        {
            QMessageBox::warning(this, "错误", "请输入有效的端口号！");
            return;
        }

        if (udpSocket->bind(QHostAddress::Any, port))
        {
            ui->pind_port->setText("解除监听");
        }
        else
        {
            QMessageBox::critical(this, "错误",
                                  QString("绑定端口失败: %1").arg(udpSocket->errorString()));
        }
    }
    else
    {
        // 解除绑定逻辑
        QString  targetIP=ui->device_ip->text();  //目标IP
        QHostAddress  targetAddr(targetIP);

        QString targetPort_str = ui->device_port->text();
        quint16 targetPort = targetPort_str.toUShort();  // 目标port

        if (targetIP.isEmpty() || targetPort_str.isEmpty())
        {
            // 强制解除绑定
            udpSocket->abort(); // 使用abort()确保立即释放端口
            ui->pind_port->setText("监听端口");
        }
        else
        {
            QString  msg= "01050004001bcc";
            QByteArray  str=msg.toUtf8();
            udpSocket->writeDatagram(str,targetAddr,targetPort); //发出数据报
        }
    }
}

void MainWindow::on_test_mode_clicked() //启动&&停止测试模式
{
    QString  targetIP=ui->device_ip->text();  //目标IP
    QHostAddress  targetAddr(targetIP);

    QString targetPort_str = ui->device_port->text();
    quint16 targetPort = targetPort_str.toUShort();  // 目标port

    if(flag_test == false)  //启动测试模式
    {
        QString  msg= "010500000298cd";
        QByteArray  str=msg.toUtf8();
        udpSocket->writeDatagram(str,targetAddr,targetPort); //发出启动测试模式命令
    }
    else                    //停止测试模式
    {
        QString  msg= "0105000200186c";
        QByteArray  str=msg.toUtf8();
        udpSocket->writeDatagram(str,targetAddr,targetPort); //发出停止测试模式命令
    }
}

void MainWindow::on_erease_flash_clicked() //启动擦除模式
{
    QString  targetIP=ui->device_ip->text();  //目标IP
    QHostAddress  targetAddr(targetIP);

    QString targetPort_str = ui->device_port->text();
    quint16 targetPort = targetPort_str.toUShort();  // 使用 toUShort 转换

    if (flag_erease == false)
    {
        QString  msg= "0105000005d90f";
        QByteArray  str=msg.toUtf8();
        udpSocket->writeDatagram(str,targetAddr,targetPort); //发出数据报
    }
}

void MainWindow::on_exp_mode_clicked() //启动&&停止实验模式
{
    QString fileDir = ui->data_path->text();
    QString fileName = ui->data_name->text();

    if (fileDir.isEmpty() || fileName.isEmpty())//如果为空
    {
        showWarning1();
    }
    else
    {
        QString  targetIP=ui->device_ip->text();  //目标IP
        QHostAddress  targetAddr(targetIP);

        QString targetPort_str = ui->device_port->text();
        quint16 targetPort = targetPort_str.toUShort();  // 使用 toUShort 转换

        if(flag_exp == false)
        {
            fullPath_exp = fileDir + "/" + fileName + "-exp" + ".csv";
            fullPath_exp_acc = fileDir + "/" + fileName + "-exp-acc" + ".csv";
            fullPath_exp_rot = fileDir + "/" + fileName + "-exp-rot" + ".csv";
            fullPath_exp_elur = fileDir + "/" + fileName + "-exp-elur" + ".csv";

            std::string stdFileDir = fileDir.toStdString();
            std::string stdFileName = fileName.toStdString();
            std::string stdfullPath = fullPath.toStdString();

            QFile file_exp(fullPath_exp);

            // 打开数据文件并保持打开（实验期间持续追加写，避免每包数据都 open/close）
            m_file_exp_acc.close();
            m_file_exp_acc.setFileName(fullPath_exp_acc);
            m_file_exp_rot.close();
            m_file_exp_rot.setFileName(fullPath_exp_rot);
            m_file_exp_elur.close();
            m_file_exp_elur.setFileName(fullPath_exp_elur);

            if (!m_file_exp_acc.open(QIODevice::Append | QIODevice::Text))
                qDebug() << "Failed to open" << fullPath_exp_acc;
            if (!m_file_exp_rot.open(QIODevice::Append | QIODevice::Text))
                qDebug() << "Failed to open" << fullPath_exp_rot;
            if (!m_file_exp_elur.open(QIODevice::Append | QIODevice::Text))
                qDebug() << "Failed to open" << fullPath_exp_elur;

            // 如果总文件不存在，写入表头
            if (!file_exp.exists())
            {
                if (file_exp.open(QIODevice::Append | QIODevice::Text))
                {
                    QTextStream out(&file_exp);

                    out << "acc_num,time_ms,accx,accy,accz,rot_num,time_ms,rotx,roty,rotz,elur_num,time_ms,roll,pitch,yaw\n";

                    file_exp.close(); // 记得关闭文件
                }
            }

            // 新文件（大小0）才写表头
            if (m_file_exp_acc.size() == 0)
            {
                QTextStream out(&m_file_exp_acc);
                out << "acc_num,time_ms,accx,accy,accz\n";
            }

            if (m_file_exp_rot.size() == 0)
            {
                QTextStream out(&m_file_exp_rot);
                out << "rot_num,time_ms,rotx,roty,rotz\n";
            }

            if (m_file_exp_elur.size() == 0)
            {
                QTextStream out(&m_file_exp_elur);
                out << "elur_num,time_ms,roll,pitch,yaw\n";
            }

            m_elapsed.restart();   //实验模式开始计时（CSV 时间戳基准）

            QString  msg= "0105000001d8cc";
            QByteArray  str=msg.toUtf8();
            udpSocket->writeDatagram(str,targetAddr,targetPort); //发出数据报
        }
        else
        {
            QString  msg= "0105000100189c";
            QByteArray  str=msg.toUtf8();
            udpSocket->writeDatagram(str,targetAddr,targetPort); //发出数据报
        }
    }
}

void MainWindow::on_select_path_clicked() // 选择路径
{
    QString existDir = ui->data_path->text();

    QString rootDir = QDir::rootPath();

    QString aDir = QFileDialog::getExistingDirectory(
        this,
        "选择一个路径",
        rootDir,
        QFileDialog::ShowDirsOnly
        );

    std::string dirStr = aDir.toStdString();

    if (aDir.isEmpty())
    {
        ui->data_path->setText(existDir); // 用户取消选择，保持原路径
    }
    else
    {
        ui->data_path->setText(aDir); // 更新选择的路径
    }
}

void MainWindow::on_open_path_clicked() //打开路径
{
    QString path = ui->data_path->text(); // 获取要打开的路径
    if (path.isEmpty())
    {
        path = QDir::homePath(); // 如果路径为空，默认打开用户主目录
    }

    QUrl url = QUrl::fromLocalFile(path);

    // 打开文件管理器
    bool success = QDesktopServices::openUrl(url);
    if (!success)
    {
        QMessageBox::warning(this, "警告", "无法打开文件资源管理器！");
    }
}

void MainWindow::on_export_data_2_clicked() //启动导出数据模式
{
    QString fileDir = ui->data_path->text();
    QString fileName = ui->data_name->text();

    if (fileDir.isEmpty() || fileName.isEmpty())//如果为空
    {
        showWarning1();
    }
    else
    {
        QString  targetIP=ui->device_ip->text();  //目标IP
        QHostAddress  targetAddr(targetIP);

        QString targetPort_str = ui->device_port->text();
        quint16 targetPort = targetPort_str.toUShort();  // 使用 toUShort 转换

        if(flag_export == false)
        {
            fullPath_export      = fileDir + "/" + fileName + "-export" + ".csv";
            fullPath_export_acc  = fileDir + "/" + fileName + "-export_acc" + ".csv";
            fullPath_export_rot  = fileDir + "/" + fileName + "-export_rot" + ".csv";
            fullPath_export_elur = fileDir + "/" + fileName + "-export_elur" + ".csv";

            std::string stdFileDir = fileDir.toStdString();
            std::string stdFileName = fileName.toStdString();
            std::string stdfullPath = fullPath.toStdString();

            QFile file_export(fullPath_export);

            // 打开数据文件并保持打开（导出期间持续追加写，避免每包数据都 open/close）
            m_file_export_acc.close();
            m_file_export_acc.setFileName(fullPath_export_acc);
            m_file_export_rot.close();
            m_file_export_rot.setFileName(fullPath_export_rot);
            m_file_export_elur.close();
            m_file_export_elur.setFileName(fullPath_export_elur);

            if (!m_file_export_acc.open(QIODevice::Append | QIODevice::Text))
                qDebug() << "Failed to open" << fullPath_export_acc;
            if (!m_file_export_rot.open(QIODevice::Append | QIODevice::Text))
                qDebug() << "Failed to open" << fullPath_export_rot;
            if (!m_file_export_elur.open(QIODevice::Append | QIODevice::Text))
                qDebug() << "Failed to open" << fullPath_export_elur;

            // 如果总文件不存在，写入表头
            if (!file_export.exists())
            {
                if (file_export.open(QIODevice::Append | QIODevice::Text))
                {
                    QTextStream out(&file_export);

                    out << "acc_num,time_ms,accx,accy,accz,rot_num,time_ms,rotx,roty,rotz,elur_num,time_ms,roll,pitch,yaw\n";

                    file_export.close(); // 记得关闭文件
                }
            }

            // 新文件（大小0）才写表头
            if (m_file_export_acc.size() == 0)
            {
                QTextStream out(&m_file_export_acc);
                out << "acc_num,time_ms,accx,accy,accz\n";
            }

            if (m_file_export_rot.size() == 0)
            {
                QTextStream out(&m_file_export_rot);
                out << "rot_num,time_ms,rotx,roty,rotz\n";
            }

            if (m_file_export_elur.size() == 0)
            {
                QTextStream out(&m_file_export_elur);
                out << "elur_num,time_ms,roll,pitch,yaw\n";
            }

            m_elapsed.restart();   //导出模式开始计时（CSV 时间戳基准）

            QString  msg= "0105000003590d";
            QByteArray  str=msg.toUtf8();
            udpSocket->writeDatagram(str,targetAddr,targetPort); //发出数据报
        }
    }
}

bool MainWindow::merge_csv_files_horizontally(const QString &outputFile,
                                              const QString &file1,
                                              const QString &file2,
                                              const QString &file3)
{
    QFile inFile1(file1);
    QFile inFile2(file2);
    QFile inFile3(file3);
    QFile outFile(outputFile);

    if (!inFile1.open(QIODevice::ReadOnly | QIODevice::Text) ||
        !inFile2.open(QIODevice::ReadOnly | QIODevice::Text) ||
        !inFile3.open(QIODevice::ReadOnly | QIODevice::Text) ||
        !outFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug() << "Failed to open one or more files.";
        return false;
    }

    QTextStream inStream1(&inFile1);
    QTextStream inStream2(&inFile2);
    QTextStream inStream3(&inFile3);
    QTextStream outStream(&outFile);

    // 1. 计算每个文件的行数
    int lineCount1 = 0, lineCount2 = 0, lineCount3 = 0;
    while (!inStream1.atEnd()) { inStream1.readLine(); lineCount1++; }
    while (!inStream2.atEnd()) { inStream2.readLine(); lineCount2++; }
    while (!inStream3.atEnd()) { inStream3.readLine(); lineCount3++; }

    // 2. 重置文件指针
    inFile1.reset(); inFile2.reset(); inFile3.reset(); // 重置底层句柄
    inStream1.seek(0); inStream2.seek(0); inStream3.seek(0); // 重置流位置

    // 获取最大行数
    int maxLines = std::max({lineCount1, lineCount2, lineCount3});

    // 3. 按最大行数合并
    for (int i = 0; i < maxLines; i++) {
        QString line1 = i < lineCount1 ? inStream1.readLine() : "";
        QString line2 = i < lineCount2 ? inStream2.readLine() : "";
        QString line3 = i < lineCount3 ? inStream3.readLine() : "";

        // --- 文件 1 处理 ---
        QStringList cols1 = line1.split(',');
        if (!line1.isEmpty() && cols1.size() != 5) {
            qDebug() << "Invalid line format in file1 (expected 5 cols):" << line1;
            // 可以选择 continue 或 handle error，这里保持原逻辑继续
            continue;
        }

        // --- 文件 2 处理 ---
        QStringList cols2 = line2.split(',');
        if (!line2.isEmpty() && cols2.size() != 5) {
            qDebug() << "Invalid line format in file2 (expected 5 cols):" << line2;
            continue;
        }

        // --- 文件 3 处理 ---
        QStringList cols3 = line3.split(',');
        if (!line3.isEmpty() && cols3.size() != 5) {
            qDebug() << "Invalid line format in file3 (expected 5 cols):" << line3;
            continue;
        }

        QString mergedLine =
            (line1.isEmpty() ? ",,,," : cols1.join(",")) + "," +
            (line2.isEmpty() ? ",,,," : cols2.join(",")) + "," +
            (line3.isEmpty() ? ",,,," : cols3.join(","));

        outStream << mergedLine << "\n";
    }

    inFile1.close(); inFile2.close(); inFile3.close();
    outFile.close();

    qDebug() << "Files merged successfully into" << outputFile
             << "(lines:" << maxLines << ")";

    // 删除源文件
    QFile::remove(file1);
    QFile::remove(file2);
    QFile::remove(file3);

    return true;
}

void MainWindow::restore_state()
{
    // 恢复状态函数
    qDebug() << "restore state !";

    flag_connect_ok = 0;

    udpSocket->close();
    ui->device_ip->clear();    // 清空 IP 输入框
    ui->device_port->clear();  // 清空端口输入框

    ui->erease_flash->setEnabled(false);
    ui->test_mode->setEnabled(false);
    ui->exp_mode->setEnabled(false);
    ui->export_data_2->setEnabled(false);

    ui->pind_port->setText("监听端口");
    setLED(ui->led_1, RED, 30);

    m_timer_connect->stop(); //停止定时器
}

void MainWindow::restore_state2()
{
    // 恢复状态函数
    qDebug() << "restore state2 !";

    if(flag_exp)
    {
        m_file_exp_acc.close();
        m_file_exp_rot.close();
        m_file_exp_elur.close();
        merge_csv_files_horizontally(fullPath_exp, fullPath_exp_acc, fullPath_exp_rot, fullPath_exp_elur);
    }

    flag_test = false;
    flag_exp = false;
    udpSocket->close();
    ui->device_ip->clear();    // 清空 IP 输入框
    ui->device_port->clear();  // 清空端口输入框

    ui->erease_flash->setEnabled(false);
    ui->test_mode->setEnabled(false);
    ui->exp_mode->setEnabled(false);
    ui->export_data_2->setEnabled(false);
    ui->pind_port->setEnabled(true);

    ui->pind_port->setText("监听端口");
    ui->test_mode->setText("启动测试模式");
    ui->exp_mode->setText("启动实验模式");

    setLED(ui->led_1, RED, 30);
    setLED(ui->led_2, RED, 30);

    flag_connect_ok = 0;

    m_timer_connect->stop(); //停止定时器

}

void MainWindow::do_timer_connect_timeout()
{
    if( !flag_exp && !flag_test && !flag_export && !flag_erease)
    {
        qDebug() << "no mode test";


        QString  targetIP=ui->device_ip->text();  //目标IP
        QHostAddress  targetAddr(targetIP);

        QString targetPort_str = ui->device_port->text();
        quint16 targetPort = targetPort_str.toUShort();  // 目标port

        QString  msg= "0105000006990e";
        QByteArray  str=msg.toUtf8();
        udpSocket->writeDatagram(str,targetAddr,targetPort); //发出检测连接

        flag_connect_ok += 1;
        if(flag_connect_ok >= 5)
        {
            restore_state();
            showWarning3();
        }

        qDebug() << "flag_connect_ok: " << static_cast<int>(flag_connect_ok);
    }
    else if( (flag_exp == true) || (flag_test == true) )
    {
        qDebug() << "have mode test";
    }
}

void MainWindow::do_timer_number_timeout()
{
    qDebug() << "have mode test";
    qDebug() << "flag_number: " << static_cast<int>(flag_number);

    if(flag_number < 50)
    {
        m_timer_number->stop();     //停止定时器

        restore_state2();
        showWarning4();
    }
    else
    {
        flag_number = 0;
    }
}

void MainWindow::do_timer_redraw_timeout()
{
    // 加速度曲线：先处理越界清空，再批量刷新
    if (m_acc_clear)
    {
        series_accx->clear();
        series_accy->clear();
        series_accz->clear();
        m_acc_clear = false;
    }
    if (!m_buf_accx.isEmpty())
    {
        series_accx->append(m_buf_accx);
        series_accy->append(m_buf_accy);
        series_accz->append(m_buf_accz);
        m_buf_accx.clear();
        m_buf_accy.clear();
        m_buf_accz.clear();
    }

    // 角速度曲线
    if (m_rot_clear)
    {
        series_rotx->clear();
        series_roty->clear();
        series_rotz->clear();
        m_rot_clear = false;
    }
    if (!m_buf_rotx.isEmpty())
    {
        series_rotx->append(m_buf_rotx);
        series_roty->append(m_buf_roty);
        series_rotz->append(m_buf_rotz);
        m_buf_rotx.clear();
        m_buf_roty.clear();
        m_buf_rotz.clear();
    }

    // 欧拉角曲线
    if (m_elur_clear)
    {
        series_roll->clear();
        series_pitch->clear();
        series_yaw->clear();
        m_elur_clear = false;
    }
    if (!m_buf_roll.isEmpty())
    {
        series_roll->append(m_buf_roll);
        series_pitch->append(m_buf_pitch);
        series_yaw->append(m_buf_yaw);
        m_buf_roll.clear();
        m_buf_pitch.clear();
        m_buf_yaw.clear();
    }
}
