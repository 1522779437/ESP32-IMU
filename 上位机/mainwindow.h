#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QUdpSocket>
#include <QLabel>
#include <QtCharts>
#include <QTimer>
#include <QFile>
#include <QVector>
#include <QElapsedTimer>

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

private:
    QLabel  *labListen;                 //状态栏标签
    QLabel  *labSocketState;            //状态栏标签

    QChart *chart_acc;
    QChart *chart_rot;
    QChart *chart_elur;

    QUdpSocket *udpSocket = nullptr;    //UDP通讯的Socket

    QString getLocalIP();               //获取本机IP地址
    QList<QString> getAllLocalIPv4Addresses();

    QTimer  *m_timer_connect; //定时器
    QTimer  *m_timer_number;  //定时器
    QTimer  *m_timer_redraw;  //图表批量刷新定时器

    // 实验/导出模式期间保持打开的数据文件，避免每包数据都 open/close
    QFile m_file_exp_acc;
    QFile m_file_exp_rot;
    QFile m_file_exp_elur;
    QFile m_file_export_acc;
    QFile m_file_export_rot;
    QFile m_file_export_elur;

    // 实验/导出模式相对时间基准（CSV 时间戳）
    QElapsedTimer m_elapsed;

    // 图表批量刷新缓冲：数据先入缓冲，定时器统一刷新，降低重绘频率
    QVector<QPointF> m_buf_accx, m_buf_accy, m_buf_accz;
    QVector<QPointF> m_buf_rotx, m_buf_roty, m_buf_rotz;
    QVector<QPointF> m_buf_roll, m_buf_pitch, m_buf_yaw;
    bool m_acc_clear = false;
    bool m_rot_clear = false;
    bool m_elur_clear = false;

    void showWarning();
    void showWarning1();
    void showWarning2();
    void showWarning3();
    void showWarning4();

    void createChart_acc();
    void createChart_rot();
    void createChart_elur();
    void prepareData(int flag);
    void setLED(QLabel* label, int color, int size);
    void updatePeakAccDisplay(float x, float y, float z);

    bool merge_csv_files_horizontally(const QString &outputFile,
                                      const QString &file1,
                                      const QString &file2,
                                      const QString &file3);

    void closeEvent(QCloseEvent *event);

    void restore_state();

    void restore_state2();

public:
    MainWindow(QWidget *parent = nullptr);

    ~MainWindow();

private slots:
    void getIP();

    void do_socketStateChange(QAbstractSocket::SocketState socketState);

    void do_socketReadyRead();   //读消息及处理

    void on_pind_port_clicked();

    void on_test_mode_clicked();

    void on_erease_flash_clicked();

    void on_exp_mode_clicked();

    void on_select_path_clicked();

    void on_open_path_clicked();

    void on_export_data_2_clicked();

    void do_timer_connect_timeout();

    void do_timer_number_timeout();

    void do_timer_redraw_timeout();

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
