#pragma once

#include <QUdpSocket>
#include <QNetworkDatagram>
#include <QHostAddress>
#include <QMainWindow>
#include <QTimer>
#include <vector>

#include "purepursuit.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void onTimerTick();
    void on_btnStart_clicked();
    void on_btnStop_clicked();
    void on_comboPath_currentIndexChanged(int index);

    // Slot nhận Odometry từ robot và lệnh từ MediaPipe
    void readPendingDatagrams();

private:
    void connectSignals();
    void syncParametersFromUi();
    void generateCirclePath (double radius, int n);
    void generateFigure8Path(double scale,  int n);

    Ui::MainWindow *ui{nullptr};
    PurePursuit  m_controller;
    Pose2D       m_pose; // Giá trị này lấy từ robot

    QTimer *m_timer{nullptr};
    static constexpr int LOOP_MS = 50;
    double m_dt{LOOP_MS / 1000.0};

    QUdpSocket *m_udpSocket{nullptr};
    QHostAddress m_robotIp{"192.168.4.1"};
    quint16 m_robotPort{1234};

    // =========================================================================
    // CÁC THÀNH PHẦN THÊM MỚI ĐỂ QUẢN LÝ PHÂN CẤP ƯU TIÊN AI (MEDIAPIPE)
    // =========================================================================
    bool m_isMediaPipeActive{false};    // Cờ bật/tắt chế độ can thiệp bằng cử chỉ tay
    QString m_mpCommand{"NONE"};        // Chuỗi lưu trạng thái lệnh tay (FORWARD/BACKWARD/STOP)
    QTimer *m_mpWatchdogTimer{nullptr}; // Watchdog kiểm tra mất kết nối hoặc rụt tay để trả quyền
};