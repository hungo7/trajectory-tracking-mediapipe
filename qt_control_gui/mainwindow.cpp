#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QNetworkDatagram>
#include <cmath>
#include <QDebug> // Thêm debug để quan sát trạng thái switch luồng

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_robotIp = QHostAddress("192.168.4.1");
    m_robotPort = 1234;

    // --- KHỞI TẠO BỘ ĐIỀU PHỐI ƯU TIÊN ---
    m_isMediaPipeActive = false;
    m_mpCommand = "NONE";

    // Bộ Watchdog: Nếu quá 300ms không nhận được gói tin từ MediaPipe,
    // coi như người dùng rụt tay lại -> Trả quyền cho Pure Pursuit bám quỹ đạo tự động.
    m_mpWatchdogTimer = new QTimer(this);
    m_mpWatchdogTimer->setSingleShot(true);
    connect(m_mpWatchdogTimer, &QTimer::timeout, this, [this]() {
        m_isMediaPipeActive = false;
        qDebug() << ">>> Timeout MediaPipe: Tra quyen lai cho Pure Pursuit.";
    });

    connectSignals();

    ui->btnStop->setEnabled(false);
    generateCirclePath(0.6, 200);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::connectSignals()
{
    m_timer = new QTimer(this);
    m_timer->setInterval(50); // LOOP_MS = 50
    connect(m_timer, &QTimer::timeout, this, &MainWindow::onTimerTick);

    m_udpSocket = new QUdpSocket(this);
    m_udpSocket->bind(QHostAddress::AnyIPv4, 1235);

    connect(m_udpSocket, &QUdpSocket::readyRead,
            this, &MainWindow::readPendingDatagrams);
}

void MainWindow::on_btnStart_clicked()
{
    if (m_timer->isActive()) return;

    syncParametersFromUi();
    m_controller.reset();

    m_pose = Pose2D{0.0, 0.0, 0.0};
    ui->canvasWidget->clearTrace();
    ui->canvasWidget->setRobotPose(m_pose);

    ui->btnStart->setEnabled(false);
    ui->btnStop->setEnabled(true);

    for (int i = 0; i < 5; ++i)
        m_udpSocket->writeDatagram("RESET\n", m_robotIp, m_robotPort);

    QTimer::singleShot(300, this, [this]() {
        if (!m_timer->isActive())
            m_timer->start();
    });
}

void MainWindow::on_btnStop_clicked()
{
    m_timer->stop();
    m_mpWatchdogTimer->stop();
    m_isMediaPipeActive = false;

    QString stopCmd = "0.0,0.0\n";
    m_udpSocket->writeDatagram(stopCmd.toUtf8(), m_robotIp, m_robotPort);

    ui->btnStart->setEnabled(true);
    ui->btnStop->setEnabled(false);
}

void MainWindow::on_comboPath_currentIndexChanged(int index)
{
    if (m_timer->isActive()) on_btnStop_clicked();

    if (index == 0) {
        generateCirclePath(0.6, 200);
    } else {
        generateFigure8Path(0.6, 400);
    }
}

// =========================================================================
// TẦNG GỬI LỆNH XUỐNG FIRMWARE (NƠI THỰC HIỆN PHÂN CẤP ƯU TIÊN ĐIỀU KHIỂN)
// =========================================================================
void MainWindow::onTimerTick()
{
    syncParametersFromUi();
    QString data;

    // KIỂM TRA QUYỀN ƯU TIÊN ĐẦU VÀO
    if (m_isMediaPipeActive) {
        // --- 1. ƯU TIÊN CAO NHẤT: BỊ ĐÈ BỞI LỆNH CỬ CHỈ TAY (MEDIAPIPE) ---
        double v_target = 0.0;
        double omega_target = 0.0;

        if (m_mpCommand == "TURN_LEFT") {
            v_target = 0.25;       // Tốc độ tiến cấu hình test (m/s)
            omega_target = 2.0;
        } else if (m_mpCommand == "TURN_RIGHT") {
            v_target = 0.25;      // Tốc độ lùi cấu hình test (m/s)
            omega_target = -2.0;
        }

        data = QString("%1,%2\n").arg(v_target, 0, 'f', 4).arg(omega_target, 0, 'f', 4);
        qDebug() << ">>> DANG UU TIÊN MEDIAPIPE -> Gửi:" << data.trimmed();

    } else {
        // --- 2. LUỒNG THẤP HƠN: CHẠY QUỸ ĐẠO TỰ ĐỘNG (PURE PURSUIT) ---
        TwistCmd cmd = m_controller.compute(m_pose, m_dt);

        if (std::fabs(cmd.v) < 1e-4 && std::fabs(cmd.omega) < 1e-4) {
            data = "0.0,0.0\n";
        } else {
            data = QString("%1,%2\n").arg(cmd.v, 0, 'f', 4).arg(cmd.omega, 0, 'f', 4);
        }
    }

    // Gửi gói tin duy nhất được chọn xuống ESP32 Firmware
    m_udpSocket->writeDatagram(data.toUtf8(), m_robotIp, m_robotPort);

    // Cập nhật vị trí lên Canvas đồ họa
    ui->canvasWidget->setRobotPose(m_pose);

    // Chỉ kết thúc quỹ đạo khi không bị can thiệp bởi cử chỉ tay và Pure Pursuit báo hoàn thành
    if (!m_isMediaPipeActive && m_controller.isGoalReached()) {
        on_btnStop_clicked();
        QMessageBox::information(this, "Hoàn thành", "Robot đã hoàn thành quỹ đạo!");
    }
}

// =========================================================================
// BỘ BỘ LỌC ĐẦU VÀO TRÊN CỔNG 1235 (PHÂN BIỆT ODOM VÀ MEDIAPIPE)
// =========================================================================
void MainWindow::readPendingDatagrams()
{
    while (m_udpSocket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = m_udpSocket->receiveDatagram();
        QString rawData = QString::fromUtf8(datagram.data()).trimmed();

        // 1. Kiểm tra nếu gói tin là các ký tự điều khiển từ Python
        if (rawData == "TURN_LEFT" || rawData == "TURN_RIGHT") {

            m_isMediaPipeActive = true;       // Kích hoạt cờ chiếm quyền ưu tiên
            m_mpCommand = rawData;            // Lưu lại trạng thái nút lệnh
            m_mpWatchdogTimer->start(300);    // Làm mới (Reload) bộ đếm 300ms liên tục

            continue; // Bỏ qua phần xử lý odom phía dưới
        }

        // 2. Nếu không phải chữ điều khiển -> Xử lý dữ liệu số Odometry truyền lên từ Firmware
        QStringList list = rawData.split(',');
        if (list.size() == 3) {
            m_pose.x = list.at(0).toDouble();
            m_pose.y = list.at(1).toDouble();
            m_pose.theta = list.at(2).toDouble();

            ui->canvasWidget->setRobotPose(m_pose);
        }
    }
}

void MainWindow::syncParametersFromUi()
{
    auto toDouble = [](QLineEdit *le, double fallback) {
        bool ok;
        double v = le->text().toDouble(&ok);
        return ok ? v : fallback;
    };

    m_controller.setParameters(
        toDouble(ui->txtLookahead, 0.3),
        toDouble(ui->txtVMax, 0.2)
        );
}

void MainWindow::generateCirclePath(double radius, int n)
{
    std::vector<Point2D> pts;
    for (int i = 0; i <= n; ++i) {
        double t = (2.0 * M_PI * i) / n;
        pts.push_back({ radius * std::sin(t), radius - radius * std::cos(t) });
    }
    m_controller.setPath(pts);
    ui->canvasWidget->setPath(pts);
}

void MainWindow::generateFigure8Path(double scale, int n)
{
    std::vector<Point2D> pts;
    for (int i = 0; i <= n; ++i) {
        double t = 2.0 * M_PI * i / n;
        pts.push_back({ -scale * std::sin(t) * std::cos(t), -scale * std::cos(t) });
    }
    m_controller.setPath(pts);
    ui->canvasWidget->setPath(pts);
}

