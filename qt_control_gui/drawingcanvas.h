#pragma once

#include <QWidget>
#include <QPointF>
#include <vector>

#include "purepursuit.h"   // Dùng Point2D, Pose2D (không phụ thuộc Qt)

// ─────────────────────────────────────────────────────────────────────────────
// Class: DrawingCanvas
//
//   Widget chuyên trách VẼ – không chứa logic điều khiển.
//   Được promote từ QWidget trong Qt Designer (mainwindow.ui).
//
//   Cách promote trong Qt Designer:
//     Right-click QWidget → "Promote to…"
//     Base class name : QWidget
//     Promoted class  : DrawingCanvas
//     Header file     : drawingcanvas.h
// ─────────────────────────────────────────────────────────────────────────────
class DrawingCanvas : public QWidget
{
    Q_OBJECT

public:
    explicit DrawingCanvas(QWidget *parent = nullptr);

public slots:
    // Hai slots này được nối từ MainWindow mỗi khi dữ liệu thay đổi.
    void setPath(const std::vector<Point2D> &path);
    void setRobotPose(const Pose2D &pose);

    // Xoá vết robot (gọi khi reset mô phỏng).
    void clearTrace();

protected:
    void paintEvent(QPaintEvent *event) override;

    // Cho phép người dùng phóng to/thu nhỏ bằng chuột (tuỳ chọn).
    void wheelEvent(QWheelEvent *event) override;

private:
    // ── Dữ liệu vẽ ──────────────────────────────────────────────────────
    std::vector<Point2D> m_path;       // Quỹ đạo tham chiếu
    std::vector<Pose2D>  m_trace;      // Vết robot thực tế
    Pose2D               m_robotPose;  // Pose hiện tại

    // ── Tham số hiển thị ─────────────────────────────────────────────────
    double m_scale{80.0};   // px / m  (thay đổi khi dùng wheelEvent)
    static constexpr double SCALE_MIN = 20.0;
    static constexpr double SCALE_MAX = 300.0;

    // ── Helpers ──────────────────────────────────────────────────────────
    // Chuyển toạ độ thế giới (m) → pixel canvas (gốc tâm widget).
    QPointF toCanvas(double worldX, double worldY) const;
    QPointF toCanvas(const Point2D &p) const { return toCanvas(p.x, p.y); }
    QPointF toCanvas(const Pose2D  &p) const { return toCanvas(p.x, p.y); }

    void drawGrid    (QPainter &p) const;
    void drawAxes    (QPainter &p) const;
    void drawPath    (QPainter &p) const;
    void drawTrace   (QPainter &p) const;
    void drawRobot   (QPainter &p) const;
};