#include "drawingcanvas.h"

#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Ctor
// ─────────────────────────────────────────────────────────────────────────────

DrawingCanvas::DrawingCanvas(QWidget *parent)
    : QWidget(parent)
{
    // Đặt nền tối để quỹ đạo nổi bật; tránh flash trắng khi resize.
    setAutoFillBackground(false);
    setAttribute(Qt::WA_OpaquePaintEvent);
    setMinimumSize(400, 400);
}

// ─────────────────────────────────────────────────────────────────────────────
// Public slots
// ─────────────────────────────────────────────────────────────────────────────

void DrawingCanvas::setPath(const std::vector<Point2D> &path)
{
    m_path = path;
    clearTrace();   // Quỹ đạo mới → reset vết cũ.
}

void DrawingCanvas::setRobotPose(const Pose2D &pose)
{
    m_robotPose = pose;
    m_trace.push_back(pose);
    if (m_trace.size() > 5000) {
        m_trace.erase(m_trace.begin());
    }
    update(); // Schedule repaint (không block UI thread).
}

void DrawingCanvas::clearTrace()
{
    m_trace.clear();
    update();
}

// ─────────────────────────────────────────────────────────────────────────────
// paintEvent – điểm vẽ duy nhất, gọi từng lớp theo thứ tự từ dưới lên
// ─────────────────────────────────────────────────────────────────────────────

void DrawingCanvas::paintEvent(QPaintEvent * /*event*/)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // Nền
    p.fillRect(rect(), QColor(14, 14, 22));

    drawGrid (p);
    drawAxes (p);
    drawPath (p);
    drawTrace(p);
    drawRobot(p);
}

// ─────────────────────────────────────────────────────────────────────────────
// wheelEvent – zoom bằng cuộn chuột
// ─────────────────────────────────────────────────────────────────────────────

void DrawingCanvas::wheelEvent(QWheelEvent *event)
{
    // numDegrees dương → cuộn lên → phóng to.
    double factor = (event->angleDelta().y() > 0) ? 1.15 : (1.0 / 1.15);
    m_scale = std::clamp(m_scale * factor, SCALE_MIN, SCALE_MAX);
    update();
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: worldToCanvas
// ─────────────────────────────────────────────────────────────────────────────

QPointF DrawingCanvas::toCanvas(double wx, double wy) const
{
    // Gốc đặt ở tâm widget; trục Y đảo (y_canvas tăng xuống dưới).
    return { width()  / 2.0 + wx * m_scale,
            height() / 2.0 - wy * m_scale };
}

// ─────────────────────────────────────────────────────────────────────────────
// Draw helpers
// ─────────────────────────────────────────────────────────────────────────────

void DrawingCanvas::drawGrid(QPainter &p) const
{
    p.setPen(QPen(QColor(30, 32, 50), 1, Qt::DotLine));
    // Lưới mỗi 1 m.
    double step = m_scale;
    double ox = std::fmod(width()  / 2.0, step);
    double oy = std::fmod(height() / 2.0, step);
    for (double x = ox; x < width();  x += step)
        p.drawLine(QPointF(x, 0), QPointF(x, height()));
    for (double y = oy; y < height(); y += step)
        p.drawLine(QPointF(0, y), QPointF(width(), y));
}

void DrawingCanvas::drawAxes(QPainter &p) const
{
    QPointF o = toCanvas(0.0, 0.0);
    p.setPen(QPen(QColor(60, 65, 100), 1));
    p.drawLine(QPointF(0, o.y()), QPointF(width(), o.y())); // X-axis
    p.drawLine(QPointF(o.x(), 0), QPointF(o.x(), height())); // Y-axis
}

void DrawingCanvas::drawPath(QPainter &p) const
{
    if (m_path.size() < 2) return;

    QPainterPath pp;
    pp.moveTo(toCanvas(m_path[0]));
    for (size_t i = 1; i < m_path.size(); ++i)
        pp.lineTo(toCanvas(m_path[i]));
    pp.closeSubpath();

    // Quỹ đạo tham chiếu: màu xanh lam, nét đứt nhẹ.
    p.setPen(QPen(QColor(64, 156, 255), 2, Qt::DashLine));
    p.setBrush(Qt::NoBrush);
    p.drawPath(pp);
}

void DrawingCanvas::drawTrace(QPainter &p) const
{
    if (m_trace.size() < 2) return;

    QPainterPath tp;
    tp.moveTo(toCanvas(m_trace[0]));
    for (size_t i = 1; i < m_trace.size(); ++i)
        tp.lineTo(toCanvas(m_trace[i]));

    // Vết thực tế: màu vàng, nét liền.
    p.setPen(QPen(QColor(255, 210, 60), 1.5));
    p.setBrush(Qt::NoBrush);
    p.drawPath(tp);
}

void DrawingCanvas::drawRobot(QPainter &p) const
{
    QPointF center = toCanvas(m_robotPose);

    p.save();
    p.translate(center);
    // Qt: rotate() theo chiều kim đồng hồ (độ); theta ngược chiều kim đồng hồ.
    p.rotate(qRadiansToDegrees(-m_robotPose.theta));

    // Thân robot: hình elip nhỏ
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(220, 60, 60, 60));
    p.drawEllipse(QPointF(0, 0), 12.0, 10.0);

    // Mũi tên chỉ hướng (tam giác)
    static const QPointF kArrow[3] = {
        { 14.0,  0.0 },   // mũi
        { -7.0,  7.0 },   // sau phải
        { -7.0, -7.0 },   // sau trái
    };
    p.setPen(QPen(QColor(255, 90, 90), 1.5));
    p.setBrush(QColor(220, 60, 60, 200));
    p.drawPolygon(kArrow, 3);

    p.restore();
}