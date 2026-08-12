#include "purepursuit.h"
#include <algorithm>

void PurePursuit::setPath(const std::vector<Point2D> &path) {
    m_path = path;
    reset();
}

void PurePursuit::setParameters(double lookahead, double vMax) {
    m_lookahead = lookahead;
    m_vMax = vMax;
    // m_kp = kp;
    // m_ki = ki;
    // m_kd = kd;
}

void PurePursuit::reset() {
    m_currentIdx = 0;
    //m_integral = 0.0;
    //m_prevError = 0.0;
    m_goalReached = false;
}

TwistCmd PurePursuit::compute(const Pose2D &pose, double dt) {
    TwistCmd cmd;
    if (m_path.empty() || m_goalReached) return cmd;

    // ── 1. Advance index (Giữ nguyên) ────────────────────────────────────
    while (m_currentIdx + 1 < m_path.size() - 1) {
        double sx = m_path[m_currentIdx + 1].x - m_path[m_currentIdx].x;
        double sy = m_path[m_currentIdx + 1].y - m_path[m_currentIdx].y;
        double rx = pose.x - m_path[m_currentIdx].x;
        double ry = pose.y - m_path[m_currentIdx].y;
        if (sx * rx + sy * ry > 0.0) {
            ++m_currentIdx;
        }
        else break;
    }

    // ── 2. Goal check (Giữ nguyên) ───────────────────────────────────────
    double distToGoal = std::hypot(pose.x - m_path.back().x, pose.y - m_path.back().y);
    // Chỉ cho phép dừng khi robot thực sự đã duyệt đến những điểm cuối cùng của quỹ đạo
    if (m_currentIdx >= (m_path.size() - 8) && distToGoal < GOAL_TOLERANCE) {
        m_goalReached = true;
        return cmd;
    }

    // ── 3. Lookahead target (Giữ nguyên) ─────────────────────────────────
    Point2D robotPt{pose.x, pose.y};
    size_t lookaheadIdx = m_path.size() - 1;
    for (size_t i = m_currentIdx; i < m_path.size(); ++i) {
        if (robotPt.distanceTo(m_path[i]) >= m_lookahead) {
            lookaheadIdx = i;
            break;
        }
    }
    const Point2D &target = m_path[lookaheadIdx];

    // ── 4. Alpha (SỬA: Thêm dấu trừ để sửa lỗi Mirror) ───────────────────
    double dx = target.x - pose.x;
    double dy = target.y - pose.y;
    //double L  = std::hypot(dx, dy);

    // if (L < 1e-4) {
    //     cmd.v = m_vMax * 0.3;
    //     return cmd;
    // }
    // Đảo dấu toàn bộ kết quả normalizeAngle để robot rẽ đúng hướng quỹ đạo
    // Bỏ dấu trừ ở phía trước
    double alpha = normalizeAngle(std::atan2(dy, dx) - pose.theta);

    // ── 5. Pure Pursuit curvature (SỬA: Giới hạn L tối thiểu tránh Stall) ──
    double kappa = 2.0 * std::sin(alpha) / m_lookahead;

    // ── 6. Velocity profile (SỬA: Giảm tốc mượt khi gần đích) ─────────────
    // double kFactor = std::fabs(kappa) * m_lookahead;
    // double curvatureFactor = std::max(0.4, 1.0 - 0.4 * kFactor);
    cmd.v = m_vMax;

    // Nếu đang hướng tới điểm cuối, giảm tốc theo khoảng cách để không bị giật/khựng
    // if (lookaheadIdx == m_path.size() - 1) {
    //     cmd.v = std::min(cmd.v, distToGoal + 0.05);
    // }

    // ── 7. Omega (Giữ nguyên logic PID) ──────────────────────────────────
    cmd.omega = cmd.v * kappa;

    //if (dt > 1e-9) {
       // m_integral = std::clamp(m_integral + alpha * dt, -1.5, 1.5);
        //double deriv = (alpha - m_prevError) / dt;
        //cmd.omega += 0.05 * (m_kp * alpha + m_ki * m_integral + m_kd * deriv);
    //}
    //m_prevError = alpha;

    // ── 8. Hard clamp (Giữ nguyên) ───────────────────────────────────────
    cmd.omega = std::clamp(cmd.omega, -3.0, 3.0);

    return cmd;
}

// std::vector<Point2D> PurePursuit::generatePath(int type, double param, int steps) {
//     std::vector<Point2D> path;
//     for (int i = 0; i <= steps; ++i) {
//         double t = (2.0 * M_PI * i) / steps;
//         if (type == 1) { // Circle: x = R*sin(t), y = R - R*cos(t)
//             path.push_back({param * std::sin(t), param - param * std::cos(t)});
//         } else {         // Figure 8: y = a*sin(t), x = a*sin(t)*cos(t)
//             path.push_back({param * std::sin(t) * std::cos(t), param * std::sin(t)});
//         }
//     }
//     return path;
// }

// int PurePursuit::findLookaheadIndex(const Pose2D &pose) const {
//     Point2D robotPt{pose.x, pose.y};
//     for (size_t i = m_currentIdx; i < m_path.size(); ++i) {
//         if (robotPt.distanceTo(m_path[i]) >= m_lookahead)
//             return static_cast<int>(i);
//     }
//     return -1;
// }

// double PurePursuit::computeOmega(const Pose2D &pose, const Point2D &target, double velocity) const {
//     double dx = target.x - pose.x;
//     double dy = target.y - pose.y;
//     double L = std::hypot(dx, dy);
//     if (L < 1e-6) return 0.0;
//     double alpha = normalizeAngle(std::atan2(dy, dx) - pose.theta);
//     return velocity * (2.0 * std::sin(alpha) / L);
// }

double PurePursuit::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}