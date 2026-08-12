#pragma once

#include <vector>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

struct TwistCmd {
    double v{0.0};
    double omega{0.0};
};

struct Point2D {
    double x{0.0};
    double y{0.0};

    double distanceTo(const Point2D &other) const {
        return std::hypot(other.x - x, other.y - y);
    }
};

class PurePursuit {
public:
    PurePursuit() = default;

    void setPath(const std::vector<Point2D> &path);
    void setParameters(double lookahead, double vMax);
    void reset();

    bool   isPathLoaded()  const { return !m_path.empty(); }
    size_t currentIndex()  const { return m_currentIdx; }
    bool   isGoalReached() const { return m_goalReached; }
    const  std::vector<Point2D>& path() const { return m_path; }

    TwistCmd compute(const Pose2D &pose, double dt);

    // Helper: 1 - Circle, 2 - Figure 8
    //static std::vector<Point2D> generatePath(int type, double param, int steps = 200);

private:
    //int findLookaheadIndex(const Pose2D &pose) const;
    //double computeOmega(const Pose2D &pose, const Point2D &target, double velocity) const;
    static double normalizeAngle(double angle);

    std::vector<Point2D> m_path;
    double m_lookahead{0.3};
    double m_vMax{0.5};
    // double m_kp{1.0};
    // double m_ki{0.01};
    // double m_kd{0.05};

    //double m_integral{0.0};
    //double m_prevError{0.0};
    size_t m_currentIdx{0};
    bool m_goalReached{false};

    static constexpr double GOAL_TOLERANCE = 0.1;
};