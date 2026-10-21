#pragma once

#include <array>
#include <limits>
#include <vector>

namespace icv {
constexpr double pi = 3.14159265358979323846;

struct Vehicle {
    double wheelbase = 2.8;
    double front = 3.6;
    double rear = 1.0;
    double width = 1.8;
    double max_steer = 32.0 * pi / 180.0;
    double max_steer_rate = 80.0 * pi / 180.0;
    double accel = 2.4;
    double brake = 4.5;
};

struct State {
    double x = 0;
    double y = 0;
    double yaw = 0;
    double v = 0;
    double steer = 0;
    double a = 0;
};

struct Point { double x; double y; };
struct Projection { double progress; double lateral; double tangent; };

class Route {
public:
    explicit Route(std::vector<Point> points, int direction = 1);
    Point at(double progress) const;
    Projection project(double x, double y, double hint = 0) const;
    double length() const { return distance_.back(); }
    int direction() const { return direction_; }
    const Point& end() const { return points_.back(); }

private:
    std::vector<Point> points_;
    std::vector<double> distance_;
    int direction_;
};

using Obstacle = std::array<double, 6>;  // x, y, yaw, length, width, speed (SI)
struct ObstacleDecision {
    double speed_limit = std::numeric_limits<double>::infinity();
    bool emergency = false;
    double gap = std::numeric_limits<double>::infinity();
};

double wrap(double angle);
double steering_control(const State&, const Route&, double progress, const Vehicle&);
double speed_control(const State&, double desired_speed, int direction, const Vehicle&, bool emergency = false);
double stopping_speed(double distance, double deceleration = 1.8);
ObstacleDecision obstacle_speed(const State&, const Route&, double progress,
                                const std::vector<Obstacle>&, const Vehicle&);
State advance(const State&, double acceleration, double steering, double dt, const Vehicle&);
}  // namespace icv
