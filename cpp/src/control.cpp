#include "icv/control.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace icv {
double wrap(double angle) {
    double value = std::fmod(angle + pi, 2 * pi);
    if (value < 0) value += 2 * pi;
    return value - pi;
}

Route::Route(std::vector<Point> points, int direction) : direction_(direction) {
    if (direction != 1 && direction != -1) throw std::invalid_argument("Route direction must be +1 or -1");
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!std::isfinite(points[i].x) || !std::isfinite(points[i].y))
            throw std::invalid_argument("Route coordinates must be finite");
        if (i == 0 || std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y) > 1e-6)
            points_.push_back(points[i]);
    }
    if (points_.size() < 2) throw std::invalid_argument("A route needs at least two distinct points");
    distance_.push_back(0);
    for (std::size_t i = 1; i < points_.size(); ++i)
        distance_.push_back(distance_.back() + std::hypot(points_[i].x - points_[i - 1].x,
                                                        points_[i].y - points_[i - 1].y));
}

Point Route::at(double progress) const {
    progress = std::clamp(progress, 0.0, length());
    if (progress >= length()) return points_.back();
    auto i = static_cast<std::size_t>(std::upper_bound(distance_.begin(), distance_.end(), progress) - distance_.begin() - 1);
    const double fraction = (progress - distance_[i]) / (distance_[i + 1] - distance_[i]);
    return {points_[i].x + fraction * (points_[i + 1].x - points_[i].x),
            points_[i].y + fraction * (points_[i + 1].y - points_[i].y)};
}

Projection Route::project(double x, double y, double hint) const {
    const auto lo_index = std::lower_bound(distance_.begin(), distance_.end(), hint - 5) - distance_.begin();
    const auto hi_index = std::lower_bound(distance_.begin(), distance_.end(), hint + 45) - distance_.begin();
    const auto lo = static_cast<std::size_t>(std::max<std::ptrdiff_t>(0, lo_index - 1));
    const auto hi = std::min(points_.size() - 1, static_cast<std::size_t>(hi_index + 1));
    double best = std::numeric_limits<double>::infinity();
    Projection result{};
    for (std::size_t i = lo; i < hi; ++i) {
        const double dx = points_[i + 1].x - points_[i].x;
        const double dy = points_[i + 1].y - points_[i].y;
        const double segment_length = distance_[i + 1] - distance_[i];
        const double fraction = std::clamp(((x - points_[i].x) * dx + (y - points_[i].y) * dy) /
                                          std::max(dx * dx + dy * dy, 1e-12), 0.0, 1.0);
        const double ex = x - points_[i].x - fraction * dx;
        const double ey = y - points_[i].y - fraction * dy;
        const double squared_error = ex * ex + ey * ey;
        if (squared_error < best) {
            best = squared_error;
            result = {distance_[i] + fraction * segment_length,
                      (dx * ey - dy * ex) / segment_length, std::atan2(dy, dx)};
        }
    }
    return result;
}

double steering_control(const State& state, const Route& route, double progress, const Vehicle& vehicle) {
    double lookahead = std::min(8.0, .65 + std::abs(state.v) * .65);
    if (route.direction() < 0) lookahead = std::min(1.5, .6 + std::abs(state.v) * .45);
    const auto target = route.at(progress + lookahead);
    const double alpha = wrap(std::atan2(target.y - state.y, target.x - state.x) - state.yaw -
                              (route.direction() < 0 ? pi : 0));
    const double distance = std::max(.3, std::hypot(target.x - state.x, target.y - state.y));
    const double steering = std::atan2(route.direction() * 2 * vehicle.wheelbase * std::sin(alpha), distance);
    return std::clamp(steering, -vehicle.max_steer, vehicle.max_steer);
}

double speed_control(const State& state, double desired, int direction, const Vehicle& vehicle, bool emergency) {
    double effort = 2.5 * (desired * direction - state.v);
    if (emergency) effort = std::abs(state.v) > .01 ? -std::copysign(vehicle.brake, state.v) : 0;
    // Match the Python model's signed, asymmetric acceleration bounds.
    return std::clamp(effort, -vehicle.brake, vehicle.accel);
}

double stopping_speed(double distance, double deceleration) {
    const double d = std::max(0.0, distance);
    return std::min(std::sqrt(2 * std::min(deceleration, .9) * d), .85 * d);
}

ObstacleDecision obstacle_speed(const State& state, const Route& route, double progress,
                                const std::vector<Obstacle>& obstacles, const Vehicle& vehicle) {
    ObstacleDecision result;
    for (const auto& o : obstacles) {
        const auto projection = route.project(o[0], o[1], progress);
        if (projection.progress >= route.length() - .01) {
            const auto& end = route.end();
            if ((o[0] - end.x) * std::cos(projection.tangent) +
                (o[1] - end.y) * std::sin(projection.tangent) > o[3] / 2 + vehicle.front) continue;
        }
        const double ahead = projection.progress - progress;
        if (ahead <= 0 || std::abs(projection.lateral) > vehicle.width / 2 + o[4] / 2 + .25 || ahead > 90) continue;
        const double clearance = ahead - vehicle.front - o[3] / 2;
        result.gap = std::min(result.gap, clearance);
        const double closing = std::max(0.0, std::abs(state.v) - o[5]);
        const double safe = 2.5 + std::abs(state.v) * 1.3;
        result.speed_limit = std::min(result.speed_limit, std::max(0.0, o[5] + (clearance - safe) * .65));
        if (clearance < std::max(1.0, closing * .35 + closing * closing / (2 * vehicle.brake) + .6))
            result.emergency = true;
    }
    return result;
}

State advance(const State& state, double acceleration, double steering, double dt, const Vehicle& vehicle) {
    if (!std::isfinite(dt) || dt <= 0 || dt > 1 || vehicle.wheelbase <= 0)
        throw std::invalid_argument("Integration requires 0 < dt <= 1 and a positive wheelbase");
    double a = std::clamp(acceleration, -vehicle.brake, vehicle.accel);
    double delta = std::clamp(steering, -vehicle.max_steer, vehicle.max_steer);
    delta = std::clamp(delta, state.steer - vehicle.max_steer_rate * dt, state.steer + vehicle.max_steer_rate * dt);
    if (state.v * (state.v + a * dt) < 0) a = -state.v / dt;
    using Vector = std::array<double, 4>;
    const auto derivative = [&](const Vector& z) -> Vector {
        return {z[3] * std::cos(z[2]), z[3] * std::sin(z[2]), z[3] * std::tan(delta) / vehicle.wheelbase, a};
    };
    const auto shifted = [](const Vector& z, const Vector& k, double factor) {
        Vector out{};
        for (std::size_t i = 0; i < z.size(); ++i) out[i] = z[i] + factor * k[i];
        return out;
    };
    Vector z{state.x, state.y, state.yaw, state.v};
    const int steps = static_cast<int>(std::ceil(dt / .01));
    const double h = dt / steps;
    for (int step = 0; step < steps; ++step) {
        const auto k1 = derivative(z);
        const auto k2 = derivative(shifted(z, k1, h / 2));
        const auto k3 = derivative(shifted(z, k2, h / 2));
        const auto k4 = derivative(shifted(z, k3, h));
        for (std::size_t i = 0; i < z.size(); ++i) z[i] += h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
    if (std::abs(z[3]) < 1e-7) z[3] = 0;
    return {z[0], z[1], wrap(z[2]), z[3], delta, a};
}
}  // namespace icv
