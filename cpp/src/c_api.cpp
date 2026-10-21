#include "icv/c_api.hpp"
#include <exception>
#include <stdexcept>
#include <string>

namespace {
thread_local std::string last_error;
template <typename Function> int guard(Function function) noexcept {
    try { function(); last_error.clear(); return 1; }
    catch (const std::exception& error) { last_error = error.what(); }
    catch (...) { last_error = "Unknown native control failure"; }
    return 0;
}
void require(bool condition) { if (!condition) throw std::invalid_argument("Invalid native API argument"); }
}

extern "C" {
int icv_abi_version() { return 1; }
const char* icv_last_error() { return last_error.c_str(); }
void* icv_route_create(const double* xy, std::size_t count, int direction) {
    icv::Route* result = nullptr;
    guard([&] {
        require(xy != nullptr && count >= 2);
        std::vector<icv::Point> points;
        points.reserve(count);
        for (std::size_t i = 0; i < count; ++i) points.push_back({xy[2 * i], xy[2 * i + 1]});
        result = new icv::Route(std::move(points), direction);
    });
    return result;
}
void icv_route_destroy(void* route) { delete static_cast<icv::Route*>(route); }
int icv_steering(const icv::State* state, const void* route, double progress, const icv::Vehicle* vehicle, double* output) {
    return guard([&] { require(state && route && vehicle && output);
        *output = icv::steering_control(*state, *static_cast<const icv::Route*>(route), progress, *vehicle); });
}
int icv_speed(const icv::State* state, double desired, int direction, const icv::Vehicle* vehicle, int emergency, double* output) {
    return guard([&] { require(state && vehicle && output && (direction == 1 || direction == -1));
        *output = icv::speed_control(*state, desired, direction, *vehicle, emergency != 0); });
}
int icv_obstacles(const icv::State* state, const void* route, double progress, const double* objects,
                  std::size_t count, const icv::Vehicle* vehicle, double* limit, int* emergency, double* gap) {
    return guard([&] {
        require(state && route && vehicle && limit && emergency && gap && (count == 0 || objects));
        std::vector<icv::Obstacle> obstacles(count);
        for (std::size_t i = 0; i < count; ++i)
            for (std::size_t j = 0; j < 6; ++j) obstacles[i][j] = objects[6 * i + j];
        const auto result = icv::obstacle_speed(*state, *static_cast<const icv::Route*>(route), progress, obstacles, *vehicle);
        *limit = result.speed_limit; *emergency = result.emergency ? 1 : 0; *gap = result.gap;
    });
}
int icv_advance(const icv::State* state, double a, double steer, double dt, const icv::Vehicle* vehicle, icv::State* output) {
    return guard([&] { require(state && vehicle && output); *output = icv::advance(*state, a, steer, dt, *vehicle); });
}
}
