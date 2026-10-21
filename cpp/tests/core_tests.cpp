#include "icv/control.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void expect(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
void near(double actual, double expected, double tolerance, const char* message) {
    expect(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}
}

int main() {
    try {
        const icv::Vehicle vehicle;
        const icv::Route straight({{0, 0}, {20, 0}, {100, 0}});
        const auto projected = straight.project(8, 2, 0);
        near(projected.progress, 8, 1e-10, "route projection progress");
        near(projected.lateral, 2, 1e-10, "route signed lateral error");
        near(straight.at(150).x, 100, 1e-10, "route endpoint clipping");
        expect(icv::steering_control({0, 1, 0, 5}, straight, 0, vehicle) < 0, "steer back toward route");
        near(icv::speed_control({0, 0, 0, 0}, 10, 1, vehicle), 2.4, 1e-10, "forward acceleration saturation");
        near(icv::speed_control({0, 0, 0, 10}, 0, 1, vehicle, true), -4.5, 1e-10, "forward emergency brake");
        near(icv::speed_control({0, 0, 0, -3}, 0, -1, vehicle, true), 2.4, 1e-10, "reverse signed acceleration limit");
        near(icv::stopping_speed(-1), 0, 1e-10, "negative stopping distance");
        const auto stopped = icv::advance({0, 0, 0, .01}, -4.5, 0, .05, vehicle);
        near(stopped.v, 0, 1e-10, "braking must not reverse the velocity");
        const auto step = icv::advance({0, 0, 0, 2}, 1, 0, .05, vehicle);
        near(step.x, .10125, 1e-10, "constant-acceleration translation");
        near(step.v, 2.05, 1e-10, "constant-acceleration speed");
        const auto turn = icv::advance({0, 0, 0, 5}, 0, 1, .05, vehicle);
        near(turn.steer, vehicle.max_steer_rate * .05, 1e-10, "steering slew rate");
        const auto danger = icv::obstacle_speed({0, 0, 0, 10}, straight, 0, {{12, 0, 0, 4.6, 1.8, 0}}, vehicle);
        expect(danger.emergency && danger.speed_limit == 0, "obstacle emergency brake");
        const auto clear = icv::obstacle_speed({0, 0, 0, 10}, straight, 0, {{12, 5, 0, 4.6, 1.8, 0}}, vehicle);
        expect(!clear.emergency && std::isinf(clear.gap), "off-corridor obstacle exclusion");
        bool rejected = false;
        try { const icv::Route invalid({{0, 0}, {0, 0}}); } catch (const std::invalid_argument&) { rejected = true; }
        expect(rejected, "duplicate-only route rejection");
        std::cout << "PASS: native route, steering, speed, AEB, integration and actuator limits\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
