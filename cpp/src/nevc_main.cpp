#include "icv/control.hpp"

#include <SimOnePNCAPI.h>
#include <SimOneSensorAPI.h>
#include <SimOneServiceAPI.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Json = nlohmann::json;
std::string value(int argc, char** argv, const std::string& option, const std::string& fallback = {}) {
    for (int i = 1; i + 1 < argc; ++i) if (argv[i] == option) return argv[i + 1];
    return fallback;
}
bool flag(int argc, char** argv, const std::string& option) {
    for (int i = 1; i < argc; ++i) if (argv[i] == option) return true;
    return false;
}
std::vector<icv::Point> route_from(const Json& config) {
    std::vector<icv::Point> points;
    for (const auto& p : config.at("route_xy")) {
        if (!p.is_array() || p.size() < 2) throw std::runtime_error("Each route_xy entry must contain [x, y]");
        points.push_back({p[0].get<double>(), p[1].get<double>()});
    }
    return points;
}
void send(const char* id, const icv::Vehicle& vehicle, const Json& config, double acceleration,
          double steering, double speed, int indicator, int direction) {
    SimOne_Data_Control control{};
    control.throttleMode = ESimOne_Throttle_Mode_Percent;
    control.brakeMode = ESimOne_Brake_Mode_Percent;
    control.steeringMode = ESimOne_Steering_Mode_Percent;
    control.throttle = static_cast<float>(std::clamp(acceleration / vehicle.accel, 0.0, .5));
    control.brake = static_cast<float>(std::clamp(-acceleration / vehicle.brake, 0.0, 1.0));
    // SimOne's percent mode documents left as negative; Pure Pursuit positive delta is left.
    control.steering = static_cast<float>(std::clamp(-steering * config.at("steering_radians_to_percent").get<double>(), -1.0, 1.0));
    control.gear = direction < 0 ? ESimOne_Gear_Mode_Reverse : ESimOne_Gear_Mode_Drive;
    if (std::abs(speed) < .05 && acceleration <= 0) { control.throttle = 0; control.brake = .35f; }
    if (!SimOneAPI::SetDrive(id, &control)) throw std::runtime_error("SimOne SetDrive failed");
    SimOne_Data_Signal_Lights signal{};
    if (indicator == 1) signal.signalLights = ESimOne_Signal_Light_RightBlinker;
    else if (indicator == 2) signal.signalLights = ESimOne_Signal_Light_LeftBlinker;
    else if (direction < 0) signal.signalLights = ESimOne_Signal_Light_BackDrive;
    if (!SimOneAPI::SetSignalLights(id, &signal)) throw std::runtime_error("SimOne SetSignalLights failed");
}
}

int main(int argc, char** argv) {
    const char* id = nullptr;
    bool initialized = false;
    try {
        if (flag(argc, argv, "--probe")) {
            std::cout << "SimOne SDK version: " << SimOneAPI::GetVersion() << '\n';
            return 0;
        }
        const auto config_file = value(argc, argv, "--config");
        if (config_file.empty()) throw std::runtime_error("Provide a measured --config JSON before connecting to NEVC");
        const auto config = Json::parse(std::ifstream(config_file));
        if (!config.value("calibrated_for_nevc", false))
            throw std::runtime_error("Configuration is not calibrated_for_nevc; real control refused");
        if (config.value("steering_radians_to_percent", 0.0) == 0.0)
            throw std::runtime_error("Calibrate steering_radians_to_percent before connecting");
        if (config.value("obstacle_source", std::string()) != "sensor" &&
            !(config.value("obstacle_source", std::string()) == "ground_truth" &&
              config.value("ground_truth_permitted_by_rules", false) && flag(argc, argv, "--allow-ground-truth")))
            throw std::runtime_error("Use a sensor or obtain explicit rules permission and pass --allow-ground-truth");
        const auto sensor_id = config.value("sensor_id", std::string());
        if (config.at("obstacle_source") == "sensor" && (sensor_id.empty() || sensor_id.rfind("REPLACE_", 0) == 0))
            throw std::runtime_error("Set sensor_id to the actual, measured NEVC sensor ID");

        const std::string vehicle_id = value(argc, argv, "--vehicle", "0");
        id = vehicle_id.c_str();
        const auto host = value(argc, argv, "--host", "127.0.0.1");
        const auto port = std::stoi(value(argc, argv, "--port", "23789"));
        if (!SimOneAPI::InitSimOneAPI(id, true, host.c_str(), port))
            throw std::runtime_error("SimOne initialization failed; check that the competition simulator is running");
        initialized = true;
        SimOneAPI::SetDriverName(id, "ICV_SIM_CPP");
        if (SimOneAPI::GetVersion() == nullptr) throw std::runtime_error("SimOne SDK version query failed");
        const icv::Vehicle vehicle;
        const auto route_points = route_from(config);
        const icv::Route route(route_points);
        const auto speed = config.value("cruise_mps", 6.0);
        const auto max_error = config.value("max_route_error_m", 1.2);
        double progress = 0;
        int previous_frame = -1;
        SimOne_Data_Gps gps{};
        SimOne_Data_Control stop{};
        std::cout << "Connected to SimOne SDK " << SimOneAPI::GetVersion()
                  << ", vehicle " << vehicle_id << ", waypoints " << route_points.size() << '\n';
        while (true) {
            const int frame = SimOneAPI::Wait();
            if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status_Stop) break;
            if (frame == previous_frame) continue;
            previous_frame = frame;
            if (!SimOneAPI::GetGps(id, &gps) || gps.isGPSLost)
                throw std::runtime_error("GPS unavailable/lost; stopping this control loop");
            const double velocity = std::hypot(gps.velX, gps.velY);
            const double direction_sign = std::cos(gps.oriZ) * gps.velX + std::sin(gps.oriZ) * gps.velY >= 0 ? 1 : -1;
            icv::State state{gps.posX, gps.posY, gps.oriZ, velocity * direction_sign,
                             gps.steering * icv::pi / 180.0, 0};
            const auto projection = route.project(state.x, state.y, progress);
            progress = projection.progress;
            if (std::abs(projection.lateral) > max_error)
                throw std::runtime_error("Route mismatch exceeds max_route_error_m; stopping");

            std::vector<icv::Obstacle> obstacles;
            if (config.at("obstacle_source") == "sensor") {
                SimOne_Data_SensorDetections detections{};
                if (!SimOneAPI::GetSensorDetections(id, sensor_id.c_str(), &detections))
                    throw std::runtime_error("Configured sensor detections are unavailable");
                for (int i = 0; i < std::clamp(detections.objectSize, 0, SOSM_SENSOR_DETECTIONS_OBJECT_SIZE_MAX); ++i) {
                    const auto& o = detections.objects[i];
                    obstacles.push_back({o.posX, o.posY, o.oriZ, o.length, o.width, std::hypot(o.velX, o.velY)});
                }
            } else {
                SimOne_Data_Obstacle truth{};
                if (!SimOneAPI::GetGroundTruth(id, &truth))
                    throw std::runtime_error("Rules-authorized ground truth is unavailable");
                for (int i = 0; i < std::clamp(truth.obstacleSize, 0, SOSM_OBSTACLE_SIZE_MAX); ++i) {
                    const auto& o = truth.obstacle[i];
                    obstacles.push_back({o.posX, o.posY, o.oriZ, o.length, o.width, std::hypot(o.velX, o.velY)});
                }
            }

            double target = std::min(speed, icv::stopping_speed(route.length() - progress, 1.3));
            for (const auto& zone : config.value("speed_zones", Json::array())) {
                const double begin = zone[0].get<double>(), end = zone[1].get<double>();
                if (begin <= progress && progress < end) target = std::min(target, .95 * zone[2].get<double>());
            }
            for (const auto& light : config.value("lights", Json::array())) {
                const double gap = light.at("stop_s").get<double>() - progress - vehicle.front;
                if (gap < 0) continue;
                SimOne_Data_TrafficLight lights{};
                const int light_id = light.at("opendrive_id").get<int>();
                if (!SimOneAPI::GetTrafficLight(id, light_id, &lights))
                    throw std::runtime_error("Configured traffic-light ID is unavailable");
                if (lights.status != ESimOne_TrafficLight_Status_Green)
                    target = std::min(target, icv::stopping_speed(gap - .3));
            }
            const auto decision = icv::obstacle_speed(state, route, progress, obstacles, vehicle);
            target = std::min(target, decision.speed_limit);
            if (config.find("pedestrian") != config.end() && config["pedestrian"].is_object()) {
                for (const auto& o : obstacles) {
                    const auto ahead = route.project(o[0], o[1], progress).progress - progress - vehicle.front;
                    if (ahead > 0 && ahead < 25 && std::abs(route.project(o[0], o[1], progress).lateral) < 2.0)
                        target = std::min(target, icv::stopping_speed(ahead - 2.0));
                }
            }
            int indicator = 0;
            for (const auto& zone : config.value("indicator_zones", Json::array()))
                if (zone[0].get<double>() <= progress && progress < zone[1].get<double>())
                    indicator = zone[2].get<int>();
            const auto acceleration = icv::speed_control(state, target, 1, vehicle, decision.emergency);
            const auto steering = icv::steering_control(state, route, progress, vehicle);
            send(id, vehicle, config, acceleration, steering, velocity, indicator, 1);
            if (frame % 100 == 0)
                std::cout << "frame=" << frame << " progress=" << progress << " speed_mps=" << velocity
                          << " cross_track_m=" << projection.lateral << '\n';
            if (route.length() - progress < .1 && velocity < .05) break;
            SimOneAPI::NextFrame(frame);
        }
        send(id, vehicle, config, -vehicle.brake, 0, 0, 0, 1);
        std::cout << "SimOne case ended; controller stopped.\n";
        return 0;
    } catch (const std::exception& error) {
        if (initialized && id) {
            try {
                const icv::Vehicle vehicle;
                Json safe_config{{"steering_radians_to_percent", 1.0}};
                send(id, vehicle, safe_config, -vehicle.brake, 0, 0, 0, 1);
            } catch (...) {}
            SimOneAPI::TerminateSimOneAPI();
        }
        std::cerr << "ICV NEVC controller error: " << error.what() << '\n';
        return 1;
    }
}
