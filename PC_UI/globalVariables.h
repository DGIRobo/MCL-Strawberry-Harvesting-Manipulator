#ifndef GLOBALVARIABLES_H
#define GLOBALVARIABLES_H

#pragma once
#include <array>
#include <QReadWriteLock>

struct MotorState {
    int    id = 0;
    int    mode = 0;
    double control = 0.0;
};

struct Telemetry {
    // Transmitting Data
    std::array<double,4> target_position{};
    std::array<double,5> posx_pid_gain{};
    std::array<double,5> posy_pid_gain{};
    std::array<double,5> posz_pid_gain{};

    // Receiving Data
    double t = 0.0;         // time_sec
    double dt = 0.0;        // dt_sec
    int    robot_mode = 0;

    MotorState motors[3];   // [0..2]

    std::array<double,3> q_bi{};
    std::array<double,3> pos_ref{};
    std::array<double,3> pos{};
    std::array<double,3> vel{};
    std::array<double,3> pos_I{};
    std::array<double,3> pos_pid{};
};

extern Telemetry      gTelemetry;
extern QReadWriteLock gTelemetryLock;

#endif // GLOBALVARIABLES_H
