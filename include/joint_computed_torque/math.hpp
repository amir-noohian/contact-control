#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>

namespace joint_computed_torque {
constexpr double duration = 30.0;
constexpr double rampDuration = 3.0;
constexpr double amplitude = 0.4; // radians, joints 2 and 4
constexpr double frequency = 0.2; // Hz
constexpr double torqueLimit[7] = {50, 40, 30, 30, 5, 5, 5};
constexpr double lower[7] = {-2.4, -1.8, -2.5, -0.8, -0.5, -0.5, -0.5};
constexpr double upper[7] = {2.4, 1.8, 2.5, 3.0, 0.5, 0.5, 0.5};
constexpr double accelerationKp[4] = {
    100.0 * 750.0 / 1000.0, 100.0, 100.0 * 400.0 / 1000.0, 100.0 * 200.0 / 1000.0
}; // 1/s^2; SLAX joint_position_control kp ratios
constexpr double accelerationKd[4] = {
    3.3, 3.3 * 8.0 / 8.3, 3.3 * 3.3 / 8.3, 3.3 * 0.8 / 8.3
}; // 1/s; SLAX joint_position_control kd ratios

inline double clamp(double x, double limit) {
    return std::max(-limit, std::min(limit, x));
}

inline double accelerationPD(double desiredPosition, double measuredPosition,
                             double desiredVelocity, double measuredVelocity,
                             size_t joint) {
    return accelerationKp[joint] * (desiredPosition - measuredPosition) +
           accelerationKd[joint] * (desiredVelocity - measuredVelocity);
}

// Quintic envelope and analytic derivatives: zero q, dq, ddq at both ends.
inline void sine(double t, double& q, double& dq, double& ddq) {
    q = dq = ddq = 0.0;
    if (t <= 0 || t >= duration) return;
    double a = 1, da = 0, dda = 0;
    if (t < rampDuration || t > duration - rampDuration) {
        const bool ending = t > duration - rampDuration;
        const double u = (ending ? duration - t : t) / rampDuration;
        a = 10*u*u*u - 15*u*u*u*u + 6*u*u*u*u*u;
        da = (30*u*u - 60*u*u*u + 30*u*u*u*u) / rampDuration;
        if (ending) da = -da;
        dda = (60*u - 180*u*u + 120*u*u*u) / (rampDuration*rampDuration);
    }
    const double w = 2*std::acos(-1.0)*frequency;
    const double s = std::sin(w*t), c = std::cos(w*t);
    q = amplitude*a*s;
    dq = amplitude*(da*s + a*w*c);
    ddq = amplitude*(dda*s + 2*da*w*c - a*w*w*s);
}

} // namespace joint_computed_torque
