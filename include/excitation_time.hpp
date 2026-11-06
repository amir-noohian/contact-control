#pragma once

#include <cmath>

namespace excitation {
struct TimeSample {
    double phase;
    double rate;
};

// Integrate a cosine speed ramp so velocity remains the derivative of position.
// duration >= 2 * transition is required by the caller.
inline TimeSample sampleTime(double t, double duration, double transition) {
    const double pi = std::acos(-1.0);
    if (t <= 0.0) return {0.0, 0.0};
    if (t >= duration) return {duration - transition, 0.0};
    if (t < transition) {
        const double angle = pi * t / transition;
        return {0.5 * (t - transition / pi * std::sin(angle)),
                0.5 * (1.0 - std::cos(angle))};
    }
    if (t > duration - transition) {
        const double remaining = duration - t;
        const double angle = pi * remaining / transition;
        return {duration - transition - 0.5 * (remaining - transition / pi * std::sin(angle)),
                0.5 * (1.0 - std::cos(angle))};
    }
    return {t - 0.5 * transition, 1.0};
}
} // namespace excitation
