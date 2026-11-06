#include "excitation_time.hpp"
#include <cassert>
#include <cmath>

int main() {
    const double duration = 60.0;
    const double dt = 0.001;
    double integrated = 0.0;
    auto previous = excitation::sampleTime(0.0, duration, 2.0);
    assert(previous.phase == 0.0 && previous.rate == 0.0);
    for (int i = 1; i <= 60000; ++i) {
        const double t = i * dt;
        const auto current = excitation::sampleTime(t, duration, 2.0);
        assert(current.phase >= previous.phase);
        assert(current.rate >= 0.0 && current.rate <= 1.0);
        integrated += dt * 0.5 * (previous.rate + current.rate);
        assert(std::abs(integrated - current.phase) < 1e-6);
        // Check the actual chain rule for a representative Fourier component.
        const double h = 1e-5;
        const double numericalVelocity =
            (std::sin(excitation::sampleTime(t + h, duration, 2.0).phase) -
             std::sin(excitation::sampleTime(t - h, duration, 2.0).phase)) / (2.0 * h);
        assert(std::abs(numericalVelocity - std::cos(current.phase) * current.rate) < 1e-6);
        previous = current;
    }
    assert(previous.rate == 0.0);
    assert(excitation::sampleTime(61.0, duration, 2.0).phase == previous.phase);
}
