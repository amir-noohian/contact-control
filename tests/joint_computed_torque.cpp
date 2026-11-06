#include "joint_computed_torque/math.hpp"
#include "dynamics/dynamics.h"
#include <cassert>
#include <iostream>

int main() {
    using namespace joint_computed_torque;
    for (double t : {-1., 0., duration, duration+1}) {
        double q,v,a; sine(t,q,v,a);
        assert(q == 0 && v == 0 && a == 0);
    }
    // Verify analytic velocity and acceleration, including both envelope ramps.
    for (double t : {0.1, 1.5, 2.9, 3.0, 7.2, 27.0, 28.5, 29.9}) {
        double q,v,a,qm,vm,am,qp,vp,ap;
        // At ramp joins jerk is discontinuous; use a small finite-difference step.
        const double h=1e-6;
        sine(t,q,v,a); sine(t-h,qm,vm,am); sine(t+h,qp,vp,ap);
        assert(std::abs(v-(qp-qm)/(2*h)) < 1e-7);
        assert(std::abs(a-(vp-vm)/(2*h)) < 1e-6);
        assert(std::abs(q) <= amplitude);
    }
    const double ff=12, gravityTorque=9, pid=3;
    const double correction=clamp(ff, torqueLimit[0])-gravityTorque;
    assert(correction+gravityTorque+pid==ff+pid);
    assert(clamp(100, torqueLimit[0])==50);
    assert(clamp(-100, torqueLimit[0])==-50);
    assert(std::abs(accelerationKp[0] / accelerationKp[1] - 0.75) < 1e-12);
    assert(std::abs(accelerationKp[2] / accelerationKp[1] - 0.4) < 1e-12);
    assert(std::abs(accelerationKp[3] / accelerationKp[1] - 0.2) < 1e-12);
    assert(std::abs(accelerationKd[1] / accelerationKd[0] - 8.0 / 8.3) < 1e-12);
    assert(std::abs(accelerationKd[2] / accelerationKd[0] - 3.3 / 8.3) < 1e-12);
    assert(std::abs(accelerationKd[3] / accelerationKd[0] - 0.8 / 8.3) < 1e-12);
    assert(std::abs(accelerationKp[0] - 75.0) < 1e-12);
    assert(std::abs(accelerationKp[1] - 100.0) < 1e-12);
    assert(std::abs(accelerationKp[2] - 40.0) < 1e-12);
    assert(std::abs(accelerationKp[3] - 20.0) < 1e-12);
    assert(std::abs(accelerationPD(0.2, 0.1, 0.4, 0.1, 0) - 8.49) < 1e-12);
    Eigen::Vector4d q,v,a;
    q << .1,.2,.1,.3; v.setZero(); a.setZero();
    const auto beta=initialize_beta();
    Eigen::Vector4d gravity=calculate_W_eigen(q,v,a)*beta;
    a << 0,.2,0,-.3;
    Eigen::Vector4d full=calculate_W_eigen(q,v,a)*beta;
    assert(gravity.allFinite() && full.allFinite());
    assert((full-gravity).norm()>1e-6); // Desired acceleration affects torque.
    std::cout << "Trajectory derivatives, feedforward limiting, gravity accounting and acceleration checks passed.\n";
}
