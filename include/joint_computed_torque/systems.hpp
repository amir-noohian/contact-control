#pragma once
#include "joint_computed_torque/math.hpp"
#include <atomic>
#include <barrett/systems.h>
#include <barrett/units.h>

namespace joint_computed_torque {
template<size_t DOF>
class SineReference : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
public:
    Input<double> timeInput;
    Output<jp_type> position;
    Output<jv_type> velocity;
    Output<ja_type> acceleration;
    std::atomic<bool> done{false};
    SineReference() : timeInput(this), position(this, &qp), velocity(this, &vp),
                      acceleration(this, &ap) {}
    ~SineReference() { this->mandatoryCleanUp(); }
private:
    typename Output<jp_type>::Value* qp;
    typename Output<jv_type>::Value* vp;
    typename Output<ja_type>::Value* ap;
    jp_type q;
    jv_type dq;
    ja_type ddq;
    void operate() override {
        const double t = timeInput.getValue();
        double p, v, a;
        sine(t, p, v, a);
        q.setZero(); dq.setZero(); ddq.setZero();
        for (size_t j = 0; j < DOF; ++j) {
            if (j == 1 || j == 3) { q[j] = p; dq[j] = v; ddq[j] = a; }
        }
        qp->setData(&q); vp->setData(&dq); ap->setData(&ddq);
        done.store(t >= duration);
    }
};

template<size_t DOF>
class DynamicsFeedforward : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
public:
    Input<jp_type> desiredPosition, measuredPosition;
    Input<jv_type> desiredVelocity, measuredVelocity;
    Input<jt_type> feedforward, gravity;
    Output<jt_type> output;
    std::atomic<bool> fault{false};
    DynamicsFeedforward()
        : desiredPosition(this), measuredPosition(this), desiredVelocity(this),
          measuredVelocity(this), feedforward(this), gravity(this), output(this, &value) {}
    ~DynamicsFeedforward() { this->mandatoryCleanUp(); }
private:
    typename Output<jt_type>::Value* value;
    jt_type command;
    void operate() override {
        const auto& qd = desiredPosition.getValue();
        const auto& q = measuredPosition.getValue();
        const auto& vd = desiredVelocity.getValue();
        const auto& v = measuredVelocity.getValue();
        const auto& ff = feedforward.getValue();
        const auto& g = gravity.getValue();
        // Check every input before producing any torque. Latch faults until restart.
        for (size_t j = 0; j < DOF; ++j) {
            if (!std::isfinite(q[j]) || !std::isfinite(v[j]) ||
                !std::isfinite(qd[j]) || !std::isfinite(vd[j]) ||
                !std::isfinite(ff[j]) || !std::isfinite(g[j]) ||
                q[j] < lower[j] || q[j] > upper[j] ||
                std::abs(qd[j]-q[j]) > 0.35 || std::abs(v[j]) > 1.5) fault.store(true);
        }
        command.setZero();
        if (!fault.load()) {
            for (size_t j = 0; j < DOF; ++j) {
                // Built-in gravity remains enabled, including on the wrist.
                // Replace its first-four-joint contribution with model feedforward.
                if (j < 4) command[j] = clamp(ff[j], torqueLimit[j]) - g[j];
            }
        }
        // On fault return to built-in gravity only while main stops position tracking.
        value->setData(&command);
    }
};

template<size_t DOF>
class AccelerationPD : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
public:
    Input<jp_type> desiredPosition, measuredPosition;
    Input<jv_type> desiredVelocity, measuredVelocity;
    Input<ja_type> desiredAcceleration;
    Output<ja_type> acceleration;
    std::atomic<bool> fault{false};
    AccelerationPD() : desiredPosition(this), measuredPosition(this),
                       desiredVelocity(this), measuredVelocity(this),
                       desiredAcceleration(this), acceleration(this, &value) {}
    ~AccelerationPD() { this->mandatoryCleanUp(); }
private:
    typename Output<ja_type>::Value* value;
    ja_type command;
    void operate() override {
        const auto& qd = desiredPosition.getValue();
        const auto& q = measuredPosition.getValue();
        const auto& vd = desiredVelocity.getValue();
        const auto& v = measuredVelocity.getValue();
        const auto& desiredAcc = desiredAcceleration.getValue();
        command = desiredAcc;
        for (size_t j = 0; j < DOF; ++j) {
            if (!std::isfinite(qd[j]) || !std::isfinite(q[j]) ||
                !std::isfinite(vd[j]) || !std::isfinite(v[j]) ||
                !std::isfinite(desiredAcc[j])) {
                fault.store(true);
            }
        }
        if (!fault.load()) {
            for (size_t j = 0; j < DOF && j < 4; ++j) {
                command[j] += accelerationPD(qd[j], q[j], vd[j], v[j], j);
            }
        } else {
            command.setZero();
        }
        value->setData(&command);
    }
};
} // namespace joint_computed_torque
