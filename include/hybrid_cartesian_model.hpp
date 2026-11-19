#pragma once

#include "hybrid_fp_ft/position_reference.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <barrett/cdlbt/kinematics.h>
#include <barrett/math/kinematics.h>

namespace hybrid_cartesian_dynamics {

constexpr std::array<int, 2> activeJoints{{1, 3}};

template <typename JointTorque>
inline JointTorque cancelTaskJointGravity(const JointTorque& taskAndHoldTorque,
                                           const JointTorque& barrettGravity) {
    JointTorque command = taskAndHoldTorque;
    // The Cartesian inverse dynamics already includes gravity on the task joints.
    for (int joint : activeJoints) {
        command[joint] -= barrettGravity[joint];
    }
    return command;
}

inline double referenceAcceleration(double time, double duration, double distance) {
    const double s = std::max(0.0, std::min(1.0, time / duration));
    return distance * 60.0 * s * (1.0 - s) * (1.0 - 2.0 * s) / (duration * duration);
}

struct ContactReference {
    double xOffset;
    double xVelocity;
    double xAcceleration;
    double zForce;
    bool finalHold;
};

inline double sineReferenceAcceleration(double time, double rampDuration, double period,
                                        double amplitude, double originOffset) {
    double blend, blendRate;
    hybrid_fp_ft::positionProfile(time, rampDuration, blend, blendRate);
    const double blendAcceleration = referenceAcceleration(time, rampDuration, 1.0);
    const double omega = 2.0 * std::acos(-1.0) / period;
    const double phase = omega * time;
    return blendAcceleration * (originOffset + amplitude * std::sin(phase))
         + 2.0 * blendRate * amplitude * omega * std::cos(phase)
         - blend * amplitude * omega * omega * std::sin(phase);
}

inline ContactReference contactReference(double time, double rampDuration, double motionDuration,
                                         double xDistance, double zForceTarget,
                                         bool sine = false, double amplitude = 0.0,
                                         double period = 1.0) {
    double forceFraction, forceRate;
    hybrid_fp_ft::positionProfile(time, rampDuration, forceFraction, forceRate);

    if (sine) {
        double xOffset, xVelocity;
        hybrid_fp_ft::sinePositionProfile(time, motionDuration, period, amplitude,
                                          0.0, xDistance, xOffset, xVelocity);
        const double xAcceleration = sineReferenceAcceleration(
            time, motionDuration, period, amplitude, xDistance);
        return {xOffset, xVelocity, xAcceleration,
                zForceTarget * forceFraction, false};
    }

    const double motionTime = std::max(0.0, time - rampDuration);
    double motionFraction, motionRate;
    hybrid_fp_ft::positionProfile(motionTime, motionDuration, motionFraction, motionRate);
    return {xDistance * motionFraction, xDistance * motionRate,
            referenceAcceleration(motionTime, motionDuration, xDistance),
            zForceTarget * forceFraction, motionTime >= motionDuration};
}

inline double updateMotionIntegral(double integral, double increment, double pdAcceleration,
                                   double integralLimit, double accelerationLimit) {
    const double trial = std::max(-integralLimit, std::min(integralLimit, integral + increment));
    const double requested = pdAcceleration + trial;
    // Allow the integral to unwind while the total acceleration command is capped.
    if ((requested > accelerationLimit && trial > integral) ||
        (requested < -accelerationLimit && trial < integral)) {
        return integral;
    }
    return trial;
}

struct PlanarState {
    Eigen::Matrix2d jacobian;
    Eigen::Matrix2d inertia;
    Eigen::Vector2d biasTorque;
    Eigen::Vector2d jacobianRateVelocity;
};

template <typename JointVector>
inline Eigen::Vector2d activeValues(const JointVector& vector) {
    return Eigen::Vector2d(vector[activeJoints[0]], vector[activeJoints[1]]);
}

template <typename JointVector>
inline bool heldJointsNearZero(const JointVector& position, double tolerance) {
    for (Eigen::Index j = 0; j < position.size(); ++j) {
        if (j != activeJoints[0] && j != activeJoints[1] && !(std::abs(position[j]) <= tolerance)) {
            return false;
        }
    }
    return true;
}

inline bool solveCartesianForce(const PlanarState& model, double xAcceleration, double zForce,
                                Eigen::Vector2d& force) {
    if (!model.jacobian.allFinite() || !model.inertia.allFinite() ||
        !model.biasTorque.allFinite() || !model.jacobianRateVelocity.allFinite() ||
        !std::isfinite(xAcceleration) || !std::isfinite(zForce) ||
        !model.inertia.isApprox(model.inertia.transpose(), 1e-9)) {
        return false;
    }
    const Eigen::LLT<Eigen::Matrix2d> inertiaFactor(model.inertia);
    const Eigen::FullPivLU<Eigen::Matrix2d> jacobianFactor(model.jacobian);
    if (inertiaFactor.info() != Eigen::Success || !jacobianFactor.isInvertible() ||
        jacobianFactor.rcond() < 1e-3) {
        return false;
    }

    // With a square J, Lambda = J^-T M J^-1. Solve for the Cartesian
    // force directly, including the normal component of tangential inertia.
    const Eigen::Vector2d taskAcceleration(xAcceleration, 0.0);
    const Eigen::Vector2d jointAcceleration =
        jacobianFactor.solve(taskAcceleration - model.jacobianRateVelocity);
    const Eigen::Vector2d inverseDynamicsTorque = model.inertia * jointAcceleration + model.biasTorque;
    force = model.jacobian.transpose().fullPivLu().solve(inverseDynamicsTorque);
    force[1] += zForce;
    return force.allFinite();
}

template <size_t DOF>
class PlanarKinematics {
public:
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    using Kinematics = barrett::math::Kinematics<DOF>;

    explicit PlanarKinematics(const libconfig::Setting& kinematics)
        : perturbedKinematics_(kinematics) {}

    void evaluate(const Kinematics& kinematics, const jp_type& position,
                  const jv_type& velocity, PlanarState& model) {
        model.jacobian = planarJacobian(kinematics);
        // Differentiate J along the measured active-joint velocity.
        constexpr double step = 1e-4;
        jp_type before = position, after = position;
        for (int joint : activeJoints) {
            before[joint] -= step * velocity[joint];
            after[joint] += step * velocity[joint];
        }
        const jv_type zeroVelocity(0.0);
        perturbedKinematics_.eval(before, zeroVelocity);
        const Eigen::Matrix2d beforeJacobian = planarJacobian(perturbedKinematics_);
        perturbedKinematics_.eval(after, zeroVelocity);
        model.jacobianRateVelocity = (planarJacobian(perturbedKinematics_) - beforeJacobian)
                                   * activeValues(velocity) / (2.0 * step);
    }

private:
    static Eigen::Matrix2d planarJacobian(const Kinematics& kinematics) {
        const gsl_matrix* jacobian = kinematics.impl->tool_jacobian;
        Eigen::Matrix2d result;
        for (size_t i = 0; i < activeJoints.size(); ++i) {
            result(0, i) = gsl_matrix_get(jacobian, 0, activeJoints[i]);
            result(1, i) = gsl_matrix_get(jacobian, 2, activeJoints[i]);
        }
        return result;
    }

    Kinematics perturbedKinematics_;
};

}  // namespace hybrid_cartesian_dynamics
