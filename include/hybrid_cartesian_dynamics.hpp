#pragma once

#include "dynamics/beta.h"
#include "dynamics/regressor_W.h"
#include "hybrid_cartesian_model.hpp"

namespace hybrid_cartesian_dynamics {

constexpr int identifiedJointCount = 4;

template <size_t DOF>
class PlanarDynamics {
public:
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    using Kinematics = barrett::math::Kinematics<DOF>;

    explicit PlanarDynamics(const libconfig::Setting& kinematics)
        : kinematics_(kinematics) {}

    PlanarDynamics(const libconfig::Setting& kinematics, const libconfig::Setting&)
        : PlanarDynamics(kinematics) {}

    jt_type commandTorque(const jt_type& taskAndHold, const jt_type& barrettGravity) const {
        return cancelTaskJointGravity(taskAndHold, barrettGravity);
    }

    // The identified bias already contains gravity.
    void addWamGravity(PlanarState&, const jt_type&) const {}

    PlanarState evaluate(const Kinematics& kinematics, const jp_type& position, const jv_type& velocity) {
        PlanarState model;
        kinematics_.evaluate(kinematics, position, velocity, model);
        evaluateIdentifiedDynamics(position.template head<identifiedJointCount>(),
                                   velocity.template head<identifiedJointCount>(), model);
        return model;
    }

private:
    void evaluateIdentifiedDynamics(const Eigen::Vector4d& position,
                                    const Eigen::Vector4d& velocity, PlanarState& model) const {
        Eigen::Vector4d acceleration = Eigen::Vector4d::Zero();
        Eigen::Matrix<double, 4, 30, Eigen::RowMajor> regressor;

        // The zero-acceleration evaluation contains Coriolis, centrifugal, friction, and gravity torque.
        calculate_W(position.data(), velocity.data(), acceleration.data(), regressor.data());
        const Eigen::Vector4d biasTorque = regressor * beta_;
        model.biasTorque = activeValues(biasTorque);

        // Keep the inertia block for the two joints driving the task.
        for (size_t i = 0; i < activeJoints.size(); ++i) {
            const int joint = activeJoints[i];
            acceleration[joint] = 1.0;
            calculate_W(position.data(), velocity.data(), acceleration.data(), regressor.data());
            const Eigen::Vector4d inertiaTorque = regressor * beta_ - biasTorque;
            model.inertia.col(i) = activeValues(inertiaTorque);
            acceleration[joint] = 0.0;
        }
    }

    PlanarKinematics<DOF> kinematics_;
    const Eigen::Matrix<double, 30, 1> beta_ = initialize_beta();
};

}  // namespace hybrid_cartesian_dynamics
