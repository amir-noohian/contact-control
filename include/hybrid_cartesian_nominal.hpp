#pragma once

#include "hybrid_cartesian_model.hpp"

#include <stdexcept>
#include <string>

namespace hybrid_cartesian_dynamics {

template <size_t DOF>
class NominalPlanarDynamics {
public:
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    using Kinematics = barrett::math::Kinematics<DOF>;

    NominalPlanarDynamics(const libconfig::Setting& kinematics, const libconfig::Setting& robot)
        : kinematics_(kinematics) {
        const auto& moving = robot["dynamics"]["moving"];
        if (moving.getLength() != DOF) {
            throw std::runtime_error("The nominal dynamics must describe every WAM link");
        }
        for (size_t i = 0; i < DOF; ++i) {
            const auto& link = moving[static_cast<int>(i)];
            auto& model = links_[i];
            model.mass = link["mass"];
            for (int row = 0; row < 3; ++row) {
                model.centerOfMass[row] = double(link["com"][row]);
                for (int col = 0; col < 3; ++col) {
                    model.inertia(row, col) = double(link["I"][row][col]);
                }
            }
            if (!std::isfinite(model.mass) || model.mass < 0.0 ||
                !model.centerOfMass.allFinite() || !model.inertia.allFinite() ||
                !model.inertia.isApprox(model.inertia.transpose(), 1e-10)) {
                throw std::runtime_error("Invalid nominal dynamics for WAM link " + std::to_string(i + 1));
            }
        }
    }

    PlanarState evaluate(const Kinematics& kinematics, const jp_type& position, const jv_type& velocity) {
        PlanarState model;
        kinematics_.evaluate(kinematics, position, velocity, model);
        model.inertia.setZero();
        model.biasTorque.setZero();

        std::array<Eigen::Vector3d, DOF> jointOrigins, jointAxes;
        Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d angularAcceleration = Eigen::Vector3d::Zero();
        Eigen::Vector3d originAcceleration = Eigen::Vector3d::Zero();
        const auto* previous = kinematics.impl->base;
        for (size_t i = 0; i < DOF; ++i) {
            const auto* link = kinematics.impl->link[i];
            const auto& nominal = links_[i];
            const Eigen::Vector3d origin = vector(link->origin_pos);
            jointOrigins[i] = vector(previous->origin_pos);
            jointAxes[i] = vector(previous->axis_z);
            const Eigen::Vector3d jointVelocity = jointAxes[i] * velocity[i];

            // Zero joint accelerations. The link origin includes the DH offset
            // from the joint axis and rotates with the current joint.
            angularAcceleration += angularVelocity.cross(jointVelocity);
            angularVelocity += jointVelocity;
            const Eigen::Vector3d offset = origin - jointOrigins[i];
            originAcceleration += angularAcceleration.cross(offset)
                                + angularVelocity.cross(angularVelocity.cross(offset));

            const Eigen::Matrix3d rotation = barrett::math::Matrix<3, 3>(link->rot_to_world);
            const Eigen::Vector3d comOffset = rotation * nominal.centerOfMass;
            const Eigen::Vector3d com = origin + comOffset;
            const Eigen::Vector3d comAcceleration = originAcceleration + angularAcceleration.cross(comOffset)
                + angularVelocity.cross(angularVelocity.cross(comOffset));
            const Eigen::Matrix3d worldInertia = rotation * nominal.inertia * rotation.transpose();
            const Eigen::Vector3d force = nominal.mass * comAcceleration;
            const Eigen::Vector3d moment = worldInertia * angularAcceleration
                + angularVelocity.cross(worldInertia * angularVelocity);

            Eigen::Matrix<double, 3, 2> linear = Eigen::Matrix<double, 3, 2>::Zero();
            Eigen::Matrix<double, 3, 2> angular = Eigen::Matrix<double, 3, 2>::Zero();
            for (size_t col = 0; col < activeJoints.size(); ++col) {
                const auto j = static_cast<size_t>(activeJoints[col]);
                if (j <= i) {
                    angular.col(col) = jointAxes[j];
                    linear.col(col) = jointAxes[j].cross(com - jointOrigins[j]);
                }
            }
            model.inertia += nominal.mass * linear.transpose() * linear
                           + angular.transpose() * worldInertia * angular;
            model.biasTorque += linear.transpose() * force + angular.transpose() * moment;
            previous = link;
        }
        return model;
    }

    jt_type inverseDynamicsBias(const Kinematics& kinematics, const jv_type& velocity) const {
        jt_type torque(0.0);
        std::array<Eigen::Vector3d, DOF> jointOrigins, jointAxes;
        Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d angularAcceleration = Eigen::Vector3d::Zero();
        Eigen::Vector3d originAcceleration = Eigen::Vector3d::Zero();
        const auto* previous = kinematics.impl->base;
        for (size_t i = 0; i < DOF; ++i) {
            const auto* link = kinematics.impl->link[i];
            jointOrigins[i] = vector(previous->origin_pos);
            jointAxes[i] = vector(previous->axis_z);

            const Eigen::Vector3d origin = vector(link->origin_pos);
            const Eigen::Vector3d jointVelocity = jointAxes[i] * velocity[i];
            angularAcceleration += angularVelocity.cross(jointVelocity);
            angularVelocity += jointVelocity;
            const Eigen::Vector3d offset = origin - jointOrigins[i];
            originAcceleration += angularAcceleration.cross(offset)
                + angularVelocity.cross(angularVelocity.cross(offset));

            const Eigen::Matrix3d rotation = barrett::math::Matrix<3, 3>(link->rot_to_world);
            const Eigen::Vector3d comOffset = rotation * links_[i].centerOfMass;
            const Eigen::Vector3d com = origin + comOffset;
            const Eigen::Vector3d comAcceleration = originAcceleration
                + angularAcceleration.cross(comOffset)
                + angularVelocity.cross(angularVelocity.cross(comOffset));
            const Eigen::Vector3d gravityForce(0.0, 0.0, links_[i].mass * 9.81);
            const Eigen::Vector3d force = links_[i].mass * comAcceleration + gravityForce;
            const Eigen::Matrix3d worldInertia = rotation * links_[i].inertia * rotation.transpose();
            const Eigen::Vector3d moment = worldInertia * angularAcceleration
                + angularVelocity.cross(worldInertia * angularVelocity);
            for (size_t joint = 0; joint <= i; ++joint) {
                torque[joint] += jointAxes[joint].dot(
                    (com - jointOrigins[joint]).cross(force) + moment);
            }
            previous = link;
        }
        return torque;
    }

    jt_type gravityTorque(const Kinematics& kinematics) const {
        const jv_type zeroVelocity(0.0);
        return inverseDynamicsBias(kinematics, zeroVelocity);
    }

    void addWamGravity(PlanarState& model, const jt_type& barrettGravity) const {
        model.biasTorque += activeValues(barrettGravity);
    }

    jt_type commandTorque(const jt_type& taskAndHold, const jt_type& barrettGravity) const {
        return cancelTaskJointGravity(taskAndHold, barrettGravity);
    }

private:
    struct Link {
        double mass;
        Eigen::Vector3d centerOfMass;
        Eigen::Matrix3d inertia;
    };

    static Eigen::Vector3d vector(const gsl_vector* value) {
        return Eigen::Vector3d(gsl_vector_get(value, 0), gsl_vector_get(value, 1), gsl_vector_get(value, 2));
    }

    PlanarKinematics<DOF> kinematics_;
    std::array<Link, DOF> links_;
};

}  // namespace hybrid_cartesian_dynamics
