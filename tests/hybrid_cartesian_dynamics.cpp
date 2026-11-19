#include "hybrid_cartesian_dynamics.hpp"
#include "hybrid_cartesian_dynamics_log.hpp"
#include "hybrid_cartesian_nominal.hpp"
#include "hybrid_fp_ft/position_reference.hpp"
#include "hybrid_ft/joint_torque.hpp"

#include <algorithm>
#include <cassert>
#include <limits>
#include <sstream>
#include <string>

namespace {
void checkNominalModel(const libconfig::Setting& robot) {
    using namespace hybrid_cartesian_dynamics;
    using JointPosition = barrett::units::JointPositions<7>::type;
    using JointVelocity = barrett::units::JointVelocities<7>::type;
    using JointTorque = barrett::units::JointTorques<7>::type;
    barrett::math::Kinematics<7> kin(robot["kinematics"]);
    NominalPlanarDynamics<7> model(robot["kinematics"], robot);
    PlanarDynamics<7> identifiedModel(robot["kinematics"]);
    const auto& links = robot["dynamics"]["moving"];
    const JointVelocity resting(0.0);
    struct LinkPose {
        Eigen::Vector3d com;
        Eigen::Matrix3d rotation;
    };
    const auto poses = [&](const JointPosition& q) {
        kin.eval(q, resting);
        std::array<LinkPose, 7> result;
        for (int i = 0; i < links.getLength(); ++i) {
            const auto* link = kin.impl->link[i];
            result[i].rotation = barrett::math::Matrix<3, 3>(link->rot_to_world);
            Eigen::Vector3d comOffset;
            for (int axis = 0; axis < 3; ++axis) {
                result[i].com[axis] = gsl_vector_get(link->origin_pos, axis);
                comOffset[axis] = double(links[i]["com"][axis]);
            }
            result[i].com += result[i].rotation * comOffset;
        }
        return result;
    };
    const auto potential = [&](const JointPosition& q) {
        const auto bodies = poses(q);
        double energy = 0.0;
        for (int i = 0; i < links.getLength(); ++i) {
            energy += 9.81 * double(links[i]["mass"]) * bodies[i].com.z();
        }
        return energy;
    };

    const auto kineticInertia = [&](const JointPosition& q) {
        // Differentiate each COM pose with respect to the two active joints.
        // The kinetic energy then yields the independent reference inertia.
        constexpr double step = 1e-4;
        const auto center = poses(q);
        std::array<std::array<LinkPose, 7>, 2> before, after;
        for (size_t j = 0; j < activeJoints.size(); ++j) {
            JointPosition lower = q, upper = q;
            lower[activeJoints[j]] -= step;
            upper[activeJoints[j]] += step;
            before[j] = poses(lower);
            after[j] = poses(upper);
        }
        Eigen::Matrix2d inertia = Eigen::Matrix2d::Zero();
        for (int i = 0; i < links.getLength(); ++i) {
            Eigen::Matrix<double, 3, 2> linear, angular;
            for (size_t j = 0; j < activeJoints.size(); ++j) {
                linear.col(j) = (after[j][i].com - before[j][i].com) / (2.0 * step);
                const Eigen::Matrix3d omega = (after[j][i].rotation - before[j][i].rotation)
                    * center[i].rotation.transpose() / (2.0 * step);
                angular.col(j) << omega(2, 1), omega(0, 2), omega(1, 0);
            }
            Eigen::Matrix3d localInertia;
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    localInertia(row, col) = double(links[i]["I"][row][col]);
                }
            }
            const Eigen::Matrix<double, 3, 2> localAngular = center[i].rotation.transpose() * angular;
            inertia += double(links[i]["mass"]) * linear.transpose() * linear
                     + localAngular.transpose() * localInertia * localAngular;
        }
        return inertia;
    };

    JointPosition q(0.0);
    q[1] = 1.3642;
    q[3] = 1.2731;
    constexpr double step = 1e-5;
    JointTorque gravity;
    for (Eigen::Index j = 0; j < q.size(); ++j) {
        JointPosition before = q, after = q;
        before[j] -= step;
        after[j] += step;
        gravity[j] = (potential(after) - potential(before)) / (2.0 * step);
    }
    assert(std::abs(gravity[1]) > 1.0 && std::abs(gravity[3]) > 0.1);

    JointVelocity velocity(0.0);
    velocity[1] = 0.12;
    velocity[3] = -0.17;
    kin.eval(q, velocity);
    assert(model.gravityTorque(kin).isApprox(gravity, 1e-8));
    PlanarState state = model.evaluate(kin, q, velocity);
    const Eigen::Matrix2d inertia = kineticInertia(q);
    assert(inertia.isApprox(state.inertia, 1e-8));
    const Eigen::Vector2d activeVelocity = activeValues(velocity);
    Eigen::Matrix2d inertiaRate = Eigen::Matrix2d::Zero();
    Eigen::Vector2d kineticEnergyGradient;
    for (size_t j = 0; j < activeJoints.size(); ++j) {
        JointPosition before = q, after = q;
        before[activeJoints[j]] -= step;
        after[activeJoints[j]] += step;
        const Eigen::Matrix2d derivative = (kineticInertia(after) - kineticInertia(before)) / (2.0 * step);
        inertiaRate += derivative * activeVelocity[j];
        kineticEnergyGradient[j] = 0.5 * activeVelocity.dot(derivative * activeVelocity);
    }
    const Eigen::Vector2d dynamicBias = inertiaRate * activeVelocity - kineticEnergyGradient;
    assert((state.biasTorque - dynamicBias).norm() < 3e-8);
    model.addWamGravity(state, gravity);
    const Eigen::Vector2d bias = activeValues(gravity) + dynamicBias;
    assert(state.biasTorque.isApprox(bias, 1e-8));
    kin.eval(q, velocity);
    const Eigen::Vector2d recursiveBias = activeValues(model.inverseDynamicsBias(kin, velocity));
    assert(recursiveBias.isApprox(bias, 1e-8));

    const double xAcceleration = 0.75, zForce = -7.5;
    Eigen::Vector2d wrench;
    assert(solveCartesianForce(state, xAcceleration, zForce, wrench));
    const Eigen::Vector2d jointAcceleration = state.jacobian.fullPivLu().solve(
        Eigen::Vector2d(xAcceleration, 0.0) - state.jacobianRateVelocity);
    const Eigen::Vector2d taskTorque = state.jacobian.transpose() * wrench;
    assert(taskTorque.isApprox(inertia * jointAcceleration + bias
        + zForce * state.jacobian.row(1).transpose(), 1e-8));

    JointTorque taskAndHold(0.2);
    taskAndHold[1] = taskTorque[0];
    taskAndHold[3] = taskTorque[1];
    const JointTorque calibratedGravity(1.0);
    const JointTorque total = identifiedModel.commandTorque(taskAndHold, calibratedGravity) + calibratedGravity;
    const JointTorque nominalTotal = model.commandTorque(taskAndHold, calibratedGravity) + calibratedGravity;
    assert(activeValues(total).isApprox(taskTorque, 1e-10));
    assert(nominalTotal.isApprox(total, 1e-10));
    for (int j : {0, 2, 4, 5, 6}) {
        assert(std::abs(total[j] - (taskAndHold[j] + calibratedGravity[j])) < 1e-8);
    }
}

void checkContactReference() {
    using namespace hybrid_cartesian_dynamics;
    constexpr double ramp = 2.0, duration = 36.0, distance = 0.3, target = -10.0;
    constexpr std::array<double, 4> times{{0.0, ramp / 4.0, ramp / 2.0, ramp}};
    constexpr std::array<double, 4> fractions{{0.0, 0.103515625, 0.5, 1.0}};
    for (std::size_t i = 0; i < times.size(); ++i) {
        const auto ref = contactReference(times[i], ramp, duration, distance, target);
        assert(ref.xOffset == 0.0 && ref.xVelocity == 0.0 && ref.xAcceleration == 0.0);
        assert(ref.zForce == target * fractions[i]);
        assert(!ref.finalHold);
    }

    const double moveTime = 9.0;
    const auto moving = contactReference(ramp + moveTime, ramp, duration, distance, target);
    double fraction, rate;
    hybrid_fp_ft::positionProfile(moveTime, duration, fraction, rate);
    assert(moving.xOffset == distance * fraction && moving.xVelocity == distance * rate);
    assert(moving.xAcceleration == referenceAcceleration(moveTime, duration, distance));
    assert(moving.zForce == target);
    assert(!moving.finalHold);
    assert(!contactReference(ramp + duration - 0.002, ramp, duration, distance, target).finalHold);
    assert(contactReference(ramp + duration, ramp, duration, distance, target).finalHold);

    const auto finished = contactReference(ramp + duration + 1.0, ramp, duration, distance, target);
    assert(finished.xOffset == distance && finished.xVelocity == 0.0 && finished.xAcceleration == 0.0);
    assert(finished.zForce == target);
    assert(finished.finalHold);
}

void checkSineContactReference() {
    using namespace hybrid_cartesian_dynamics;
    constexpr double ramp = 1.5, duration = 4.0, originOffset = 0.16;
    constexpr double amplitude = 0.05, period = 8.0, target = -10.0;
    constexpr double time = 0.75;
    const auto duringRamp = contactReference(time, ramp, duration, originOffset, target,
                                             true, amplitude, period);
    double expectedPosition, expectedVelocity, forceFraction, forceRate;
    hybrid_fp_ft::sinePositionProfile(time, duration, period, amplitude, 0.0, originOffset,
                                      expectedPosition, expectedVelocity);
    hybrid_fp_ft::positionProfile(time, ramp, forceFraction, forceRate);
    assert(std::abs(duringRamp.xOffset - expectedPosition) < 1e-12);
    assert(std::abs(duringRamp.xVelocity - expectedVelocity) < 1e-12);
    assert(std::abs(duringRamp.zForce - target * forceFraction) < 1e-12);
    assert(!duringRamp.finalHold);
    assert(contactReference(0.0, ramp, duration, originOffset, target, true,
                            amplitude, period).xOffset == 0.0);
    assert(contactReference(0.0, ramp, duration, originOffset, target, true,
                            amplitude, period).xVelocity == 0.0);

    constexpr double dt = 1e-5;
    for (double sampleTime : {0.75, 4.75}) {
        const auto before = contactReference(sampleTime - dt, ramp, duration, originOffset,
                                             target, true, amplitude, period);
        const auto after = contactReference(sampleTime + dt, ramp, duration, originOffset,
                                            target, true, amplitude, period);
        const auto at = contactReference(sampleTime, ramp, duration, originOffset,
                                         target, true, amplitude, period);
        assert(std::abs(at.xAcceleration - (after.xVelocity - before.xVelocity) / (2.0 * dt)) < 1e-7);
        assert(!at.finalHold);
    }

    double sinePosition, sineVelocity;
    hybrid_fp_ft::sinePositionProfile(4.75, duration, period, amplitude, 0.0, originOffset,
                                      sinePosition, sineVelocity);
    const auto afterRamp = contactReference(4.75, ramp, duration, originOffset, target,
                                            true, amplitude, period);
    assert(std::abs(afterRamp.xOffset - sinePosition) < 1e-12);
    assert(std::abs(afterRamp.xVelocity - sineVelocity) < 1e-12);
    assert(afterRamp.zForce == target);
}

void checkMotionIntegral() {
    using hybrid_cartesian_dynamics::updateMotionIntegral;
    constexpr double integralLimit = 6.0, accelerationLimit = 8.0;
    assert(updateMotionIntegral(0.0, 0.25, 1.0, integralLimit, accelerationLimit) == 0.25);
    assert(updateMotionIntegral(0.0, 0.0, 1.0, integralLimit, accelerationLimit) == 0.0);
    assert(updateMotionIntegral(5.75, 1.0, 1.0, integralLimit, accelerationLimit) == 6.0);
    assert(updateMotionIntegral(-5.75, -1.0, -1.0, integralLimit, accelerationLimit) == -6.0);
    assert(updateMotionIntegral(1.5, 0.5, 7.0, integralLimit, accelerationLimit) == 1.5);
    assert(updateMotionIntegral(-1.5, -0.5, -7.0, integralLimit, accelerationLimit) == -1.5);
    assert(updateMotionIntegral(2.0, -0.25, 7.0, integralLimit, accelerationLimit) == 1.75);
    assert(updateMotionIntegral(-2.0, 0.25, -7.0, integralLimit, accelerationLimit) == -1.75);
}

void checkLogBuffer() {
    using namespace hybrid_cartesian_dynamics;
    LogBuffer log(2);
    LogSample sample{};
    log.record(sample);
    sample[0] = 0.002;
    log.record(sample);
    sample[0] = 0.004;
    log.record(sample);
    assert(log.dropped() == 1);

    log.swapPending();
    sample[0] = 0.006;
    log.record(sample);
    std::ostringstream output;
    writeLogRow(output, logColumns);
    log.writeBatch(output);
    log.swapPending();
    log.writeBatch(output);
    std::istringstream rows(output.str());
    std::string row;
    assert(std::getline(rows, row) && row.find("time_s,pdes_x,pcur_x") == 0);
    for (const char* timestamp : {"0,", "0.002,", "0.006,"}) {
        assert(std::getline(rows, row) && row.find(timestamp) == 0);
        assert(static_cast<std::size_t>(std::count(row.begin(), row.end(), ',')) == logColumns.size() - 1);
    }
    assert(!std::getline(rows, row));
    log.record(sample);
    log.reset();
    assert(log.dropped() == 0);
    log.swapPending();
    std::ostringstream nextRun;
    log.writeBatch(nextRun);
    assert(nextRun.str().empty());
}
} // namespace

int main() {
    using namespace hybrid_cartesian_dynamics;
    checkContactReference();
    checkSineContactReference();
    checkMotionIntegral();
    checkLogBuffer();
    PlanarState sample;
    sample.jacobian.setIdentity();
    sample.inertia << 3.0, 0.7, 0.7, 2.0;
    sample.biasTorque << 0.4, -0.2;
    sample.jacobianRateVelocity.setZero();
    Eigen::Vector2d force;
    assert(solveCartesianForce(sample, 2.0, -5.0, force));
    assert(force.isApprox(Eigen::Vector2d(6.4, -3.8), 1e-12));

    using JointPosition = barrett::units::JointPositions<7>::type;
    using JointVelocity = barrett::units::JointVelocities<7>::type;
    using JointTorque = barrett::units::JointTorques<7>::type;
    using Mapper = hybrid_ft::BaseControlToJointTorque<7>;
    // Nonzero Jacobian columns on held joints make accidental task torque observable.
    Mapper::jacobian_type jacobian;
    jacobian.setOnes();
    jacobian(0, 3) = 0.0;
    jacobian(2, 1) = 0.0;
    Mapper::base_control_type taskForce;
    taskForce << force[0], 0.0, force[1];
    JointPosition initial;
    initial << 0.04, 1.2, -0.02, 2.0, 0.0, -0.05, 0.03;
    JointVelocity velocity(0.01);
    barrett::math::Vector<7>::type kp(10.0), kd(2.0);
    JointTorque barrettGravity;
    barrettGravity << 0.3, 4.5, -1.0, 2.0, -0.1, -0.2, 0.5;
    const JointTorque taskAndHold = Mapper::calculateTorque(
        jacobian, taskForce, initial, velocity, JointPosition(0.0), kp, kd);
    const JointTorque total = cancelTaskJointGravity(taskAndHold, barrettGravity) + barrettGravity;
    assert(activeValues(total).isApprox(force, 1e-12));
    for (int held : {0, 2, 4, 5, 6}) {
        assert(std::abs(total[held] - (barrettGravity[held] - 10.0 * initial[held] - 0.02)) < 1e-12);
    }
    assert(heldJointsNearZero(initial, 0.05));
    initial[5] = -0.051;
    assert(!heldJointsNearZero(initial, 0.05));
    initial[5] = 0.0;
    initial[6] = std::numeric_limits<double>::quiet_NaN();
    assert(!heldJointsNearZero(initial, 0.05));

    sample.jacobian(0, 0) = 1e-5;
    assert(!solveCartesianForce(sample, 2.0, -5.0, force));
    sample.jacobian.setIdentity();
    sample.inertia(1, 1) = -2.0;
    assert(!solveCartesianForce(sample, 2.0, -5.0, force));
    sample.inertia(1, 1) = 2.0;
    assert(!solveCartesianForce(sample, std::numeric_limits<double>::quiet_NaN(), -5.0, force));

    // The quintic acceleration must match the derivative of the reference velocity.
    const double duration = 5.0, time = 1.3, distance = 0.02, dt = 1e-5;
    double fraction, beforeRate, afterRate;
    hybrid_fp_ft::positionProfile(time - dt, duration, fraction, beforeRate);
    hybrid_fp_ft::positionProfile(time + dt, duration, fraction, afterRate);
    assert(std::abs(referenceAcceleration(time, duration, distance)
                    - distance * (afterRate - beforeRate) / (2.0 * dt)) < 1e-10);
    assert(referenceAcceleration(0, duration, distance) == 0);
    assert(referenceAcceleration(duration + 1, duration, distance) == 0);

    libconfig::Config config;
    config.readFile("default.conf");
    checkNominalModel(config.lookup("wam7w"));
    const auto& kinematics = config.lookup("wam7w")["kinematics"];
    barrett::math::Kinematics<7> kin(kinematics);
    PlanarDynamics<7> planar(kinematics);
    JointPosition q(0.0), before(0.0), after(0.0);
    JointVelocity qd(0.0), zeroVelocity(0.0);
    q[1] = 1.164;
    q[3] = 2.026;
    qd[0] = 0.03;
    qd[1] = 0.15;
    qd[2] = -0.02;
    qd[3] = -0.25;
    kin.eval(q, qd);
    const PlanarState model = planar.evaluate(kin, q, qd);
    NominalPlanarDynamics<7> nominal(kinematics, config.lookup("wam7w"));
    const PlanarState nominalModel = nominal.evaluate(kin, q, qd);
    assert(solveCartesianForce(model, 0.18, -3.0, force));
    const Eigen::Vector2d jointAcceleration =
        model.jacobian.fullPivLu().solve(Eigen::Vector2d(0.18, 0.0) - model.jacobianRateVelocity);
    // Compare both task joints against the full identified model, including bias and gravity.
    Eigen::Vector4d qdd = Eigen::Vector4d::Zero();
    qdd[1] = jointAcceleration[0];
    qdd[3] = jointAcceleration[1];
    const Eigen::Vector4d q4 = q.head<4>(), qd4 = qd.head<4>();
    const auto beta = initialize_beta();
    const Eigen::Vector4d identifiedTorque = calculate_W_eigen(q4, qd4, qdd) * beta;
    assert((model.jacobian.transpose() * force)
        .isApprox(activeValues(identifiedTorque) - 3.0 * model.jacobian.row(1).transpose(), 1e-10));

    const Eigen::Vector4d zero = Eigen::Vector4d::Zero();
    kin.eval(q, zeroVelocity);
    const PlanarState resting = planar.evaluate(kin, q, zeroVelocity);
    assert(solveCartesianForce(resting, 0.0, 0.0, force));
    const Eigen::Vector4d identifiedGravity = calculate_W_eigen(q4, zero, zero) * beta;
    assert(identifiedGravity.norm() > 1.0);
    assert((resting.jacobian.transpose() * force).isApprox(activeValues(identifiedGravity), 1e-10));

    const auto toolXZ = [&]() {
        const gsl_vector* position = kin.impl->tool->origin_pos;
        return Eigen::Vector2d(gsl_vector_get(position, 0), gsl_vector_get(position, 2));
    };
    const Eigen::Vector2d center = toolXZ();
    before = after = q;
    constexpr double step = 1e-3;
    for (int joint : activeJoints) {
        before[joint] -= step * qd[joint];
        after[joint] += step * qd[joint];
    }
    kin.eval(before, zeroVelocity);
    const Eigen::Vector2d previous = toolXZ();
    kin.eval(after, zeroVelocity);
    const Eigen::Vector2d numericalAcceleration = (toolXZ() - 2.0 * center + previous) / (step * step);
    assert((model.jacobianRateVelocity - numericalAcceleration).norm() < 1e-7);
}
