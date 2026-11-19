#include "hybrid_cartesian_dynamics.hpp"
#include "joint_computed_torque/log_path.hpp"

#define BARRETT_SMF_VALIDATE_ARGS
#define BARRETT_SMF_DONT_WAIT_FOR_SHIFT_ACTIVATE
#include <barrett/standard_main_function.h>

#include <array>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace {
using hybrid_cartesian_dynamics::activeJoints;
using hybrid_cartesian_dynamics::activeValues;
using Clock = std::chrono::steady_clock;

constexpr double targetQ2 = 1.3642, targetQ4 = 1.2731;
constexpr double samplePeriod = 0.02, sampleDuration = 3.0;
constexpr double positionTolerance = 0.05, speedTolerance = 0.05;
constexpr double maxPositionDrift = 0.005;
constexpr auto captureTimeout = std::chrono::seconds(12);

Eigen::Vector2d predictedHoldTorque(const Eigen::Vector4d& q) {
    const Eigen::Vector4d zero = Eigen::Vector4d::Zero();
    const Eigen::Vector4d torque = calculate_W_eigen(q, zero, zero) * initialize_beta();
    return activeValues(torque);
}

template <size_t DOF>
struct Sample {
    double time;
    std::array<double, DOF> position, velocity;
    std::array<double, 2> hold;
};

template <size_t DOF>
void report(const std::vector<Sample<DOF>>& samples) {
    const char* dataDir = HYBRID_CARTESIAN_STATIC_DATA_DIR;
    const char* logDir = HYBRID_CARTESIAN_STATIC_DATA_DIR "/hybrid_cartesian_static_check";
    for (const char* dir : {dataDir, logDir}) {
        if (mkdir(dir, 0775) != 0 && errno != EEXIST) {
            throw std::system_error(errno, std::generic_category(), std::string("Create ") + dir);
        }
    }
    const auto path = joint_computed_torque::reserveTrackingCsv(logDir, "static_hold");
    std::ofstream csv(path);
    csv.exceptions(std::ios::failbit | std::ios::badbit);
    csv << std::setprecision(16) << "time_s";
    for (size_t j = 1; j <= DOF; ++j) csv << ",q" << j;
    for (size_t j = 1; j <= DOF; ++j) csv << ",qd" << j;
    csv << ",hold_cmd_2_Nm,hold_cmd_4_Nm,beta_static_2_Nm,beta_static_4_Nm"
           ",hold_minus_beta_2_Nm,hold_minus_beta_4_Nm\n";

    Eigen::Vector2d meanHold = Eigen::Vector2d::Zero();
    Eigen::Vector2d meanPrediction = Eigen::Vector2d::Zero();
    Eigen::Vector2d meanPosition = Eigen::Vector2d::Zero();
    for (const auto& sample : samples) {
        const Eigen::Map<const Eigen::Vector4d> q(sample.position.data());
        const Eigen::Vector2d hold(sample.hold[0], sample.hold[1]);
        const Eigen::Vector2d prediction = predictedHoldTorque(q);
        const Eigen::Vector2d difference = hold - prediction;
        meanHold += hold;
        meanPrediction += prediction;
        meanPosition += activeValues(q);
        csv << sample.time - samples.front().time;
        for (double value : sample.position) csv << ',' << value;
        for (double value : sample.velocity) csv << ',' << value;
        csv << ',' << hold[0] << ',' << hold[1] << ',' << prediction[0] << ',' << prediction[1]
            << ',' << difference[0] << ',' << difference[1] << '\n';
    }
    csv.close();
    meanHold /= samples.size();
    meanPrediction /= samples.size();
    meanPosition /= samples.size();
    std::cout << "\nJoint   Mean angle (rad)   Hold command (Nm)   Beta static (Nm)   Hold minus beta (Nm)\n";
    for (size_t i = 0; i < activeJoints.size(); ++i) {
        std::cout << std::fixed << std::setprecision(4)
                  << std::setw(5) << activeJoints[i] + 1 << std::setw(19) << meanPosition[i]
                  << std::setw(20) << meanHold[i] << std::setw(18) << meanPrediction[i]
                  << std::setw(22) << meanHold[i] - meanPrediction[i] << '\n';
    }
    std::cout << "Static hold CSV " << path << '\n';
}
} // namespace

bool validate_args(int argc, char** argv) {
    if (argc == 1) return true;
    const std::string option(argv[1]);
    if (argc == 2 && option == "--predict-only") {
        const Eigen::Vector4d q(0.0, targetQ2, 0.0, targetQ4);
        const auto torque = predictedHoldTorque(q);
        std::cout << "Beta static prediction with q2=" << targetQ2 << " rad and q4=" << targetQ4
                  << " rad\nJoint 2: " << torque[0] << " Nm\nJoint 4: " << torque[1] << " Nm\n";
        std::exit(torque.allFinite() ? 0 : 1);
    }
    std::cout << "Usage: hybrid_cartesian_static_check [--predict-only]\n";
    if (argc == 2 && option == "--help") std::exit(0);
    return false;
}

template <size_t DOF>
int wam_main(int, char**, barrett::ProductManager& pm, barrett::systems::Wam<DOF>& wam) {
    if (DOF != 4 && DOF != 7) {
        std::cerr << "The static check requires a 4- or 7-DOF WAM.\n";
        return 1;
    }
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    auto* const safety = pm.getSafetyModule();
    auto* const em = pm.getExecutionManager();
    const auto active = [&]() { return safety->getMode() == barrett::SafetyModule::ACTIVE; };
    const auto finish = [&](int result) {
        if (active()) {
            std::cout << "Use the pendant to leave Active mode before exiting.\n" << std::flush;
            while (active()) barrett::btsleep(0.05);
        }
        wam.idle();
        return result;
    };

    std::cout << "Activate the WAM at the pendant for gravity compensation.\n" << std::flush;
    safety->waitForMode(barrett::SafetyModule::ACTIVE, true, 0.05);
    wam.gravityCompensate();
    std::cout << "Unloaded hold at q2=" << targetQ2 << " rad and q4=" << targetQ4
              << " rad with all other joint references at zero.\n"
                 "With the tool installed and its path clear of the fixture, type move to start.\n"
              << std::flush;
    std::string input;
    if (!std::getline(std::cin, input) || input != "move" || !active()) return finish(1);

    jp_type target(0.0);
    target[activeJoints[0]] = targetQ2;
    target[activeJoints[1]] = targetQ4;
    wam.moveTo(target, false, 0.15, 0.15);
    while (active() && !wam.moveIsDone()) barrett::btsleep(0.05);
    if (!active()) return finish(1);

    std::cout << "Capturing three seconds of steady hold at the target.\n" << std::flush;
    const auto start = Clock::now();
    std::vector<Sample<DOF>> samples;
    samples.reserve(static_cast<size_t>(sampleDuration / samplePeriod) + 2);
    jp_type firstPosition;
    while (active() && Clock::now() - start < captureTimeout) {
        jp_type q;
        jv_type qd;
        jt_type torque;
        {
            BARRETT_SCOPED_LOCK(em->getMutex());
            q = wam.getJointPositions();
            qd = wam.getJointVelocities();
            torque = wam.getJointTorques();
        }
        const double time = std::chrono::duration<double>(Clock::now() - start).count();
        const bool steady = q.allFinite() && qd.allFinite() && torque.allFinite() &&
            (q - target).cwiseAbs().maxCoeff() <= positionTolerance &&
            qd.cwiseAbs().maxCoeff() <= speedTolerance;
        if (!steady) {
            samples.clear();
        } else {
            if (!samples.empty() && (q - firstPosition).cwiseAbs().maxCoeff() > maxPositionDrift) {
                samples.clear();
            }
            if (samples.empty()) firstPosition = q;
            Sample<DOF> sample;
            sample.time = time;
            for (size_t j = 0; j < DOF; ++j) {
                sample.position[j] = q[j];
                sample.velocity[j] = qd[j];
            }
            for (size_t i = 0; i < activeJoints.size(); ++i) sample.hold[i] = torque[activeJoints[i]];
            samples.push_back(sample);
            if (samples.back().time - samples.front().time >= sampleDuration) break;
        }
        barrett::btsleep(samplePeriod);
    }
    if (!active() || samples.empty() || samples.back().time - samples.front().time < sampleDuration) {
        std::cerr << "The check ended before a full steady hold was recorded.\n";
        return finish(1);
    }
    try {
        report(samples);
    } catch (const std::exception& e) {
        std::cerr << "Cannot save the static hold: " << e.what() << '\n';
        return finish(1);
    }
    return finish(0);
}
