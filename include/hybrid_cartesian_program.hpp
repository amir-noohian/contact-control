#pragma once

#include "hybrid_cartesian_model.hpp"
#include "hybrid_cartesian_dynamics_log.hpp"
#include "hybrid_ft/controller_config.hpp"
#include "hybrid_ft/force_low_pass.hpp"
#include "hybrid_ft/joint_torque.hpp"
#include "joint_computed_torque/log_path.hpp"

#include <barrett/products/product_manager.h>
#include <barrett/systems/wam.h>

#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <string>
#include <sys/stat.h>
#include <system_error>
#include <unistd.h>

namespace hybrid_cartesian_program {
using namespace hybrid_cartesian_dynamics;
using barrett::systems::connect;

struct Settings {
    double distance, duration, motionKp, motionKd, motionKi, motionIntegralLimit, maxAcceleration;
    double forceDesired, forceRampDuration, forceKp, forceKi, integralLimit, cutoffHz, zForceFeedbackLimit;
    double maxForce, maxJointSpeed, holdZeroTolerance;
    std::array<double, 7> target, torqueLimits;
    std::string positionPattern;
    double positionAmplitude, positionPeriod;
};

constexpr std::array<double, 7> defaultTarget{{0.0, 1.164, 0.0, 2.026, 0.0, 0.0, 0.0}};
constexpr double defaultForceRampDuration = 2.0;

struct Program {
    const char* name;
    const char* modelName;
    const char* tuningPath;
    const char* dataDirectory;
};

inline Settings loadSettings(const char* path) {
    libconfig::Config config;
    config.setAutoConvert(true);
    config.readFile(path);
    const auto number = [&](const char* name, double minimum, bool strictlyGreater = false) {
        const double value = config.lookup(name);
        if (!std::isfinite(value) || value < minimum || (strictlyGreater && value == minimum)) {
            throw std::runtime_error(std::string("Invalid setting ") + name);
        }
        return value;
    };
    const double distance = config.lookup("x_distance");
    const double force = config.lookup("z_force");
    const std::string positionPattern = config.exists("position_pattern")
        ? static_cast<const char*>(config.lookup("position_pattern")) : "move";
    if (positionPattern != "move" && positionPattern != "sine") {
        throw std::runtime_error("position_pattern must be move or sine");
    }
    const double positionAmplitude = config.exists("position_amplitude")
        ? number("position_amplitude", 0.0) : 0.2;
    const double positionPeriod = config.exists("position_period")
        ? number("position_period", 0.0, true) : 24.0;
    Settings settings{
        distance, number("duration", 0.0, true),
        number("motion_kp", 0.0), number("motion_kd", 0.0),
        config.exists("motion_ki") ? number("motion_ki", 0.0) : 0.0,
        config.exists("motion_integral_acceleration_limit")
            ? number("motion_integral_acceleration_limit", 0.0) : 6.0,
        number("max_x_acceleration", 0.0, true),
        force, config.exists("force_ramp_duration")
            ? number("force_ramp_duration", 0.0, true) : defaultForceRampDuration,
        number("force_kp", 0.0), number("force_ki", 0.0),
        number("force_integral_limit", 0.0, true), number("force_filter_hz", 0.0),
        config.exists("z_force_feedback_limit") ? number("z_force_feedback_limit", 0.0, true) : 20.0,
        number("max_contact_force", 0.0, true),
        number("max_joint_speed", 0.0, true),
        config.exists("hold_zero_tolerance") ? number("hold_zero_tolerance", 0.0, true) : 0.05,
        config.exists("target_position")
            ? hybrid_ft::ControllerConfig::vector<7>(config, "target_position") : defaultTarget,
        hybrid_ft::ControllerConfig::vector<7>(config, "joint_torque_limits", true),
        positionPattern, positionAmplitude, positionPeriod
    };
    if (!std::isfinite(distance) || !std::isfinite(force) ||
        force >= 0.0 || -force >= settings.maxForce ||
        *std::min_element(settings.torqueLimits.begin(), settings.torqueLimits.end()) <= 0.0) {
        throw std::runtime_error("Invalid x distance, signed z force, contact thresholds, or torque limits");
    }
    for (size_t j = 0; j < settings.target.size(); ++j) {
        if (j != activeJoints[0] && j != activeJoints[1] && settings.target[j] != 0.0) {
            throw std::runtime_error("target_position must be zero on joints other than 2 and 4");
        }
    }
    return settings;
}

struct Fault {
    const char* message = nullptr;
    int joint = 0;
    double speed = 0.0;
};

struct XAccelerationLimit {
    bool reached = false;
    double referenceTravel = 0.0;
    double measuredTravel = 0.0;
    double requestedAcceleration = 0.0;
    double appliedAcceleration = 0.0;
    double measuredXForce = 0.0;
    double measuredZForce = 0.0;
    double commandedZForce = 0.0;
    std::array<double, 2> jointTorques{{0.0, 0.0}};
};

template <size_t DOF, template <size_t> class Model>
class Controller : public barrett::systems::System, public barrett::systems::KinematicsInput<DOF> {
public:
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    using ForceVector = hybrid_ft::ForceLowPassState::vector_type;

    Input<jp_type> jointPosition;
    Input<jv_type> jointVelocity;
    Input<cp_type> toolPosition;
    Input<cv_type> toolVelocity;
    Input<jt_type> barrettGravity;
    Output<jt_type> output;

    Controller(const libconfig::Setting& kinematics, const libconfig::Setting& robot,
               barrett::ForceTorqueSensor& sensor, double period, const v_type& holdKp,
               const v_type& holdKd, LogBuffer& log, const Settings& tuning)
        : System("CartesianHybridDynamics"), barrett::systems::KinematicsInput<DOF>(this),
          jointPosition(this), jointVelocity(this), toolPosition(this), toolVelocity(this),
          barrettGravity(this), output(this, &outputValue_), sensor_(sensor), model_(kinematics, robot),
          period_(period), holdKp_(holdKp), holdKd_(holdKd), log_(log), settings(tuning) {}

    ~Controller() override { this->mandatoryCleanUp(); }

    // The command loop calls these methods with the execution-manager mutex held.
    void invalidateTarget() {
        targetReady_ = false;
        stop();
    }

    void tareAtTarget() {
        sensor_.tare();
        if (!toolPosition.valueDefined() || !std::isfinite(toolPosition.getValue()[0])) {
            throw std::runtime_error("Tool position is unavailable at the target");
        }
        targetX_ = toolPosition.getValue()[0];
        filter_.reset();
        targetReady_ = true;
    }

    const char* start() {
        if (active_) {
            return "Contact controller already active.";
        }
        if (!targetReady_) {
            return "Use t to reach and capture the target first.";
        }
        if (!jointPosition.valueDefined() || !jointVelocity.valueDefined() || !toolPosition.valueDefined()) {
            return "WAM state is unavailable.";
        }
        const auto& position = jointPosition.getValue();
        const auto& velocity = jointVelocity.getValue();
        startX_ = toolPosition.getValue()[0];
        if (!position.allFinite() || !velocity.allFinite() || !std::isfinite(startX_)) {
            return "Invalid joint or tool position.";
        }
        if (!heldJointsNearZero(position, settings.holdZeroTolerance)) {
            return "Contact requires the held joints to be within hold_zero_tolerance.";
        }
        if (velocity.cwiseAbs().maxCoeff() > settings.maxJointSpeed) {
            return "Contact requires joint speeds within max_joint_speed.";
        }
        xTravel_ = targetX_ + (settings.positionPattern == "sine" ? 0.0 : settings.distance) - startX_;
        if (!std::isfinite(xTravel_)) {
            return "Invalid x trajectory.";
        }
        integral_ = 0.0;
        xIntegralAcceleration_ = 0.0;
        filter_.reset();
        fault_ = {};
        xLimit_ = {};
        log_.reset();
        contactCycle_ = 0;
        active_ = true;
        return nullptr;
    }

    void stop() { active_ = false; }
    bool active() const { return active_; }
    Fault fault() const { return fault_; }
    XAccelerationLimit xAccelerationLimit() const { return xLimit_; }

private:
    void invalidateOutputs() override {
        if (active_) {
            fail("WAM state became unavailable");
        }
        System::invalidateOutputs();
    }

    void operate() override {
        torque_.setZero();
        outputValue_->setData(&torque_);
        if (!active_) {
            return;
        }

        try {
            sensor_.update(true);
        } catch (const std::exception&) {
            fail("F/T read failed");
            return;
        }
        const auto& kinematics = this->kinInput.getValue();
        const barrett::math::Matrix<3, 3> rotation(kinematics.impl->tool->rot_to_world);
        const ForceVector rawForce = -(rotation * sensor_.getForce());
        if (!rawForce.allFinite()) {
            fail("Invalid F/T sample");
            return;
        }
        if (rawForce.norm() > settings.maxForce) {
            fail("Measured force limit exceeded during contact control");
            return;
        }
        const double filteredZForce = filter_.update(rawForce, settings.cutoffHz, period_)[2];
        const double measuredZForce = std::max(-settings.zForceFeedbackLimit,
            std::min(settings.zForceFeedbackLimit, filteredZForce));

        const auto& q = jointPosition.getValue();
        const auto& qd = jointVelocity.getValue();
        const auto& gravity = barrettGravity.getValue();
        const double x = toolPosition.getValue()[0];
        const double xd = toolVelocity.getValue()[0];
        if (!q.allFinite() || !qd.allFinite() || !gravity.allFinite() ||
            !std::isfinite(x) || !std::isfinite(xd)) {
            fail("Invalid joint or tool state during contact control");
            return;
        }
        if (!checkContactJointSpeed(qd)) {
            return;
        }
        if (!heldJointsNearZero(q, settings.holdZeroTolerance)) {
            fail("A held joint exceeded hold_zero_tolerance");
            return;
        }

        const double time = contactCycle_ * period_;
        const ContactReference reference = contactReference(time, settings.forceRampDuration,
            settings.duration, xTravel_, settings.forceDesired,
            settings.positionPattern == "sine", settings.positionAmplitude, settings.positionPeriod);
        const double xReference = startX_ + reference.xOffset;
        const double xError = xReference - x;
        const double xdError = reference.xVelocity - xd;
        const double pdAcceleration = reference.xAcceleration
                                    + settings.motionKp * xError + settings.motionKd * xdError;
        if (reference.finalHold) {
            xIntegralAcceleration_ = updateMotionIntegral(xIntegralAcceleration_,
                settings.motionKi * period_ * xError, pdAcceleration,
                settings.motionIntegralLimit, settings.maxAcceleration);
        }
        const double acceleration = pdAcceleration + xIntegralAcceleration_;
        const double xAcceleration = std::max(-settings.maxAcceleration,
                                              std::min(settings.maxAcceleration, acceleration));

        const double forceError = reference.zForce - measuredZForce;
        const double nextIntegral = std::max(-settings.integralLimit,
            std::min(settings.integralLimit, integral_ + period_ * forceError));
        const double forceWithoutIntegral = reference.zForce + settings.forceKp * forceError;
        const double trialForce = forceWithoutIntegral + settings.forceKi * nextIntegral;
        if (!(trialForce < -settings.maxForce && forceError < 0.0) &&
            !(trialForce > 0.0 && forceError > 0.0)) {
            integral_ = nextIntegral;
        }
        const double zForce = std::max(-settings.maxForce,
            std::min(0.0, forceWithoutIntegral + settings.forceKi * integral_));

        PlanarState dynamics = model_.evaluate(kinematics, q, qd);
        model_.addWamGravity(dynamics, gravity);
        Eigen::Vector2d force;
        if (!std::isfinite(acceleration) ||
            !solveCartesianForce(dynamics, xAcceleration, zForce, force)) {
            fail("Invalid dynamics or near-singular x/z Jacobian");
            return;
        }
        ForceVector taskForce;
        taskForce << force[0], 0.0, force[1];
        const barrett::math::Matrix<6, DOF> jacobian(kinematics.impl->tool_jacobian);
        const jt_type taskAndHoldTorque = hybrid_ft::BaseControlToJointTorque<DOF>::calculateTorque(
            jacobian, taskForce, q, qd, jp_type(0.0), holdKp_, holdKd_);
        const jt_type command = model_.commandTorque(taskAndHoldTorque, gravity);

        // Check the torque after the WAM adds its downstream gravity signal.
        const jt_type totalTorque = command + gravity;
        for (size_t j = 0; j < DOF; ++j) {
            if (!std::isfinite(totalTorque[j]) || std::abs(totalTorque[j]) > settings.torqueLimits[j]) {
                fail("Total modeled joint-torque limit exceeded");
                return;
            }
        }
        torque_ = command;
        const auto j2 = activeJoints[0], j4 = activeJoints[1];
        log_.record({{
            time, xReference, x, reference.xVelocity, xd,
            rawForce[0], rawForce[1], rawForce[2], filteredZForce, measuredZForce,
            reference.zForce, zForce, force[0], force[1], acceleration, xAcceleration,
            q[j2], q[j4], qd[j2], qd[j4], totalTorque[j2], totalTorque[j4], xIntegralAcceleration_
        }});
        if (!xLimit_.reached && acceleration != xAcceleration) {
            xLimit_ = {true, reference.xOffset, x - startX_, acceleration, xAcceleration,
                rawForce[0], rawForce[2], zForce, {{totalTorque[activeJoints[0]], totalTorque[activeJoints[1]]}}};
        }
        ++contactCycle_;
    }

    bool checkContactJointSpeed(const jv_type& velocity) {
        Eigen::Index joint;
        const double speed = velocity.cwiseAbs().maxCoeff(&joint);
        if (speed <= settings.maxJointSpeed) {
            return true;
        }
        fail("Joint-speed limit exceeded during contact control", static_cast<int>(joint) + 1, speed);
        return false;
    }

    void fail(const char* message, int joint = 0, double speed = 0.0) {
        fault_ = {message, joint, speed};
        stop();
    }

    typename Output<jt_type>::Value* outputValue_;
    barrett::ForceTorqueSensor& sensor_;
    Model<DOF> model_;
    double period_;
    v_type holdKp_, holdKd_;
    LogBuffer& log_;
    const Settings& settings;
    std::size_t contactCycle_ = 0;
    jt_type torque_;
    hybrid_ft::ForceLowPassState filter_;
    double targetX_ = 0.0, startX_ = 0.0, xTravel_ = 0.0, integral_ = 0.0;
    double xIntegralAcceleration_ = 0.0;
    bool targetReady_ = false, active_ = false;
    Fault fault_;
    XAccelerationLimit xLimit_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline bool validateArgs(int argc, char** argv, const Program& program, Settings& settings) {
    const std::string usage = std::string("Usage: ") + program.name + " [--check-config] [tuning.conf]";
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << usage << '\n';
        std::exit(0);
    }
    const bool checkOnly = argc >= 2 && std::string(argv[1]) == "--check-config";
    const int pathIndex = checkOnly ? 2 : 1;
    if (argc > pathIndex + 1) {
        std::cerr << usage << '\n';
        return false;
    }
    const char* path = argc > pathIndex ? argv[pathIndex] : program.tuningPath;
    try {
        settings = loadSettings(path);
    } catch (const libconfig::ParseException& e) {
        std::cerr << path << ':' << e.getLine() << ' ' << e.getError() << '\n';
        return false;
    } catch (const libconfig::SettingException& e) {
        std::cerr << "Invalid or missing setting in " << path << ": " << e.getPath() << '\n';
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Cannot load " << path << ": " << e.what() << '\n';
        return false;
    }
    std::cout << "Dynamics model: " << program.modelName << '\n'
              << "Tuning config: " << path << '\n'
              << "X motion tuning: motion_kp=" << settings.motionKp
              << " 1/s^2, motion_kd=" << settings.motionKd
              << " 1/s, motion_ki=" << settings.motionKi
              << " 1/s^3, motion_integral_acceleration_limit=" << settings.motionIntegralLimit
              << " m/s^2, max_x_acceleration=" << settings.maxAcceleration
              << " m/s^2\n"
              << "X position pattern: " << settings.positionPattern;
    if (settings.positionPattern == "sine") {
        std::cout << ", amplitude=" << settings.positionAmplitude
                  << " m, period=" << settings.positionPeriod << " s";
    }
    std::cout << std::endl;
    if (checkOnly) {
        std::exit(0);
    }
    return true;
}

template <template <size_t> class Model, size_t DOF>
int runController(barrett::ProductManager& pm, barrett::systems::Wam<DOF>& wam,
                  const Program& program, const Settings& settings) {
    using ContactController = Controller<DOF, Model>;
    if (DOF < 4 || DOF > 7 || !pm.foundForceTorqueSensor()) {
        std::cerr << "Requires a 4- or 7-DOF WAM with a Barrett F/T sensor.\n";
        return 1;
    }
    auto* const em = pm.getExecutionManager();
    auto* const safety = pm.getSafetyModule();
    auto& robotConfig = pm.getConfig();
    robotConfig.setAutoConvert(true);
    const auto& robot = robotConfig.lookup(pm.getWamDefaultConfigPath());
    const auto& kinematics = robot["kinematics"];
    // Force rotation and task kinematics share base axes for this configuration.
    const auto& transform = kinematics["world_to_base"];
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            const double value = transform[row][col];
            if (!std::isfinite(value) || std::abs(value - double(row == col)) > 1e-9) {
                std::cerr << "The Cartesian controller requires aligned world and base axes.\n";
                return 1;
            }
        }
    }
    const double period = em->getPeriod();
    if (!std::isfinite(period) || period <= 0.0) {
        std::cerr << "Invalid WAM execution period.\n";
        return 1;
    }
    LogBuffer log(1024);
    ContactController controller(kinematics, robot, *pm.getForceTorqueSensor(), period,
                                 wam.jpController.getKp(), wam.jpController.getKd(), log, settings);
    connect(wam.jpOutput, controller.jointPosition);
    connect(wam.jvOutput, controller.jointVelocity);
    connect(wam.toolPosition.output, controller.toolPosition);
    connect(wam.toolVelocity.output, controller.toolVelocity);
    connect(wam.gravity.output, controller.barrettGravity);
    connect(wam.kinematicsBase.kinOutput, controller.kinInput);
    em->startManaging(controller);

    std::cout << "Gravity compensation starts at pendant activation.\n";
    safety->waitForMode(barrett::SafetyModule::ACTIVE, true, 0.05);
    wam.gravityCompensate();
    std::cout << "\nCommands:\n"
                 "  t : move to target and zero F/T sensor\n"
                 "  c : start force ramp and configured x trajectory (t required first)\n"
                 "  x : stop contact controller\n"
                 "  ENTER : move home\n"
                 "  q : quit\n";

    typename ContactController::jp_type target;
    for (size_t j = 0; j < DOF; ++j) {
        target[j] = settings.target[j];
    }

    const auto waitForPositionMove = [&]() {
        while (safety->getMode() == barrett::SafetyModule::ACTIVE) {
            if (wam.moveIsDone()) {
                return true;
            }
            barrett::btsleep(0.01);
        }
        wam.idle();
        return false;
    };

    const char* reportedFault = nullptr;
    bool reportedXLimit = false;
    typename ContactController::jt_type torqueBeforeContact(0.0);
    std::string csvPath;
    std::ofstream csv;
    const auto openLog = [&]() {
        const std::string logDir = std::string(program.dataDirectory) + "/" + program.name;
        try {
            for (const char* dir : {program.dataDirectory, logDir.c_str()}) {
                if (mkdir(dir, 0775) != 0 && errno != EEXIST) {
                    throw std::system_error(errno, std::generic_category(), std::string("Create ") + dir);
                }
            }
            csvPath = joint_computed_torque::reserveTrackingCsv(logDir, std::string(program.name) + "_datalog");
            csv.open(csvPath);
            if (!csv) {
                throw std::runtime_error("Cannot open " + csvPath);
            }
            csv << std::setprecision(16);
            writeLogRow(csv, logColumns);
            std::cout << "CSV log: " << csvPath << '\n';
        } catch (const std::exception& e) {
            std::cerr << "Contact logging unavailable: " << e.what() << '\n';
        }
    };
    const auto flushLog = [&](bool finish) {
        if (!csv.is_open()) {
            return;
        }
        std::size_t dropped;
        {
            BARRETT_SCOPED_LOCK(em->getMutex());
            log.swapPending();
            dropped = log.dropped();
        }
        log.writeBatch(csv);
        if (finish) {
            csv.close();
        } else {
            csv.flush();
        }
        if (!csv) {
            std::cerr << "Error writing contact log: " << csvPath << '\n';
            if (csv.is_open()) {
                csv.close();
            }
        } else if (finish) {
            std::cout << "Contact data saved to " << csvPath << '\n';
            if (dropped != 0) {
                std::cerr << "Contact log dropped " << dropped << " samples while its buffer was full.\n";
            }
        }
    };
    const auto handleCommand = [&](const std::string& line) {
        if (line == "t" || line.empty()) {
            {
                BARRETT_SCOPED_LOCK(em->getMutex());
                if (controller.active()) {
                    wam.idle();
                }
                controller.invalidateTarget();
            }
            flushLog(true);
            if (line.empty()) {
                std::cout << "Returning home..." << std::endl;
                wam.moveHome(false);
                if (waitForPositionMove()) {
                    std::cout << "At home." << std::endl;
                }
                return;
            }

            std::cout << "Moving to target..." << std::endl;
            wam.moveTo(target, false);
            if (!waitForPositionMove()) {
                return;
            }
            std::cout << "At target. Zeroing F/T sensor..." << std::endl;
            try {
                {
                    BARRETT_SCOPED_LOCK(em->getMutex());
                    controller.tareAtTarget();
                }
                std::cout << "F/T sensor zeroed." << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Cannot prepare the target: " << e.what() << '\n';
            }
        } else if (line == "c") {
            if (csv.is_open()) {
                bool wasActive;
                {
                    BARRETT_SCOPED_LOCK(em->getMutex());
                    wasActive = controller.active();
                }
                if (wasActive) {
                    std::cout << "Contact controller already active.\n";
                    return;
                }
                flushLog(true);
            }
            const char* rejection;
            {
                BARRETT_SCOPED_LOCK(em->getMutex());
                rejection = controller.start();
                if (!rejection) {
                    torqueBeforeContact = wam.getJointTorques();
                    wam.trackReferenceSignal(controller.output);
                }
            }
            if (!rejection) {
                reportedFault = nullptr;
                reportedXLimit = false;
                std::cout << "Starting contact controller. Ramping z force over "
                          << settings.forceRampDuration << " s before x travel." << std::endl;
                openLog();
            } else {
                std::cout << rejection << '\n';
            }
        } else if (line == "x") {
            bool wasActive;
            {
                BARRETT_SCOPED_LOCK(em->getMutex());
                wasActive = controller.active();
                if (wasActive) {
                    controller.stop();
                    wam.idle();
                }
            }
            std::cout << (wasActive ? "Stopping contact controller..." : "Contact controller not active.") << '\n';
            flushLog(true);
        } else {
            std::cout << "Unknown command.\n";
        }
    };

    // Monitor the pendant and report latched faults even while the terminal is idle.
    std::string line;
    bool quitting = false;
    std::cout << "\nEnter command: " << std::flush;
    while (!quitting && safety->getMode() == barrett::SafetyModule::ACTIVE) {
        Fault fault;
        XAccelerationLimit xLimit;
        typename ContactController::jt_type latestWamTorque(0.0);
        bool hasLatestWamTorque = false;
        {
            BARRETT_SCOPED_LOCK(em->getMutex());
            fault = controller.fault();
            xLimit = controller.xAccelerationLimit();
            if (xLimit.reached && !reportedXLimit && controller.active()) {
                latestWamTorque = wam.getJointTorques();
                hasLatestWamTorque = latestWamTorque.allFinite();
            }
        }
        if (fault.message && fault.message != reportedFault) {
            std::cerr << "\nTask stopped: " << fault.message;
            if (fault.joint != 0) {
                std::cerr << " (joint " << fault.joint << ": " << std::setprecision(8) << fault.speed
                          << " rad/s, limit " << settings.maxJointSpeed << " rad/s)";
            }
            std::cerr << "\nEnter command: ";
            reportedFault = fault.message;
        }
        flushLog(fault.message != nullptr);
        if (xLimit.reached && !reportedXLimit) {
            std::cout << "\nX acceleration capped at " << xLimit.appliedAcceleration
                      << " m/s^2 (requested " << xLimit.requestedAcceleration << ")."
                      << " X travel: reference " << 1000.0 * xLimit.referenceTravel
                      << " mm, measured " << 1000.0 * xLimit.measuredTravel << " mm."
                      << " Measured x force: " << xLimit.measuredXForce << " N."
                      << " Z force: command " << xLimit.commandedZForce
                      << " N, measured " << xLimit.measuredZForce << " N.\n"
                      << "Joint 2/4 torque commands (Nm): before c "
                      << torqueBeforeContact[activeJoints[0]] << ", " << torqueBeforeContact[activeJoints[1]]
                      << "; controller at cap " << xLimit.jointTorques[0] << ", " << xLimit.jointTorques[1];
            if (hasLatestWamTorque) {
                std::cout << "; latest WAM "
                          << latestWamTorque[activeJoints[0]] << ", " << latestWamTorque[activeJoints[1]];
            }
            std::cout << "\nEnter command: " << std::flush;
            reportedXLimit = true;
        }
        pollfd terminal{STDIN_FILENO, POLLIN, 0};
        const int ready = poll(&terminal, 1, 50);
        if (ready < 0 && errno != EINTR) {
            break;
        }
        if (ready <= 0) {
            continue;
        }
        char buffer[64];
        const ssize_t size = read(STDIN_FILENO, buffer, sizeof(buffer));
        if (size < 0 && errno == EINTR) {
            continue;
        }
        if (size <= 0 || safety->getMode() != barrett::SafetyModule::ACTIVE) {
            break;
        }
        for (ssize_t i = 0; i < size; ++i) {
            if (buffer[i] == '\n') {
                if (line == "q") {
                    std::cout << "Exiting..." << std::endl;
                    quitting = true;
                    break;
                }
                handleCommand(line);
                line.clear();
                if (safety->getMode() != barrett::SafetyModule::ACTIVE) {
                    quitting = true;
                    break;
                }
                std::cout << "\nEnter command: " << std::flush;
            } else if (line.size() < 2) {
                line += buffer[i];
            }
        }
    }
    {
        BARRETT_SCOPED_LOCK(em->getMutex());
        if (controller.active()) {
            controller.stop();
            wam.idle();
        }
    }
    em->stopManaging(controller);
    flushLog(true);
    if (safety->getMode() == barrett::SafetyModule::ACTIVE) {
        std::cout << "Waiting for the pendant to leave Active mode.\n";
        while (safety->getMode() == barrett::SafetyModule::ACTIVE) {
            barrett::btsleep(0.05);
        }
    }
    return 0;
}
} // namespace hybrid_cartesian_program
