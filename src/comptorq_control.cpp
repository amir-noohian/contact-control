#include "dynamics/dynamics.h"
#include "joint_computed_torque/systems.hpp"
#include "joint_computed_torque/logging.hpp"
#include "joint_computed_torque/log_path.hpp"
#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cerrno>
#include <sys/stat.h>
#include <barrett/os.h>
#include <barrett/detail/stl_utils.h>
#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

bool validate_args(int argc, char** argv) {
    if (argc != 1) {
        std::cout << "Usage: " << argv[0] << "\n"
                  << "Joint computed torque with configured PD gains converted through the inertia matrix.\n"
                  << "30 seconds, 3-second ramps, centered on zero. Tracking CSV saved in data/comptorq_control/.\n";
        return false;
    }
    return true;
}

template<size_t DOF>
int wam_main(int, char**, barrett::ProductManager& pm, barrett::systems::Wam<DOF>& wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    using namespace barrett;
    using namespace joint_computed_torque;
    if (DOF != 4 && DOF != 7) {
        std::cerr << "This controller requires a 4- or 7-DOF WAM.\n";
        return 1;
    }
    for (const char* directory : {JOINT_TRACKING_DATA_DIR,
                                 JOINT_TRACKING_DATA_DIR "/comptorq_control"}) {
        if (mkdir(directory, 0775) != 0 && errno != EEXIST) {
            std::perror("Create tracking directory");
            return 1;
        }
    }
    std::string csvPath;
    try {
        csvPath = reserveTrackingCsv(JOINT_TRACKING_DATA_DIR "/comptorq_control",
                                     "comptorq_tracking");
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    std::ofstream csv(csvPath);
    if (!csv) { std::cerr << "Cannot open tracking CSV.\n"; return 1; }
    char binaryPath[] = "/tmp/comptorq_tracking_XXXXXX";
    const int binaryFd = mkstemp(binaryPath);
    if (binaryFd == -1) { std::perror("Create binary log"); return 1; }
    close(binaryFd);
    std::cout << "Tracking CSV: " << csvPath << " (binary recovery file: " << binaryPath << ")\n";
    pm.getSafetyModule()->setVelocityLimit(1.5);
    wam.gravityCompensate();
    std::cout << "Press Enter to move to the all-zero joint configuration.\n";
    detail::waitForEnter();
    wam.moveTo(jp_type(0.0), true);

    systems::Ramp time(pm.getExecutionManager(), 1.0);
    time.stop(); time.reset();
    SineReference<DOF> reference;
    Dynamics<DOF> dynamics(true);
    AccelerationPD<DOF> feedback;
    DynamicsFeedforward<DOF> controller;
    systems::connect(time.output, reference.timeInput);
    systems::connect(wam.jpOutput, dynamics.jpInputDynamics);
    systems::connect(wam.jvOutput, dynamics.jvInputDynamics);
    systems::connect(reference.position, feedback.desiredPosition);
    systems::connect(wam.jpOutput, feedback.measuredPosition);
    systems::connect(reference.velocity, feedback.desiredVelocity);
    systems::connect(wam.jvOutput, feedback.measuredVelocity);
    systems::connect(reference.acceleration, feedback.desiredAcceleration);
    systems::connect(feedback.acceleration, dynamics.jaInputDynamics);
    systems::connect(reference.position, controller.desiredPosition);
    systems::connect(reference.velocity, controller.desiredVelocity);
    systems::connect(wam.jpOutput, controller.measuredPosition);
    systems::connect(wam.jvOutput, controller.measuredVelocity);
    systems::connect(dynamics.dynamicsFeedFWD, controller.feedforward);
    systems::connect(wam.gravity.output, controller.gravity);

    typedef TrackingLog<DOF> Log;
    typename Log::Grouper samples;
    const double period = pm.getExecutionManager()->getPeriod();
    systems::connect(time.output, samples.template getInput<0>());
    systems::connect(reference.position, samples.template getInput<1>());
    systems::connect(wam.jpOutput, samples.template getInput<2>());
    systems::connect(reference.velocity, samples.template getInput<3>());
    systems::connect(wam.jvOutput, samples.template getInput<4>());
    systems::connect(reference.acceleration, samples.template getInput<5>());
    systems::PeriodicDataLogger<typename Log::Record> logger(
        pm.getExecutionManager(), new log::RealTimeWriter<typename Log::Record>(binaryPath, period), 1);

    std::cout << "Press Enter to start the 30-second joint 2/4 sine trajectory.\n";
    detail::waitForEnter();
    wam.trackReferenceSignal(controller.output);
    systems::connect(samples.output, logger.input);
    time.start();
    while (!reference.done.load() && !controller.fault.load() &&
           !feedback.fault.load() &&
           pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) btsleep(0.01);
    time.stop();
    logger.closeLog();
    bool returnedHome = false;
    if (feedback.fault.load()) {
        std::cerr << "Controller stopped: invalid state in computed-torque feedback.\n";
    } else if (controller.fault.load()) {
        std::cerr << "Controller stopped: tracking error, joint/velocity bound, or invalid input.\n";
    } else if (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
        wam.moveHome();
        returnedHome = pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE;
        std::cout << "Trajectory complete; returned home. Shift-idle to exit.\n";
    }
    if (!returnedHome) wam.idle();
    bool saved = false;
    try {
        Log::exportCsv(binaryPath, csv);
        csv.close();
        saved = !csv.fail();
    } catch (const std::exception& e) {
        std::cerr << "CSV export failed: " << e.what() << '\n';
    }
    if (saved) {
        std::remove(binaryPath);
        std::cout << "Tracking data saved to " << csvPath << '\n';
    } else {
        std::cerr << "Export incomplete; binary log retained at " << binaryPath << '\n';
    }
    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
    wam.idle();
    return (controller.fault.load() || feedback.fault.load() || !saved) ? 1 : 0;
}
