#include "external_torque.hpp"
#include "dynamics.hpp"
#include "dynamic_external_torque.hpp"
#include "dynamic_external_wrench.hpp"
#include "external_wrench.hpp"
#include "jacobian.hpp"
#include "utilities.hpp"
#include "bt_transformation.hpp"
#include "task_space_controller.hpp"
#include "tj_transformation.hpp"
#include "hybrid_task_csv_logger.hpp"
#include "sample_delay.hpp"
#include "jointtorque.hpp"

#include <barrett/systems/tuple_grouper.h>
#include <barrett/systems.h>
#include <barrett/units.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <barrett/detail/ca_macro.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <limits>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

namespace {

/** Ensures data dir exists; path hybrid_task_datalog_NNNN.csv (NNNN = 1 + max existing .csv). */
std::string nextHybridTaskLogPath()
{
    const char* dataDir = "../../data";
    const char* prefix = "hybrid_task_datalog_";
    mkdir(dataDir, 0755);

    int maxN = 0;
    DIR* d = opendir(dataDir);
    if (d) {
        const size_t prefixLen = std::strlen(prefix);
        while (struct dirent* e = readdir(d)) {
            const char* name = e->d_name;
            if (std::strncmp(name, prefix, prefixLen) != 0) {
                continue;
            }
            char* endp = NULL;
            long v = std::strtol(name + prefixLen, &endp, 10);
            if (endp == name + prefixLen) {
                continue;
            }
            if (std::strcmp(endp, ".csv") != 0) {
                continue;
            }
            if (v > maxN) {
                maxN = static_cast<int>(v);
            }
        }
        closedir(d);
    }

    const int nextN = maxN + 1;
    char buf[512];
    std::snprintf(buf, sizeof(buf), "%s/%s%04d.csv", dataDir, prefix, nextN);
    return std::string(buf);
}

}  // namespace

template<size_t DOF>
int wam_main(int argc, char** argv, barrett::ProductManager& pm, barrett::systems::Wam<DOF>& wam)
{
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    typedef barrett::math::Matrix<6, DOF> jacobian_type;
    typedef typename barrett::math::Vector<3>::type task_vector_force_type;
    typedef typename barrett::math::Vector<3>::type task_vector_velocity_type;
    typedef typename barrett::math::Vector<3>::type task_control_type;

    // default gains
    v_type default_p_gains = wam.jpController.getKp();
    v_type default_d_gains = wam.jpController.getKd();
    v_type default_i_gains = wam.jpController.getKi();

    // contact gains
    v_type contact_p_gains = default_p_gains * 0.0;
    v_type contact_d_gains = default_d_gains * 0.0;
    v_type contact_i_gains = default_i_gains * 0.0;




    // helper lambdas
    auto setDefaultGains = [&]() {
        wam.jpController.setKp(default_p_gains);
        wam.jpController.setKd(default_d_gains);
        wam.jpController.setKi(default_i_gains);
    };

    auto setContactGains = [&]() {
        wam.jpController.setKp(contact_p_gains);
        wam.jpController.setKd(contact_d_gains);
        wam.jpController.setKi(contact_i_gains);
    };


    ExternalTorque<DOF> externalTorque(pm.getExecutionManager());
    ExternalWrench<DOF> externalWrench(pm.getExecutionManager());
    Dynamics<DOF> dynamics(pm.getExecutionManager());
    DynamicExternalTorque<DOF> dynamicExternalTorque(pm.getExecutionManager());
    DynamicExternalWrench<DOF> dynamicExternalWrench(pm.getExecutionManager());
    ToolJacobianOutput<DOF> toolJacobian(wam, pm.getExecutionManager());
    LinearCartesianVelocity<DOF> linearCartesianVelocity(pm.getExecutionManager());
    BaseToTaskTransform<DOF> basetoTaskTransform(pm.getExecutionManager());
    BaseToTaskForce<DOF> basetoTaskForce(pm.getExecutionManager());
    BaseToTaskVelocity<DOF> basetoTaskVelocity(pm.getExecutionManager());
    HybridForceVelocityControl<DOF> hybridControl(pm.getExecutionManager());
    TaskToBaseControl<DOF> taskToBaseTransform(pm.getExecutionManager());
    BaseControlToJointTorque<DOF> controlToJoint(pm.getExecutionManager());
    JointTorqueWithCompensation<DOF> torqueSum(pm.getExecutionManager());
    JointTorqueOutput<DOF> jointTorqueOut(wam, pm.getExecutionManager());
    barrett::systems::Summer<jt_type, 2> customjtSum;
    pm.getExecutionManager()->startManaging(customjtSum);

    // acceleration
    double h_omega_p = 25.0;
    barrett::systems::FirstOrderFilter<jv_type> hp1;
    hp1.setHighPass(jv_type(h_omega_p), jv_type(h_omega_p));
    barrett::systems::Gain<jv_type, double, ja_type> jaWAM(1.0);
    pm.getExecutionManager()->startManaging(hp1);

    barrett::systems::FirstOrderFilter<ja_type> jaFilter;
    ja_type l_omega_p = ja_type::Constant(50.0);
    jaFilter.setLowPass(l_omega_p);
    pm.getExecutionManager()->startManaging(jaFilter);



    barrett::systems::PrintToStream<cf_type> printCartesianForce(pm.getExecutionManager(), "cartesianForce: ");
    barrett::systems::PrintToStream<cf_type> printdynamicCartesianForce(pm.getExecutionManager(), "dynamicCartesianForce: ");
    barrett::systems::PrintToStream<cp_type> printCartesianPosition(pm.getExecutionManager(), "cartesianPosition: ");
    barrett::systems::PrintToStream<cv_type> printCartesianVelocity(pm.getExecutionManager(), "cartesianVelocity: ");
    barrett::systems::PrintToStream<jacobian_type> printJacobian(pm.getExecutionManager(), "Jacobian: ");
    barrett::systems::PrintToStream<jt_type> printJointTorque(pm.getExecutionManager(), "externalJointTorque: ");
    barrett::systems::PrintToStream<cv_type> printCartesianVelocityJacobian(pm.getExecutionManager(), "cartesianVelocityJacobian: ");
    barrett::systems::PrintToStream<task_vector_force_type> printTaskForce(pm.getExecutionManager(), "toolForce: ");
    barrett::systems::PrintToStream<task_vector_velocity_type> printTaskVelocity(pm.getExecutionManager(), "toolVelocity: ");
    barrett::systems::PrintToStream<task_control_type> printTaskControl(pm.getExecutionManager(), "taskControl: ");
    barrett::systems::PrintToStream<jt_type> printJointCommand(pm.getExecutionManager(), "jointCommand: ");
    barrett::systems::PrintToStream<jt_type> printcustomjtSum(pm.getExecutionManager(), "customjtSum: ");
    barrett::systems::PrintToStream<jt_type> printjtSum(pm.getExecutionManager(), "jtSum: ");
    barrett::systems::PrintToStream<jt_type> printGravity(pm.getExecutionManager(), "gravity: ");
    barrett::systems::PrintToStream<jt_type> printTorque(pm.getExecutionManager(), "jointTorque: ");
    barrett::systems::PrintToStream<jt_type> printSC(pm.getExecutionManager(), "supervisoryController: ");
    barrett::systems::PrintToStream<jt_type> printDynamics(pm.getExecutionManager(), "dynamics: ");

    // delay for force
    task_vector_force_type zeroTaskForce;
    zeroTaskForce.setZero();
    SampleDelay<task_vector_force_type> delayedTaskForce(pm.getExecutionManager(), zeroTaskForce, "DelayedTaskForce");




    // acceleration
    barrett::systems::connect(wam.jvOutput, hp1.input);
    barrett::systems::connect(hp1.output, jaWAM.input);
    barrett::systems::connect(jaWAM.output, jaFilter.input);

    // dynamics 
    barrett::systems::connect(wam.jpOutput, dynamics.jpInputDynamics);
    barrett::systems::connect(wam.jvOutput, dynamics.jvInputDynamics);
    barrett::systems::connect(jaFilter.output, dynamics.jaInputDynamics);

    // dynamic external torque
    barrett::systems::connect(dynamics.dynamicsFeedFWD, dynamicExternalTorque.wamDynamicsIn);
    barrett::systems::connect(wam.jtSum.output, dynamicExternalTorque.wamTorqueSumIn);

    // dynamics external force and momentum
    barrett::systems::connect(dynamicExternalTorque.wamExternalTorqueOut, dynamicExternalWrench.dynamicExternalTorqueIn);
    barrett::systems::connect(toolJacobian.output, dynamicExternalWrench.jacobianIn);

    // jtsum for external torque
    barrett::systems::connect(wam.gravity.output, customjtSum.getInput(0));
    // barrett::systems::connect(controlToJoint.jointTorqueOut, customjtSum.getInput(1)); // it is not good, as when the robot goes through singularity, the estimations become Nan.
    barrett::systems::connect(wam.supervisoryController.output, customjtSum.getInput(1)); // it seems that switching from PID joitn position controoler to Cartesian force/motion controller adds noise in force estimation

    // external torque
    barrett::systems::connect(wam.gravity.output, externalTorque.wamGravityIn);
    barrett::systems::connect(customjtSum.output, externalTorque.wamTorqueSumIn);

    // external force and momentum
    barrett::systems::connect(externalTorque.wamExternalTorqueOut, externalWrench.externalTorqueIn);
    barrett::systems::connect(toolJacobian.output, externalWrench.jacobianIn);

    // linear velocity from jacobian
    barrett::systems::connect(toolJacobian.output, linearCartesianVelocity.jacobianIn);
    barrett::systems::connect(wam.jvOutput, linearCartesianVelocity.jointVelocityIn);

    // base to task space transformation for force
    barrett::systems::connect(externalWrench.forceOut, basetoTaskForce.forceBaseIn);
    barrett::systems::connect(basetoTaskTransform.baseToTaskOut, basetoTaskForce.baseToTaskIn);

    // base to task space transformation for velocity
    barrett::systems::connect(wam.toolVelocity.output, basetoTaskVelocity.velocityBaseIn);
    barrett::systems::connect(basetoTaskTransform.baseToTaskOut, basetoTaskVelocity.baseToTaskIn);

    // task space controller (Kv, Kd, Kf, Ki defaults in task_space_controller.hpp)

    task_vector_force_type desiredForce;
    desiredForce.setZero();
    desiredForce[2] = -5.0;

    // Smooth straight-line motion in task frame: sinusoidal velocity on axis 0
    const int velTrajAxis = 0;
    const double velTrajPeakM_s = 0.2;
    const double velTrajPeriod_s = 6.0;
    TaskVelocitySinusoid<task_vector_velocity_type> desiredVelocityTraj(
        pm.getExecutionManager(), velTrajAxis, velTrajPeakM_s, velTrajPeriod_s,
        "DesiredVelTraj");

    task_vector_force_type force_limit;
    force_limit[0] = 5.0;  // x
    force_limit[1] = 5.0;  // y
    force_limit[2] = 7.0;  // z (larger for contact direction)
    ForceClamp<DOF> forceClamp(pm.getExecutionManager(), force_limit);

    ConstantOutput<task_vector_force_type> desiredForceSource(pm.getExecutionManager(), desiredForce, "DesiredForce");

    barrett::systems::connect(desiredVelocityTraj.output, hybridControl.desiredVelocityIn);
    barrett::systems::connect(desiredForceSource.output, hybridControl.desiredForceIn);
    barrett::systems::connect(basetoTaskVelocity.velocityTaskOut, hybridControl.currentVelocityIn);
    barrett::systems::connect(basetoTaskForce.forceTaskOut, forceClamp.input);
    barrett::systems::connect(forceClamp.output, hybridControl.currentForceIn);
    // barrett::systems::connect(basetoTaskForce.forceTaskOut, hybridControl.currentForceIn);
    // barrett::systems::connect(delayedTaskForce.output, hybridControl.currentForceIn);


    // joint space command
    barrett::systems::connect(hybridControl.controlOutput, taskToBaseTransform.controlTaskIn);
    barrett::systems::connect(basetoTaskTransform.baseToTaskOut, taskToBaseTransform.baseToTaskIn);
    barrett::systems::connect(taskToBaseTransform.controlBaseOut, controlToJoint.controlBaseIn);
    barrett::systems::connect(toolJacobian.output, controlToJoint.jacobianIn);

    // joint space command with dynamics compensation
    barrett::systems::connect(controlToJoint.jointTorqueOut, torqueSum.controlTorqueIn);
    barrett::systems::connect(dynamics.dynamicsFeedFWD, torqueSum.dynamicsIn);
    barrett::systems::connect(wam.gravity.output, torqueSum.gravityIn);


    // printing outputs
    barrett::systems::connect(externalWrench.forceOut, printCartesianForce.input);
    // barrett::systems::connect(dynamicExternalWrench.forceOut, printdynamicCartesianForce.input);
    // barrett::systems::connect(wam.toolPosition.output, printCartesianPosition.input);
    // barrett::systems::connect(wam.toolVelocity.output, printCartesianVelocity.input);
    // barrett::systems::connect(toolJacobian.output, printJacobian.input);
    barrett::systems::connect(externalTorque.wamExternalTorqueOut, printJointTorque.input);
    // barrett::systems::connect(linearCartesianVelocity.linearVelocityOut, printCartesianVelocityJacobian.input);
    // barrett::systems::connect(basetoTaskForce.forceTaskOut, printTaskForce.input);
    // barrett::systems::connect(basetoTaskVelocity.velocityTaskOut, printTaskVelocity.input);
    // barrett::systems::connect(hybridControl.controlOutput, printTaskControl.input);
    barrett::systems::connect(controlToJoint.jointTorqueOut, printJointCommand.input);
    barrett::systems::connect(customjtSum.output, printcustomjtSum.input);
    // barrett::systems::connect(wam.jtSum.output, printjtSum.input);
    // barrett::systems::connect(wam.gravity.output, printGravity.input);
    // barrett::systems::connect(jointTorqueOut.output, printTorque.input);
    // barrett::systems::connect(wam.supervisoryController.output, printSC.input);
    // barrett::systems::connect(dynamics.dynamicsFeedFWD, printDynamics.input);


    jp_type target;
    target.setZero();

    // Example only. Replace with a safe pose for your robot.
    // For a 4-DOF WAM, set only the first 4 entries.
    if (DOF >= 1) { target[0] =  0.0; } //
    if (DOF >= 2) { target[1] =  1.21; } // whiteboard 1.21 // 1.12 kitchen scale
    if (DOF >= 3) { target[2] =  0.0; }
    if (DOF >= 4) { target[3] =  1.90; } // whiteboard 1.90 // 1.12 kitchen scale
    if (DOF >= 5) { target[4] =  0.0; }
    if (DOF >= 6) { target[5] =  0.0; }
    if (DOF >= 7) { target[6] =  0.0; }

    //datalog
    typedef barrett::systems::TupleGrouper<
    task_vector_velocity_type,
    task_vector_velocity_type,
    task_vector_force_type,
    task_vector_force_type> hybrid_task_log_grouper_t;
    hybrid_task_log_grouper_t hybridTaskLogGroup("HybridTaskLogGroup");
    pm.getExecutionManager()->startManaging(hybridTaskLogGroup);
    typedef hybrid_task_log_grouper_t::tuple_type hybrid_task_log_record_t;

    barrett::systems::connect(desiredVelocityTraj.output, hybridTaskLogGroup.getInput<0>());
    barrett::systems::connect(basetoTaskVelocity.velocityTaskOut, hybridTaskLogGroup.getInput<1>());
    barrett::systems::connect(desiredForceSource.output, hybridTaskLogGroup.getInput<2>());
    barrett::systems::connect(basetoTaskForce.forceTaskOut, hybridTaskLogGroup.getInput<3>());

    const size_t kHybridLogPeriodMult = 1;

    HybridTaskCsvLogger<hybrid_task_log_record_t>* hybridTaskLogger = NULL;

    bool contactActive = false;

    std::cout << "\nCommands:\n";
    std::cout << "  t : move to target\n";
    std::cout << "  c : start contact controller\n";
    std::cout << "  x : stop contact controller\n";
    std::cout << "  ENTER : move home\n";
    std::cout << "  q : quit\n";
    std::cout << "  (datalog: ../../data/hybrid_task_datalog_NNNN.csv while contact is active)\n";

    barrett::systems::ExecutionManager* const em = pm.getExecutionManager();

    auto stopHybridDatalog = [&]() {
        if (hybridTaskLogger == NULL) {
            return;
        }
        barrett::systems::disconnect(hybridTaskLogger->input);
        hybridTaskLogger->closeFile();
        delete hybridTaskLogger;
        hybridTaskLogger = NULL;
    };

    auto startHybridDatalog = [&]() {
        const std::string logPath = nextHybridTaskLogPath();
        hybridTaskLogger = new HybridTaskCsvLogger<hybrid_task_log_record_t>(
            em, logPath, kHybridLogPeriodMult, "HybridTaskCsvLogger");
        barrett::systems::forceConnect(hybridTaskLogGroup.output, hybridTaskLogger->input);
        std::cout << "CSV datalog (task frame) -> " << logPath << std::endl;
    };

    wam.gravityCompensate();

    while (true) {
        std::cout << "\nEnter command: ";

        if (std::cin.peek() == '\n') {
            std::cin.get();

            if (contactActive) {
                barrett::systems::disconnect(wam.input);
                delayedTaskForce.reset(zeroTaskForce);
                hybridControl.resetState();
                stopHybridDatalog();
                contactActive = false;
            }

            // setDefaultGains();

            std::cout << "Returning home..." << std::endl;
            wam.moveHome();
            std::cout << "At home." << std::endl;

            continue;
        }

        

        char cmd;
        std::cin >> cmd;

        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');


        if (cmd == 'q') {
            std::cout << "Exiting..." << std::endl;

            if (contactActive) {
                barrett::systems::disconnect(wam.input);
                stopHybridDatalog();
                contactActive = false;
            }

            // setDefaultGains();
            break;
        }

        switch (cmd) {

            case 't':
                if (contactActive) {
                    barrett::systems::disconnect(wam.input);
                    delayedTaskForce.reset(zeroTaskForce);
                    hybridControl.resetState();
                    stopHybridDatalog();
                    contactActive = false;
                }

                // setDefaultGains();

                std::cout << "Moving to target..." << std::endl;
                wam.moveTo(target, true);
                std::cout << "At target." << std::endl;
                break;


            case 'c':
                if (!contactActive) {
                    std::cout << "Starting contact controller..." << std::endl;

                    delayedTaskForce.reset(zeroTaskForce);
                    hybridControl.resetState();
                    desiredVelocityTraj.resetPhase();

                    startHybridDatalog();

                    // setContactGains();basetoTaskForce.forceTaskOut
                    // barrett::systems::connect(controlToJoint.jointTorqueOut, wam.input);

                    // wam.idle(); 
                    // setDefaultGains();
                    wam.trackReferenceSignal(controlToJoint.jointTorqueOut);
                    // barrett::systems::connect(controlToJoint.jointTorqueOut, wam.input);

                    contactActive = true;
                } else {
                    std::cout << "Contact controller already active." << std::endl;
                }
                break;

            // case 'i':
            //     std::cout << "Switching to idle..." << std::endl;

            //     if (contactActive) {
            //         stopHybridDatalog();
            //         delayedTaskForce.reset(zeroTaskForce);
            //         hybridControl.resetState();
            //         contactActive = false;
            //     }

            //     wam.idle();
            //     break;

            // case 'c':
            //     if (!contactActive) {
            //         std::cout << "Starting contact controller..." << std::endl;

            //         delayedTaskForce.reset(zeroTaskForce);
            //         hybridControl.resetState();
            //         desiredVelocityTraj.resetPhase();

            //         startHybridDatalog();

            //         // optionally set contact gains here
            //         // setContactGains();

            //         wam.trackReferenceSignal(controlToJoint.jointTorqueOut);

            //         contactActive = true;
            //     } else {
            //         std::cout << "Contact controller already active." << std::endl;
            //     }
            //     break;


            case 'x':
                if (contactActive) {
                    std::cout << "Stopping contact controller..." << std::endl;

                    barrett::systems::disconnect(wam.input);
                    delayedTaskForce.reset(zeroTaskForce);
                    hybridControl.resetState();
                    stopHybridDatalog();

                    // setDefaultGains();

                    contactActive = false;
                } else {
                    std::cout << "Contact controller not active." << std::endl;
                }
                break;

            default:
                std::cout << "Unknown command." << std::endl;
                break;
        }
    }

    return 0;
}