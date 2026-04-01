#include "external_torque.hpp"
#include "external_wrench.hpp"
#include "jacobian.hpp"
#include "utilities.hpp"
#include "bt_transformation.hpp"
#include "task_space_controller.hpp"
#include "tj_transformation.hpp"

#include <barrett/systems.h>
#include <barrett/units.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <barrett/detail/ca_macro.h>


template<size_t DOF>
int wam_main(int argc, char** argv, barrett::ProductManager& pm, barrett::systems::Wam<DOF>& wam)
{
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    typedef barrett::math::Matrix<6, DOF> jacobian_type;
    typedef typename barrett::math::Vector<3>::type task_vector_force_type;
    typedef typename barrett::math::Vector<3>::type task_vector_velocity_type;
    typedef typename barrett::math::Vector<3>::type task_control_type;
   
    
    wam.gravityCompensate();

    ExternalTorque<DOF> externalTorque(pm.getExecutionManager());
    ExternalWrench<DOF> externalWrench(pm.getExecutionManager());
    ToolJacobianOutput<DOF> toolJacobian(wam, pm.getExecutionManager());
    LinearCartesianVelocity<DOF> linearCartesianVelocity(pm.getExecutionManager());
    BaseToTaskTransform<DOF> basetoTaskTransform(pm.getExecutionManager());
    BaseToTaskForce<DOF> basetoTaskForce(pm.getExecutionManager());
    BaseToTaskVelocity<DOF> basetoTaskVelocity(pm.getExecutionManager());
    HybridForceVelocityControl<DOF> hybridControl(pm.getExecutionManager());
    TaskToBaseControl<DOF> taskToBaseTransform(pm.getExecutionManager());
    BaseControlToJointTorque<DOF> controlToJoint(pm.getExecutionManager());

    // barrett::systems::Summer<jt_type, 2> customjtSum;
    // pm.getExecutionManager()->startManaging(customjtSum);

    barrett::systems::PrintToStream<cf_type> printCartesianForce(pm.getExecutionManager(), "cartesianForce: ");
    barrett::systems::PrintToStream<cp_type> printCartesianPosition(pm.getExecutionManager(), "cartesianPosition: ");
    barrett::systems::PrintToStream<cv_type> printCartesianVelocity(pm.getExecutionManager(), "cartesianVelocity: ");
    barrett::systems::PrintToStream<jacobian_type> printJacobian(pm.getExecutionManager(), "Jacobian: ");
    barrett::systems::PrintToStream<jt_type> printJointTorque(pm.getExecutionManager(), "externalJointTorque: ");
    barrett::systems::PrintToStream<cv_type> printCartesianVelocityJacobian(pm.getExecutionManager(), "cartesianVelocityJacobian: ");
    barrett::systems::PrintToStream<task_vector_force_type> printTaskForce(pm.getExecutionManager(), "toolForce: ");
    barrett::systems::PrintToStream<task_vector_velocity_type> printTaskVelocity(pm.getExecutionManager(), "toolVelocity: ");
    barrett::systems::PrintToStream<task_control_type> printTaskControl(pm.getExecutionManager(), "taskControl: ");
    barrett::systems::PrintToStream<jt_type> printJointCommand(pm.getExecutionManager(), "jointCommand: ");
    // barrett::systems::PrintToStream<jt_type> printcustomjtSum(pm.getExecutionManager(), "customjtSum: ");
    barrett::systems::PrintToStream<jt_type> printjtSum(pm.getExecutionManager(), "jtSum: ");


    // jtsum for external torque
    // barrett::systems::connect(wam.gravity.output, customjtSum.getInput(0));
    // barrett::systems::connect(wam.supervisoryController.output, customjtSum.getInput(1));

    // external torque
    barrett::systems::connect(wam.gravity.output, externalTorque.wamGravityIn);
    barrett::systems::connect(wam.jtSum.output, externalTorque.wamTorqueSumIn);

    //external force and momentum
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

    // task space controller
    task_vector_force_type desiredForce;
    task_vector_velocity_type desiredVelocity;
    desiredVelocity.setZero();
    desiredForce.setZero();
    desiredVelocity[0] = 0.2;
    desiredForce[2] = 6.0;

    ConstantOutput<task_vector_velocity_type> desiredVelocitySource(pm.getExecutionManager(), desiredVelocity, "DesiredVelocity");
    ConstantOutput<task_vector_force_type> desiredForceSource(pm.getExecutionManager(), desiredForce, "DesiredForce");
    
    barrett::systems::connect(desiredVelocitySource.output, hybridControl.desiredVelocityIn);
    barrett::systems::connect(desiredForceSource.output, hybridControl.desiredForceIn);
    barrett::systems::connect(basetoTaskVelocity.velocityTaskOut, hybridControl.currentVelocityIn);
    barrett::systems::connect(basetoTaskForce.forceTaskOut, hybridControl.currentForceIn);

    // joint space command
    barrett::systems::connect(hybridControl.controlOutput, taskToBaseTransform.controlTaskIn);
    barrett::systems::connect(basetoTaskTransform.baseToTaskOut, taskToBaseTransform.baseToTaskIn);
    barrett::systems::connect(taskToBaseTransform.controlBaseOut, controlToJoint.controlBaseIn);
    barrett::systems::connect(toolJacobian.output, controlToJoint.jacobianIn);

    // printing outputs
    barrett::systems::connect(externalWrench.forceOut, printCartesianForce.input);
    // barrett::systems::connect(wam.toolPosition.output, printCartesianPosition.input);
    barrett::systems::connect(wam.toolVelocity.output, printCartesianVelocity.input);
    // barrett::systems::connect(toolJacobian.output, printJacobian.input);
    // barrett::systems::connect(externalTorque.wamExternalTorqueOut, printJointTorque.input);
    barrett::systems::connect(linearCartesianVelocity.linearVelocityOut, printCartesianVelocityJacobian.input);
    barrett::systems::connect(basetoTaskForce.forceTaskOut, printTaskForce.input);
    barrett::systems::connect(basetoTaskVelocity.velocityTaskOut, printTaskVelocity.input);
    // barrett::systems::connect(hybridControl.controlOutput, printTaskControl.input);
    // barrett::systems::connect(controlToJoint.jointTorqueOut, printJointCommand.input);
    // barrett::systems::connect(customjtSum.output, printcustomjtSum.input);
    barrett::systems::connect(wam.jtSum.output, printjtSum.input);


    jp_type target;
    target.setZero();

    // Example only. Replace with a safe pose for your robot.
    // For a 4-DOF WAM, set only the first 4 entries.
    if (DOF >= 1) { target[0] =  0.0; }
    if (DOF >= 2) { target[1] =  0.9; }
    if (DOF >= 3) { target[2] =  0.0; }
    if (DOF >= 4) { target[3] =  1.85; }
    if (DOF >= 5) { target[4] =  0.0; }
    if (DOF >= 6) { target[5] =  0.0; }
    if (DOF >= 7) { target[6] =  0.0; }

    // std::cout << "Press Enter to move to target position..." << std::endl;

    // // Important: clear buffer first
    // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    // std::cin.get();

    // std::cout << "Moving to target..." << std::endl;
    // wam.moveTo(target, true);

    // std::cout << "At target position." << std::endl;
    // std::cout << "Apply force to the robot now." << std::endl;
    // std::cout << "Press Enter to go back home." << std::endl;

    // // Clear input buffer
    // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    // std::cin.get();

    // std::cout << "Returning home..." << std::endl;

    // wam.moveHome();

    // std::cout << "Done." << std::endl;

    bool contactActive = false;

    std::cout << "\nCommands:\n";
    std::cout << "  t : move to target\n";
    std::cout << "  c : start contact controller\n";
    std::cout << "  x : stop contact controller\n";
    std::cout << "  ENTER : move home\n";
    std::cout << "  q : quit\n";

    while (true) {
        std::cout << "\nEnter command: ";

        // Check if user pressed ENTER only
        if (std::cin.peek() == '\n') {
            std::cin.get();  // consume newline

            std::cout << "Returning home..." << std::endl;
            wam.moveHome();
            std::cout << "At home." << std::endl;

            continue;
        }

        char cmd;
        std::cin >> cmd;

        // Clear remaining input
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        if (cmd == 'q') {
            std::cout << "Exiting..." << std::endl;
            break;
        }

        switch (cmd) {

            case 't':
                std::cout << "Moving to target..." << std::endl;
                wam.moveTo(target, true);
                std::cout << "At target." << std::endl;
                break;

            case 'c':
                if (!contactActive) {
                    std::cout << "Starting contact controller..." << std::endl;

                    // Use wam.input (jtSum JT_INPUT): trackReferenceSignal routes through
                    // supervisory Converter, which can leave SC undefined when jtSum runs
                    // (jtSum uses undefined-as-zero), so only gravity appeared in jtSum.
                    wam.idle();
                    barrett::systems::forceConnect(controlToJoint.jointTorqueOut, wam.input);

                    contactActive = true;
                } else {
                    std::cout << "Contact controller already active." << std::endl;
                }
                break;

            case 'x':
                if (contactActive) {
                    std::cout << "Stopping contact controller..." << std::endl;

                    barrett::systems::disconnect(wam.input);

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