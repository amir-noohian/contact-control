#include "hybrid_cartesian_nominal.hpp"

#include <barrett/detail/stl_utils.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <barrett/systems.h>
#include <barrett/units.h>

#include <iostream>

using namespace barrett;

template <size_t DOF>
typename units::JointTorques<DOF>::type saturateJt(
        const typename units::JointTorques<DOF>::type& torque,
        const typename units::JointTorques<DOF>::type& limit) {
    int index;
    const double ratio = limit.cwiseQuotient(torque.cwiseAbs()).minCoeff(&index);
    return ratio < 1.0 ? ratio * torque : torque;
}

template <size_t DOF>
class NominalGravityFeedforward : public systems::System,
                                  public systems::KinematicsInput<DOF> {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
    Output<jt_type> output;

    NominalGravityFeedforward(const libconfig::Setting& kinematics, const libconfig::Setting& robot)
        : System("NominalGravityFeedforward"),
          systems::KinematicsInput<DOF>(this),
          output(this, &outputValue_),
          model_(kinematics, robot) {}

    ~NominalGravityFeedforward() override { mandatoryCleanUp(); }

protected:
    void operate() override {
        torque_ = model_.gravityTorque(this->kinInput.getValue());
        outputValue_->setData(&torque_);
    }

private:
    typename Output<jt_type>::Value* outputValue_;
    hybrid_cartesian_dynamics::NominalPlanarDynamics<DOF> model_;
    jt_type torque_;
};

template <size_t DOF>
class NominalGravityCommand : public systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
    Input<jt_type> nominalGravityInput, barrettGravityInput;
    Output<jt_type> output;

    NominalGravityCommand()
        : System("NominalGravityCommand"),
          nominalGravityInput(this),
          barrettGravityInput(this),
          output(this, &outputValue_) {}

    ~NominalGravityCommand() override { mandatoryCleanUp(); }

protected:
    void operate() override {
        const auto& nominal = nominalGravityInput.getValue();
        const auto& barrettGravity = barrettGravityInput.getValue();
        command_.setZero();
        command_.head(4) = nominal.head(4) - barrettGravity.head(4);
        outputValue_->setData(&command_);
    }

private:
    typename Output<jt_type>::Value* outputValue_;
    jt_type command_;
};

template <size_t DOF>
int wam_main(int, char**, ProductManager& pm, systems::Wam<DOF>& wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    if (DOF != 4 && DOF != 7) {
        std::cerr << "Nominal gravity compensation requires a 4- or 7-DOF WAM.\n";
        return 1;
    }

    auto& robotConfig = pm.getConfig();
    robotConfig.setAutoConvert(true);
    const auto& robot = robotConfig.lookup(pm.getWamDefaultConfigPath());
    const auto& kinematics = robot["kinematics"];
    NominalGravityFeedforward<DOF> nominalGravity(kinematics, robot);
    NominalGravityCommand<DOF> command;

    jt_type limits;
    limits.setConstant(10.0);
    limits.head(4) << 50.0, 40.0, 30.0, 30.0;
    systems::Callback<jt_type> torqueLimit(
        boost::bind(saturateJt<DOF>, _1, 0.5 * limits));

    systems::connect(wam.kinematicsBase.kinOutput, nominalGravity.kinInput);
    systems::connect(nominalGravity.output, command.nominalGravityInput);
    systems::connect(wam.gravity.output, command.barrettGravityInput);
    systems::connect(command.output, torqueLimit.input);

    pm.getSafetyModule()->setVelocityLimit(1.5);
    wam.gravityCompensate();

    std::cout << "Nominal gravity compensation from the active WAM dynamics config.\n"
                 "Press Enter to start.\n";
    detail::waitForEnter();
    wam.trackReferenceSignal(torqueLimit.output);

    std::cout << "Press Enter to stop.\n";
    detail::waitForEnter();
    wam.idle();
    wam.moveHome();
    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
    return 0;
}
