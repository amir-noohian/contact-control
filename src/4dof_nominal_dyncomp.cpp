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
class NominalDynamicsCompensation : public systems::System,
                                   public systems::KinematicsInput<DOF> {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
    Input<jv_type> jointVelocity;
    Input<jt_type> barrettGravity;
    Output<jt_type> output;

    NominalDynamicsCompensation(const libconfig::Setting& kinematics, const libconfig::Setting& robot)
        : System("NominalDynamicsCompensation"),
          systems::KinematicsInput<DOF>(this),
          jointVelocity(this),
          barrettGravity(this),
          output(this, &outputValue_),
          model_(kinematics, robot) {}

    ~NominalDynamicsCompensation() override { mandatoryCleanUp(); }

protected:
    void operate() override {
        const jt_type nominal = model_.inverseDynamicsBias(
            this->kinInput.getValue(), jointVelocity.getValue());
        const auto& gravity = barrettGravity.getValue();
        command_.setZero();
        command_.head(4) = nominal.head(4) - gravity.head(4);
        outputValue_->setData(&command_);
    }

private:
    typename Output<jt_type>::Value* outputValue_;
    hybrid_cartesian_dynamics::NominalPlanarDynamics<DOF> model_;
    jt_type command_;
};

template <size_t DOF>
int wam_main(int, char**, ProductManager& pm, systems::Wam<DOF>& wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    if (DOF != 4 && DOF != 7) {
        std::cerr << "Nominal dynamic compensation requires a 4- or 7-DOF WAM.\n";
        return 1;
    }

    auto& robotConfig = pm.getConfig();
    robotConfig.setAutoConvert(true);
    const auto& robot = robotConfig.lookup(pm.getWamDefaultConfigPath());
    const auto& kinematics = robot["kinematics"];
    NominalDynamicsCompensation<DOF> compensation(kinematics, robot);

    jt_type limits;
    limits.setConstant(10.0);
    limits.head(4) << 50.0, 40.0, 30.0, 30.0;
    systems::Callback<jt_type> torqueLimit(
        boost::bind(saturateJt<DOF>, _1, 0.5 * limits));

    systems::connect(wam.kinematicsBase.kinOutput, compensation.kinInput);
    systems::connect(wam.jvOutput, compensation.jointVelocity);
    systems::connect(wam.gravity.output, compensation.barrettGravity);
    systems::connect(compensation.output, torqueLimit.input);

    pm.getSafetyModule()->setVelocityLimit(1.5);
    wam.gravityCompensate();

    std::cout << "Nominal inverse-dynamics compensation from the active WAM config "
                 "(zero joint acceleration, first four joints).\n"
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
