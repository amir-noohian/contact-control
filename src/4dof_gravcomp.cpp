/*
 * 4dof_gravcomp.cpp
 *
 *  Created on: July, 2024
 *      Author: Amir Noohian
 */


#include "dynamics/dynamics.h"
#include <unistd.h>
#include <iostream>
#include <string>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/standard_main_function.h>

using namespace barrett;

//The WAM arm has a torque saturation limit
template <size_t DOF>
typename units::JointTorques<DOF>::type saturateJt(
    const typename units::JointTorques<DOF>::type& x,
    const typename units::JointTorques<DOF>::type& limit) {
    int index;
    double minRatio;

	minRatio = limit.cwiseQuotient(x.cwiseAbs()).minCoeff(&index);
    if (minRatio < 1.0) {
        return minRatio * x;
    } else {
        return x;
    }
}

//Our joint space inverse dynamics control (idc) : id_feedFWD + PD controller
template<size_t DOF>
class JsIDController :  public systems::System{ //
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	Input<jt_type> gravityInput; //Gravity
	Input<jt_type> feedFWDInput; //Inverse Dynamic Feed FWD

// IO  (outputs)
public:
	Output<jt_type> controlJtOutput;    // Output Control Joint Torque

protected:
	typename Output<jt_type>::Value* controlJtOutputValue;

public:
jt_type computedT;

public:
	explicit JsIDController(const std::string& sysName = "JsIDController"):
		System(sysName), controlJtOutput(this, &controlJtOutputValue), feedFWDInput(this), gravityInput(this)/*, kpCf(kpCoeff), kdCf(KdCoeff)*/{}

	virtual ~JsIDController() { this->mandatoryCleanUp(); }

protected:
	jt_type jt_out, feedFWD, gravity, gravity_mod;

	virtual void operate() {

		/*Taking feed forward term from the input terminal of this system*/
		gravity = this->gravityInput.getValue();
		feedFWD = this->feedFWDInput.getValue();

		gravity_mod.setZero();
        gravity_mod.head(4) = gravity.head(4);

		jt_out = feedFWD - gravity_mod;

		computedT = jt_out;

		this->controlJtOutputValue->setData(&jt_out);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(JsIDController);
};

template<size_t DOF>
int wam_main(int, char**, ProductManager& pm,	systems::Wam<DOF>& wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    if (DOF < 4) {
        std::cerr << "Compensation requires at least four joints.\n";
        return 1;
    }

	pm.getSafetyModule()->setVelocityLimit(1.5);


	wam.gravityCompensate();

	JsIDController<DOF> IDController;
	Dynamics<DOF> wam4dofDynamics;

	jt_type jtLimits;
	jtLimits.setConstant(10.0);
    jtLimits.head(4) << 50, 40, 30, 30;
	systems::Callback<jt_type> jtSat(boost::bind(saturateJt<DOF>,_1, 0.5*jtLimits));

	systems::Ramp time(pm.getExecutionManager(), 1.0);


	jv_type jv;
    jv.setConstant(0.0);
    systems::Constant<jv_type> zeroVelocity(jv);
    pm.getExecutionManager()->startManaging(zeroVelocity);

	ja_type ja;
    ja.setConstant(0.0);
    systems::Constant<ja_type> zeroAcceleration(ja);
    pm.getExecutionManager()->startManaging(zeroAcceleration);

	systems::connect(wam.jpOutput, wam4dofDynamics.jpInputDynamics);
	systems::connect(zeroVelocity.output, wam4dofDynamics.jvInputDynamics);
	systems::connect(zeroAcceleration.output, wam4dofDynamics.jaInputDynamics);
	systems::connect(wam4dofDynamics.dynamicsFeedFWD, IDController.feedFWDInput);
	systems::connect(wam.gravity.output, IDController.gravityInput);
	systems::connect(IDController.controlJtOutput, jtSat.input);


	std::cout<<"Press [Enter] to start."<<std::endl;
	detail::waitForEnter();
	time.start();
	usleep(2500);
	wam.trackReferenceSignal(jtSat.output); // Put it after time start so that feefwd term is active when we start applying torque.


	std::cout<<"Press [Enter] to stop."<<std::endl;
	detail::waitForEnter();
	time.stop();

    wam.idle();
	wam.moveHome();

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
