// This file calculates the inverse dynamics of only the first four dofs of the WAM.

#ifndef CONTACT_CONTROL_DYNAMICS_H_
#define CONTACT_CONTROL_DYNAMICS_H_

#pragma once
#include <eigen3/Eigen/Dense>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/math/kinematics.h>
#include "beta.h"
#include "regressor_W.h"

using namespace barrett;

template<size_t DOF>
class Dynamics: public systems::System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	/* Torque*/
public:
	Input<jp_type> jpInputDynamics;
	Input<jv_type> jvInputDynamics;
    Input<ja_type> jaInputDynamics;

public:
	Output<jt_type> dynamicsFeedFWD;
protected:
	typename Output<jt_type>::Value* dynamicsFeedFWDValue;

public:
	// Opt in to acceleration feedforward; legacy compensation keeps it disabled.
	explicit Dynamics(bool useAcceleration = false) :
			jpInputDynamics(this), jvInputDynamics(this), jaInputDynamics(this), dynamicsFeedFWD(this,
					&dynamicsFeedFWDValue), use_acceleration(useAcceleration) {
	//		      em->startManaging(*this);
//		    }
	}

	virtual ~Dynamics() {
		this->mandatoryCleanUp();
	}

protected:
	jp_type tmp_theta_pos;
	jv_type tmp_theta_vel;
	ja_type tmp_theta_acc;
	Eigen::Matrix<double, 4, 30> W;
	Eigen::Matrix<double, 30, 1> beta;
	Eigen::Vector4d ThetaInput;
	Eigen::Vector4d ThetadotInput;
	Eigen::Vector4d ThetaddotInput;
	Eigen::Vector4d FeedFwd;
	jt_type dynFeedFWD;

	virtual void operate() {
		tmp_theta_pos = this->jpInputDynamics.getValue();
		ThetaInput << tmp_theta_pos[0], tmp_theta_pos[1], tmp_theta_pos[2], tmp_theta_pos[3];
		tmp_theta_vel = this->jvInputDynamics.getValue();
		ThetadotInput << tmp_theta_vel[0], tmp_theta_vel[1], tmp_theta_vel[2], tmp_theta_vel[3];
		tmp_theta_acc = this->jaInputDynamics.getValue();
		ThetaddotInput << tmp_theta_acc[0], tmp_theta_acc[1], tmp_theta_acc[2], tmp_theta_acc[3];
		if (!use_acceleration) ThetaddotInput.setZero();
		beta = initialize_beta();
        W = calculate_W_eigen(ThetaInput, ThetadotInput, ThetaddotInput);

		FeedFwd = W * beta;

		dynFeedFWD.setZero();
		dynFeedFWD.head(4) = FeedFwd;

		this->dynamicsFeedFWDValue->setData(&dynFeedFWD);

	}
private:
	const bool use_acceleration;
	DISALLOW_COPY_AND_ASSIGN(Dynamics);
};
#endif /* CONTACT_CONTROL_DYNAMICS_H_ */
