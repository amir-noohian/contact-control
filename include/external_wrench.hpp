/*
This file calculates the force applied to the robot by the environment.
*/

#pragma once

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/detail/ca_macro.h>

#include <Eigen/Dense>

using namespace barrett;

template <size_t DOF>
class ExternalWrench : public systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jt_type> externalTorqueIn;
    Input<math::Matrix<6, DOF> > jacobianIn;

    Output<cf_type> forceOut;
    Output<ct_type> momentOut;

  protected:
    typename Output<cf_type>::Value* forceOutputValue;
    typename Output<ct_type>::Value* momentOutputValue;

  public:
    explicit ExternalWrench(barrett::systems::ExecutionManager* em, const std::string& sysName = "ExternalWrench")
        : System(sysName)
        , externalTorqueIn(this)
        , jacobianIn(this)
        , forceOut(this, &forceOutputValue)
        , momentOut(this, &momentOutputValue) {}

    virtual ~ExternalWrench() {
        this->mandatoryCleanUp();
    }

  protected:
    jt_type tauExt;
    math::Matrix<6, DOF> J;

    cf_type force;
    ct_type moment;

    Eigen::Matrix<double, DOF, 1> tauEigen;
    Eigen::Matrix<double, DOF, 6> JT;
    Eigen::Matrix<double, 6, 1> wrenchEigen;

    virtual void operate() {
        tauExt = externalTorqueIn.getValue();
        J = jacobianIn.getValue();

        // Convert directly (no loops)
        tauEigen = tauExt;
        JT = J.transpose();

        // Solve J^T W = tau
        wrenchEigen = JT.colPivHouseholderQr().solve(tauEigen);

        // Split cleanly (no loops)
        force = wrenchEigen.template head<3>();
        moment = wrenchEigen.template tail<3>();

        forceOutputValue->setData(&force);
        momentOutputValue->setData(&moment);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(ExternalWrench);
};