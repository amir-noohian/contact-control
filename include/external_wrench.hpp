/*
This file calculates the force and moment applied to the robot by the environment.
For 4-DOF robots, force and moment are estimated separately using the
translational and rotational parts of the Jacobian.
*/

#pragma once

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/detail/ca_macro.h>

#include <Eigen/Dense>
#include <string>

template <size_t DOF>
class ExternalWrench : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jt_type> externalTorqueIn;
    Input<barrett::math::Matrix<6, DOF> > jacobianIn;

    Output<cf_type> forceOut;
    Output<ct_type> momentOut;

  protected:
    typename Output<cf_type>::Value* forceOutputValue;
    typename Output<ct_type>::Value* momentOutputValue;

  public:
    explicit ExternalWrench(barrett::systems::ExecutionManager* em,
                            const std::string& sysName = "ExternalWrench")
        : barrett::systems::System(sysName)
        , externalTorqueIn(this)
        , jacobianIn(this)
        , forceOut(this, &forceOutputValue)
        , momentOut(this, &momentOutputValue)
        , lambdaForce_(1e-3)
        , lambdaMoment_(1e-3) {}

    virtual ~ExternalWrench() {
        this->mandatoryCleanUp();
    }

    void setForceDamping(double lambda) {
        lambdaForce_ = lambda;
    }

    void setMomentDamping(double lambda) {
        lambdaMoment_ = lambda;
    }

  protected:
    jt_type tauExt;
    barrett::math::Matrix<6, DOF> J;

    cf_type force;
    ct_type moment;

    Eigen::Matrix<double, DOF, 1> tauEigen;

    Eigen::Matrix<double, 3, DOF> Jforce;
    Eigen::Matrix<double, 3, DOF> Jmoment;

    Eigen::Matrix<double, 3, 1> forceEigen;
    Eigen::Matrix<double, 3, 1> momentEigen;

    double lambdaForce_;
    double lambdaMoment_;

    virtual void operate() {
        tauExt = externalTorqueIn.getValue();
        J = jacobianIn.getValue();

        tauEigen = tauExt;

        // Split Jacobian into translational and rotational parts
        Jforce  = J.template topRows<3>();
        Jmoment = J.template bottomRows<3>();

        // Estimate force: Jforce^T * F = tauExt
        forceEigen = solveDampedLeastSquares(Jforce, tauEigen, lambdaForce_);

        // Estimate moment: Jmoment^T * M = tauExt
        momentEigen = solveDampedLeastSquares(Jmoment, tauEigen, lambdaMoment_);

        force = forceEigen;
        moment = momentEigen;

        forceOutputValue->setData(&force);
        momentOutputValue->setData(&moment);
    }

    Eigen::Matrix<double, 3, 1> solveDampedLeastSquares(
        const Eigen::Matrix<double, 3, DOF>& Jpart,
        const Eigen::Matrix<double, DOF, 1>& tau,
        double lambda) {

        // Solve Jpart^T * x = tau
        // x = (Jpart * Jpart^T + lambda I)^(-1) * Jpart * tau

        Eigen::Matrix3d A = Jpart * Jpart.transpose();
        A += lambda * Eigen::Matrix3d::Identity();

        return A.ldlt().solve(Jpart * tau);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(ExternalWrench);
};