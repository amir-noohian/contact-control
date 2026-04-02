#pragma once

#include <algorithm>

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>

template <size_t DOF>
class HybridForceVelocityControl : public barrett::systems::System {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

 public:
    typedef typename barrett::math::Vector<3>::type task_vector_force_type;
    typedef typename barrett::math::Vector<3>::type task_vector_velocity_type;
    typedef typename barrett::math::Matrix<3, 3>::type selection_matrix_type;
    typedef typename barrett::math::Matrix<3, 3>::type velocity_gain_type;
    typedef typename barrett::math::Matrix<3, 3>::type force_gain_type;
    typedef typename barrett::math::Vector<3>::type task_control_velocity_type;
    typedef typename barrett::math::Vector<3>::type task_control_force_type;
    typedef typename barrett::math::Vector<3>::type task_control_type;

    /** Default gains for the stock selection matrix (velocity on task axis 0, force on 2). */
    static constexpr double defaultKvAxis0 = 50.0;
    static constexpr double defaultKdAxis0 = 0.0;
    static constexpr double defaultKfAxis2 = 0.5;
    static constexpr double defaultKiAxis2 = 0.0;
    static constexpr double defaultForceIntegralLimit = 25.0;

    Input<task_vector_velocity_type> desiredVelocityIn;
    Input<task_vector_velocity_type> currentVelocityIn;

    Input<task_vector_force_type> desiredForceIn;
    Input<task_vector_force_type> currentForceIn;

    Output<task_control_type> controlOutput;

    explicit HybridForceVelocityControl(
        barrett::systems::ExecutionManager* em,
        const std::string& sysName = "HybridForceVelocityControl")
        : System(sysName)
        , desiredVelocityIn(this)
        , currentVelocityIn(this)
        , desiredForceIn(this)
        , currentForceIn(this)
        , controlOutput(this, &controlOutputValue)
        , em_(em)
        , haveVerrPrev_(false)
        , fIntegralLimit_(defaultForceIntegralLimit) {

        if (em != NULL) {
            em->startManaging(*this);
        }

        // Identity
        I.setIdentity();

        // Default selection matrix:
        // motion in x, force in z for example
        S(0,0) = 1.0;  S(0,1) = 0.0;  S(0,2) = 0.0;
        S(1,0) = 0.0;  S(1,1) = 0.0;  S(1,2) = 0.0;
        S(2,0) = 0.0;  S(2,1) = 0.0;  S(2,2) = 0.0;

        // Default velocity gains (P + D on velocity error in velocity-controlled axes)
        Kv.setZero();
        Kv(0, 0) = defaultKvAxis0;
        Kd.setZero();
        Kd(0, 0) = defaultKdAxis0;

        // Default force gains (P + I on force error in force-controlled axes)
        Kf.setZero();
        Kf(2, 2) = defaultKfAxis2;
        Ki.setZero();
        Ki(2, 2) = defaultKiAxis2;

        fIntegral.setZero();
        vErrPrev.setZero();
    }

    virtual ~HybridForceVelocityControl() {
        this->mandatoryCleanUp();
    }

    void setSelectionMatrix(const selection_matrix_type& selection) {
        S = selection;
    }

    void setVelocityGain(const velocity_gain_type& velocityGain) {
        Kv = velocityGain;
    }

    void setVelocityDerivativeGain(const velocity_gain_type& derivativeGain) {
        Kd = derivativeGain;
    }

    void setForceGain(const force_gain_type& forceGain) {
        Kf = forceGain;
    }

    void setForceIntegralGain(const force_gain_type& integralGain) {
        Ki = integralGain;
    }

    /** Per-axis clamp on integral of force error (|fErr|·s); reduces windup. */
    void setForceIntegralLimit(double limit) {
        fIntegralLimit_ = std::max(0.0, limit);
    }

    /** Clear integral and derivative state (e.g. when enabling/disabling contact). */
    void resetState() {
        fIntegral.setZero();
        vErrPrev.setZero();
        haveVerrPrev_ = false;
    }

 protected:
    typename Output<task_control_type>::Value* controlOutputValue;

    barrett::systems::ExecutionManager* em_;

    task_vector_velocity_type vDes;
    task_vector_velocity_type vCur;
    task_vector_force_type fDes;
    task_vector_force_type fCur;

    task_vector_velocity_type vErr;
    task_vector_force_type fErr;

    task_vector_velocity_type vErrPrev;
    bool haveVerrPrev_;

    task_vector_force_type fIntegral;
    double fIntegralLimit_;

    task_control_velocity_type velocityTerm;
    task_control_force_type forceTerm;
    task_control_type u;

    selection_matrix_type S;
    selection_matrix_type Sf;
    velocity_gain_type Kv;
    velocity_gain_type Kd;
    force_gain_type Kf;
    force_gain_type Ki;
    selection_matrix_type I;

    virtual void operate() {
        const double dt =
            (em_ != NULL && em_->getPeriod() > 0.0) ? em_->getPeriod() : 0.001;

        vDes = desiredVelocityIn.getValue();
        vCur = currentVelocityIn.getValue();

        fDes = desiredForceIn.getValue();
        fCur = currentForceIn.getValue();

        Sf = I - S;

        vErr = vDes - vCur;
        fErr = fDes - fCur;

        task_vector_velocity_type vErrDot;
        if (!haveVerrPrev_) {
            vErrPrev = vErr;
            haveVerrPrev_ = true;
            vErrDot.setZero();
        } else {
            vErrDot = (vErr - vErrPrev) / dt;
            vErrPrev = vErr;
        }

        task_vector_force_type fErrSel = Sf * fErr;
        fIntegral += fErrSel * dt;
        if (fIntegralLimit_ > 0.0) {
            for (int i = 0; i < 3; ++i) {
                fIntegral[i] =
                    std::max(-fIntegralLimit_,
                             std::min(fIntegralLimit_, fIntegral[i]));
            }
        }

        velocityTerm = Kv * (S * vErr) + Kd * (S * vErrDot);
        forceTerm = Kf * fErrSel + Ki * fIntegral;

        u = velocityTerm + forceTerm;

        controlOutputValue->setData(&u);
    }

 private:
    DISALLOW_COPY_AND_ASSIGN(HybridForceVelocityControl);
};

