#pragma once

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
        , controlOutput(this, &controlOutputValue) {

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

        // Default velocity gain
        Kv(0,0) = 20.0; Kv(0,1) = 0.0; Kv(0,2) = 0.0;
        Kv(1,0) = 0.0; Kv(1,1) = 0.0; Kv(1,2) = 0.0;
        Kv(2,0) = 0.0; Kv(2,1) = 0.0; Kv(2,2) = 0.0;

        // Default force gain
        Kf(0,0) = 0.0; Kf(0,1) = 0.0; Kf(0,2) = 0.0;
        Kf(1,0) = 0.0; Kf(1,1) = 0.0; Kf(1,2) = 0.0;
        Kf(2,0) = 0.0; Kf(2,1) = 0.0; Kf(2,2) = 0.0;
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

    void setForceGain(const force_gain_type& forceGain) {
        Kf = forceGain;
    }

 protected:
    typename Output<task_control_type>::Value* controlOutputValue;

    task_vector_velocity_type vDes;
    task_vector_velocity_type vCur;
    task_vector_force_type fDes;
    task_vector_force_type fCur;

    task_vector_velocity_type vErr;
    task_vector_force_type fErr;

    task_control_velocity_type velocityTerm;
    task_control_force_type forceTerm;
    task_control_type u;

    selection_matrix_type S;
    selection_matrix_type Sf;
    velocity_gain_type Kv;
    force_gain_type Kf;
    selection_matrix_type I;

    virtual void operate() {
        vDes = desiredVelocityIn.getValue();
        vCur = currentVelocityIn.getValue();

        fDes = desiredForceIn.getValue();
        fCur = currentForceIn.getValue();

        // complementary selection for force channel
        Sf = I - S;

        // errors
        vErr = vDes - vCur;
        fErr = fDes - fCur;

        // hybrid terms
        velocityTerm = Kv * (S * vErr);
        forceTerm = Kf * (Sf, fErr);

        // final task-space command
        u = velocityTerm + forceTerm;

        controlOutputValue->setData(&u);
    }

 private:
    DISALLOW_COPY_AND_ASSIGN(HybridForceVelocityControl);
};

