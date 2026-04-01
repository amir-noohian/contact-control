#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <boost/numeric/ublas/operation.hpp>

template <size_t DOF>
class TaskToBaseControl : public barrett::systems::System {
  public:
    typedef typename barrett::math::Vector<3>::type task_control_type;
    typedef typename barrett::math::Vector<3>::type base_control_type;
    typedef typename barrett::math::Matrix<3, 3>::type transform_type;

    Input<task_control_type> controlTaskIn;
    Input<transform_type> baseToTaskIn;

    Output<base_control_type> controlBaseOut;

    explicit TaskToBaseControl(barrett::systems::ExecutionManager* em,
                               const std::string& sysName = "TaskToBaseControl")
        : System(sysName)
        , controlTaskIn(this)
        , baseToTaskIn(this)
        , controlBaseOut(this, &controlBaseOutputValue) {
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~TaskToBaseControl() {
        this->mandatoryCleanUp();
    }

  protected:
    typename Output<base_control_type>::Value* controlBaseOutputValue;

    task_control_type controlTask;
    base_control_type controlBase;
    transform_type Rtb;

    virtual void operate() {
        controlTask = controlTaskIn.getValue();
        Rtb = baseToTaskIn.getValue();

        // base = Rtb^T * task
        controlBase(0) = Rtb(0,0) * controlTask(0) + Rtb(1,0) * controlTask(1) + Rtb(2,0) * controlTask(2);
        controlBase(1) = Rtb(0,1) * controlTask(0) + Rtb(1,1) * controlTask(1) + Rtb(2,1) * controlTask(2);
        controlBase(2) = Rtb(0,2) * controlTask(0) + Rtb(1,2) * controlTask(1) + Rtb(2,2) * controlTask(2);


        controlBaseOutputValue->setData(&controlBase);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(TaskToBaseControl);
};


template <size_t DOF>
class BaseControlToJointTorque : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    typedef typename barrett::math::Vector<3>::type base_control_type;
    typedef barrett::math::Matrix<6, DOF> jacobian_type;

    Input<base_control_type> controlBaseIn;
    Input<jacobian_type> jacobianIn;

    Output<jt_type> jointTorqueOut;

    explicit BaseControlToJointTorque(barrett::systems::ExecutionManager* em,
                                      const std::string& sysName = "BaseControlToJointTorque")
        : System(sysName)
        , controlBaseIn(this)
        , jacobianIn(this)
        , jointTorqueOut(this, &jointTorqueOutputValue) {
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~BaseControlToJointTorque() {
        this->mandatoryCleanUp();
    }

  protected:
    typename Output<jt_type>::Value* jointTorqueOutputValue;

    base_control_type controlBase;
    jacobian_type J;
    jt_type tau;

    virtual void operate() {
        controlBase = controlBaseIn.getValue();
        J = jacobianIn.getValue();

        // tau_i = J(0,i)*fx + J(1,i)*fy + J(2,i)*fz
        for (size_t i = 0; i < DOF; ++i) {
            tau[i] = J(0, i) * controlBase[0]
                   + J(1, i) * controlBase[1]
                   + J(2, i) * controlBase[2];
        }

        jointTorqueOutputValue->setData(&tau);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(BaseControlToJointTorque);
};