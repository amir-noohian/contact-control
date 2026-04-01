#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>
#include <Eigen/Core>

template <size_t DOF>
class LinearCartesianVelocity : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    typedef barrett::math::Matrix<6, DOF> jacobian_type;

    Input<jacobian_type> jacobianIn;
    Input<jv_type> jointVelocityIn;
    Output<cv_type> linearVelocityOut;

    explicit LinearCartesianVelocity(barrett::systems::ExecutionManager* em,
                                     const std::string& sysName = "LinearCartesianVelocity")
        : barrett::systems::System(sysName)
        , jacobianIn(this)
        , jointVelocityIn(this)
        , linearVelocityOut(this, &linearVelocityOutputValue) {

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~LinearCartesianVelocity() {
        this->mandatoryCleanUp();
    }

  protected:
    typename Output<cv_type>::Value* linearVelocityOutputValue;

    jacobian_type J;
    jv_type qdot;
    cv_type linearVelocity;

    virtual void operate() {
        J = jacobianIn.getValue();
        qdot = jointVelocityIn.getValue();
        linearVelocity = J.template topRows<3>() * qdot;
        linearVelocityOutputValue->setData(&linearVelocity);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(LinearCartesianVelocity);
};




template <typename T>
class ConstantOutput : public barrett::systems::System {
 public:
    Output<T> output;

    explicit ConstantOutput(barrett::systems::ExecutionManager* em,
                            const T& initialValue,
                            const std::string& sysName = "ConstantOutput")
        : System(sysName)
        , output(this, &outputValue)
        , value(initialValue) {
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~ConstantOutput() {
        this->mandatoryCleanUp();
    }

    void setValue(const T& newValue) {
        value = newValue;
    }

 protected:
    typename Output<T>::Value* outputValue;
    T value;

    virtual void operate() {
        outputValue->setData(&value);
    }

 private:
    DISALLOW_COPY_AND_ASSIGN(ConstantOutput);
};
