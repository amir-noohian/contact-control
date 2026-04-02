#pragma once

#include <chrono>
#include <cmath>

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


/** Smooth back-and-forth task-frame velocity along one axis: v(t) = peakSpeed * sin(2*pi*t/T). */
template <typename VelocityVectorType>
class TaskVelocitySinusoid : public barrett::systems::System {
 public:
    Output<VelocityVectorType> output;

    explicit TaskVelocitySinusoid(barrett::systems::ExecutionManager* em,
                                    int taskAxis,
                                    double peakSpeed,
                                    double periodSec,
                                    const std::string& sysName = "TaskVelocitySinusoid")
        : System(sysName)
        , output(this, &outputValue)
        , taskAxis_(taskAxis)
        , peakSpeed_(peakSpeed)
        , periodSec_(periodSec > 0.0 ? periodSec : 1.0)
        , haveT0_(false) {
        if (em != NULL) {
            em->startManaging(*this);
        }
        vOut_.setZero();
    }

    virtual ~TaskVelocitySinusoid() {
        this->mandatoryCleanUp();
    }

    void setPeakSpeed(double v) {
        peakSpeed_ = v;
    }

    void setPeriod(double sec) {
        if (sec > 0.0) {
            periodSec_ = sec;
        }
    }

    void setTaskAxis(int axis) {
        taskAxis_ = axis;
    }

    /** Restart phase so motion starts at zero velocity (smooth). */
    void resetPhase() {
        haveT0_ = false;
    }

 protected:
    typename Output<VelocityVectorType>::Value* outputValue;
    VelocityVectorType vOut_;

    int taskAxis_;
    double peakSpeed_;
    double periodSec_;
    bool haveT0_;

    std::chrono::steady_clock::time_point t0_;

    virtual void operate() {
        using std::chrono::duration;
        using std::chrono::steady_clock;

        if (!haveT0_) {
            t0_ = steady_clock::now();
            haveT0_ = true;
        }

        const double t =
            duration<double>(steady_clock::now() - t0_).count();
        const double w = 2.0 * M_PI / periodSec_;
        const double s = std::sin(w * t);

        vOut_.setZero();
        if (taskAxis_ >= 0 && taskAxis_ < 3) {
            vOut_[taskAxis_] = peakSpeed_ * s;
        }

        outputValue->setData(&vOut_);
    }

 private:
    DISALLOW_COPY_AND_ASSIGN(TaskVelocitySinusoid);
};
