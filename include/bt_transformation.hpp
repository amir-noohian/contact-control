#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <boost/numeric/ublas/matrix.hpp>

template <size_t DOF>
class BaseToTaskVelocity : public barrett::systems::System {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

 public:
    typedef typename barrett::math::Vector<3>::type task_vector_velocity_type;
    typedef typename barrett::math::Matrix<3, 3>::type transform_type;

    Input<cv_type> velocityBaseIn;
    Input<transform_type> baseToTaskIn;
    Output<task_vector_velocity_type> velocityTaskOut;

    explicit BaseToTaskVelocity(barrett::systems::ExecutionManager* em,
                                const std::string& sysName = "BaseToTaskVelocity")
        : System(sysName)
        , velocityBaseIn(this)
        , baseToTaskIn(this)
        , velocityTaskOut(this, &velocityTaskOutputValue) {
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~BaseToTaskVelocity() {
        this->mandatoryCleanUp();
    }

 protected:
    typename Output<task_vector_velocity_type>::Value* velocityTaskOutputValue;

    cv_type velocityBase;
    transform_type Rtb;
    task_vector_velocity_type velocityTask;

    virtual void operate() {
        velocityBase = velocityBaseIn.getValue();
        Rtb = baseToTaskIn.getValue();

        task_vector_velocity_type tmp;
        tmp = Rtb * velocityBase;
        velocityTask = tmp;

        velocityTaskOutputValue->setData(&velocityTask);
    }

 private:
    DISALLOW_COPY_AND_ASSIGN(BaseToTaskVelocity);
};


template <size_t DOF>
class BaseToTaskForce : public barrett::systems::System {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

 public:
    typedef typename barrett::math::Vector<3>::type task_vector_force_type;
    typedef typename barrett::math::Matrix<3, 3>::type transform_type;

    Input<cf_type> forceBaseIn;
    Input<transform_type> baseToTaskIn;
    Output<task_vector_force_type> forceTaskOut;

    explicit BaseToTaskForce(barrett::systems::ExecutionManager* em,
                             const std::string& sysName = "BaseToTaskForce")
        : System(sysName)
        , forceBaseIn(this)
        , baseToTaskIn(this)
        , forceTaskOut(this, &forceTaskOutputValue) {
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~BaseToTaskForce() {
        this->mandatoryCleanUp();
    }

 protected:
    typename Output<task_vector_force_type>::Value* forceTaskOutputValue;

    cf_type forceBase;
    transform_type Rtb;
    task_vector_force_type forceTask;

    virtual void operate() {
        // forceBase = forceBaseIn.getValue();
        if (forceBaseIn.valueDefined()) {
            forceBase = forceBaseIn.getValue();
        } else {
            forceBase << 0.0, 0.0, 0.0;
        }

        Rtb = baseToTaskIn.getValue();

        task_vector_force_type tmp;
        tmp = Rtb * forceBase;
        forceTask = tmp;

        forceTaskOutputValue->setData(&forceTask);
    }

 private:
    DISALLOW_COPY_AND_ASSIGN(BaseToTaskForce);
};


template <size_t DOF>
class ForceClamp : public barrett::systems::System {
public:
    typedef typename barrett::math::Vector<3>::type task_vector_force_type;

    Input<task_vector_force_type> input;
    Output<task_vector_force_type> output;

    ForceClamp(barrett::systems::ExecutionManager* em,
               const task_vector_force_type& limit,
               const std::string& sysName = "ForceClamp")
        : System(sysName), input(this), output(this, &outputValue), limit_(limit) {
        if (em) em->startManaging(*this);
    }

protected:
    typename Output<task_vector_force_type>::Value* outputValue;
    task_vector_force_type limit_;

    virtual void operate() {
        task_vector_force_type f = input.getValue();

        for (size_t i = 0; i < 3; ++i) {
            // NaN protection
            if (!std::isfinite(f[i])) {
                f[i] = 0.0;
            }

            // clamp per axis
            if (f[i] > limit_[i])  f[i] = limit_[i];
            if (f[i] < -limit_[i]) f[i] = -limit_[i];
        }

        outputValue->setData(&f);
    }
};


template <size_t DOF>
class BaseToTaskTransform : public barrett::systems::System {
 public:
    typedef typename barrett::math::Matrix<3, 3>::type transform_type;

    Output<transform_type> baseToTaskOut;

    explicit BaseToTaskTransform(barrett::systems::ExecutionManager* em,
                                 const std::string& sysName = "BaseToTaskTransform")
        : System(sysName)
        , baseToTaskOut(this, &transformOutputValue) {
        if (em != NULL) {
            em->startManaging(*this);
        }

        Rtb.setIdentity();
    }

    virtual ~BaseToTaskTransform() {
        this->mandatoryCleanUp();
    }

 protected:
    typename Output<transform_type>::Value* transformOutputValue;
    transform_type Rtb;

    virtual void operate() {
        transformOutputValue->setData(&Rtb);
    }

 private:
    DISALLOW_COPY_AND_ASSIGN(BaseToTaskTransform);
};