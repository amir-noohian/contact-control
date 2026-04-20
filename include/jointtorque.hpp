#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems.h>
#include <barrett/units.h>

template <size_t DOF>
class JointTorqueOutput : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Output<jt_type> output;

    explicit JointTorqueOutput(barrett::systems::Wam<DOF>& wam_,
                               barrett::systems::ExecutionManager* em,
                               const std::string& sysName = "JointTorqueOutput")
        : System(sysName)
        , wam(wam_)
        , output(this, &outputValue) {
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~JointTorqueOutput() {
        this->mandatoryCleanUp();
    }

  protected:
    barrett::systems::Wam<DOF>& wam;
    typename Output<jt_type>::Value* outputValue;
    jt_type tau;

    virtual void operate() {
        tau = wam.getJointTorques();
        outputValue->setData(&tau);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(JointTorqueOutput);
};