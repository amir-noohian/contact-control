#pragma once

#include <barrett/systems.h>
#include <barrett/detail/ca_macro.h>
#include <string>

template <typename T>
class SampleDelay : public barrett::systems::System {
  public:
    Input<T> input;
    Output<T> output;

    explicit SampleDelay(barrett::systems::ExecutionManager* em,
                         const T& initialValue,
                         const std::string& sysName = "SampleDelay")
        : barrett::systems::System(sysName)
        , input(this)
        , output(this, &outputValue_)
        , currentOutput_(initialValue)
        , previousValue_(initialValue)
    {
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~SampleDelay() {
        this->mandatoryCleanUp();
    }

    void reset(const T& value) {
        currentOutput_ = value;
        previousValue_ = value;
        if (outputValue_ != NULL) {
            outputValue_->setData(&currentOutput_);
        }
    }

  protected:
    typename Output<T>::Value* outputValue_;

    T currentOutput_;
    T previousValue_;

    virtual void operate() {
        // output previous sample
        currentOutput_ = previousValue_;
        outputValue_->setData(&currentOutput_);

        // store current input for next cycle
        if (input.valueDefined()) {
            previousValue_ = input.getValue();
        }
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(SampleDelay);
};