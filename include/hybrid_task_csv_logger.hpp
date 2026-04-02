#pragma once

#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

/**
 * Samples TupleGrouper output at the same rate as PeriodicDataLogger (periodMultiplier)
 * and writes one CSV row per sample: v_des(3), v_cur(3), f_des(3), f_cur(3) task frame.
 */
template <typename RecordTuple>
class HybridTaskCsvLogger : public barrett::systems::System,
                            public barrett::systems::SingleInput<RecordTuple> {
 public:
  HybridTaskCsvLogger(barrett::systems::ExecutionManager* em,
                        const std::string& path,
                        size_t periodMultiplier = 1,
                        const std::string& sysName = "HybridTaskCsvLogger")
      : barrett::systems::System(sysName),
        barrett::systems::SingleInput<RecordTuple>(this),
        logging_(true),
        ecCount_(0),
        ecCountRollover_(periodMultiplier == 0 ? 1 : periodMultiplier) {
    out_.open(path.c_str());
    if (!out_) {
      throw std::runtime_error("HybridTaskCsvLogger: failed to open " + path);
    }
    out_ << std::fixed << std::setprecision(8);
    out_ << "vdes_x,vdes_y,vdes_z,vcur_x,vcur_y,vcur_z,"
            "fdes_x,fdes_y,fdes_z,fcur_x,fcur_y,fcur_z\n";
    out_.flush();
    if (em != NULL) {
      em->startManaging(*this);
    }
  }

  virtual ~HybridTaskCsvLogger() {
    this->mandatoryCleanUp();
    closeFile();
  }

  void closeFile() {
    if (out_.is_open()) {
      out_.flush();
      out_.close();
    }
    logging_ = false;
  }

 protected:
  virtual bool inputsValid() {
    ecCount_ = (ecCount_ + 1) % ecCountRollover_;
    return logging_ && ecCount_ == 0 && this->input.valueDefined();
  }

  virtual void operate() {
    const RecordTuple& rec = this->input.getValue();
    writeVec3(out_, rec.template get<0>());
    out_ << ',';
    writeVec3(out_, rec.template get<1>());
    out_ << ',';
    writeVec3(out_, rec.template get<2>());
    out_ << ',';
    writeVec3(out_, rec.template get<3>());
    out_ << '\n';
  }

  virtual void invalidateOutputs() {}

 private:
  template <typename Vec3>
  static void writeVec3(std::ofstream& o, const Vec3& v) {
    o << v[0] << ',' << v[1] << ',' << v[2];
  }

  std::ofstream out_;
  bool logging_;
  size_t ecCount_;
  size_t ecCountRollover_;

  DISALLOW_COPY_AND_ASSIGN(HybridTaskCsvLogger);
};
