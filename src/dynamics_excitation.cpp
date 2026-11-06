#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include "excitation_time.hpp"

#include <Eigen/Core>
#include <barrett/log.h>
#include <barrett/os.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <barrett/systems.h>
#include <barrett/units.h>

using namespace barrett;

template <size_t DOF> class FourierTrajectory : public systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<double> timeInput;
    Input<jp_type> feedbackInput;
    Output<jv_type> referenceOutput;

    FourierTrajectory()
        : timeInput(this), feedbackInput(this), referenceOutput(this, &outputValue), faulted(false), is_active(false),
          max_duration(60.0), fault_joint(0), fault_position(0.0), velocity_command(0.0) {
        if (DOF == 7) {
            lower_bounds[0] = -2.4;
            upper_bounds[0] = 2.4;
            lower_bounds[1] = -1.8;
            upper_bounds[1] = 1.8;
            lower_bounds[2] = -2.5;
            upper_bounds[2] = 2.5;
            lower_bounds[3] = -0.8;
            upper_bounds[3] = 3.0;
            lower_bounds[4] = -0.5;
            upper_bounds[4] = 0.5;
            lower_bounds[5] = -0.5;
            upper_bounds[5] = 0.5;
            lower_bounds[6] = -0.5;
            upper_bounds[6] = 0.5;
        } else if (DOF == 4) {
            lower_bounds[0] = -2.4;
            upper_bounds[0] = 2.4;
            lower_bounds[1] = -1.8;
            upper_bounds[1] = 1.8;
            lower_bounds[2] = -2.5;
            upper_bounds[2] = 2.5;
            lower_bounds[3] = -0.8;
            upper_bounds[3] = 3.0;
        } else if (DOF == 8) {
            lower_bounds[0] = -2.4;
            upper_bounds[0] = 2.4;
            lower_bounds[1] = -1.8;
            upper_bounds[1] = 1.8;
            lower_bounds[2] = -2.5;
            upper_bounds[2] = 2.5;
            lower_bounds[3] = -0.8;
            upper_bounds[3] = 3.0;
            lower_bounds[4] = -0.5;
            upper_bounds[4] = 0.5;
            lower_bounds[5] = -0.5;
            upper_bounds[5] = 0.5;
            lower_bounds[6] = -0.5;
            upper_bounds[6] = 0.5;
            lower_bounds[7] = -0.5;
            upper_bounds[7] = 0.5;
        } else {
            throw std::logic_error("FATAL: FourierTrajectory is strictly defined for 4, 7 and 8 DOF.");
        }

        static const double a_b_q0_full[77] = {
            -0.22453075, -0.06509415, 0.0084825,   0.7463596,   0.04482256,  0.15292831,  0.17945343,  0.13698774,
            0.15005761,  0.40428706,  -0.06912397, 0.64039236,  0.41406122,  -0.05345818, 0.46419411,  0.27601025,
            0.55370932,  0.065848,    0.04071695,  0.41272949,  -0.17373824, 0.89269347,  0.45028845,  -0.26077073,
            0.10794414,  -0.16322598, 0.22051963,  -0.13005116, 0.38388683,  0.93214031,  0.36874565,  0.11058303,
            -0.13207445, 0.39799543,  0.28705749,  0.09974301,  -0.09950086, 0.77226083,  -0.28470714, 0.06468834,
            0.0187677,   -0.05244391, -0.40604485, 0.43133299,  -0.57178278, -0.12185623, 0.51881781,  0.27173848,
            0.15961459,  -0.29374854, 0.2251367,   -0.29917266, 0.09730888,  0.24762081,  0.07477825,  -0.11031733,
            0.08532278,  0.21811307,  -0.10056108, 0.56254872,  -0.02235595, 0.34816564,  -0.24721517, -0.1987875,
            -0.29918626, 0.19194917,  -0.39180923, -0.60461852, 0.00473514,  -0.24540649, -0.24181945, 0.03114698,
            -0.08510632, 1.54641938,  -1.67238949, -0.17570127, 0.5569734};

        typedef Eigen::Matrix<double, K_used, L, Eigen::RowMajor> MatrixKL;
        A = Eigen::Map<const MatrixKL>(a_b_q0_full);
        B = Eigen::Map<const MatrixKL>(a_b_q0_full + OFFSET_B);
        q0 = Eigen::Map<const Eigen::Matrix<double, K_used, 1>>(a_b_q0_full + OFFSET_Q0);

        // NOTE: Reduce factor if joint out of bounds or velocity limit
        factors << 1.0, 1.0, 1.0, 0.7;
        for (int l = 0; l < L; ++l) {
            omegas(l) = (l + 1) * 0.314;
        }
    }
    ~FourierTrajectory() { this->mandatoryCleanUp(); }

    void start(double duration) {
        if (!std::isfinite(duration) || duration < 2.0 * transition_duration) {
            throw std::invalid_argument("Excitation duration must be at least four seconds.");
        }
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        max_duration = duration;
        is_active = true;
        faulted = false;
        fault_joint = 0;
    }

    bool isActive() const {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        return is_active;
    }

    size_t getFault(double &position) const {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        position = fault_position;
        return fault_joint;
    }

    void evaluateTrajectory(double t, jp_type &q_full, jv_type &qdot_full) {
        Eigen::Matrix<double, L, 1> C;
        Eigen::Matrix<double, L, 1> S;
        Eigen::Matrix<double, L, 1> C_w;
        Eigen::Matrix<double, L, 1> S_w;

        for (int l = 0; l < L; ++l) {
            double w = omegas(l);
            double wt = w * t;
            double cos_wt = std::cos(wt);
            double sin_wt = std::sin(wt);
            C(l) = cos_wt;
            S(l) = sin_wt;

            C_w(l) = cos_wt / w;
            S_w(l) = sin_wt / w;
        }
        Eigen::Matrix<double, K_used, 1> q_base = A * S_w - B * C_w;
        Eigen::Matrix<double, K_used, 1> qdot_base = A * C + B * S;

        q_full.setZero();
        qdot_full.setZero();

        q_full.template head<K_used>() = (q0.array() + q_base.array()) * factors.array();
        qdot_full.template head<K_used>() = qdot_base.array() * factors.array();
    }

  protected:
    void operate() override {
        // Latch the fault state to prevent resuming if the arm bounces back into bounds
        if (faulted || !is_active) {
            outputValue->setData(&zero_vel);
            return;
        }

        const jp_type &current_pos = feedbackInput.getValue();
        for (size_t i = 0; i < DOF; i++) {
            if (!std::isfinite(current_pos[i]) || current_pos[i] > (upper_bounds[i] + 0.05) ||
                current_pos[i] < (lower_bounds[i] - 0.05)) {
                fault_joint = i + 1;
                fault_position = current_pos[i];
                faulted = true;
                is_active = false;
                outputValue->setData(&zero_vel);
                return;
            }
        }

        double t = timeInput.getValue();

        if (t >= max_duration) {
            is_active = false;
            outputValue->setData(&zero_vel);
            return;
        }

        jp_type q;
        // setData retains a pointer: the command must outlive this callback.
        const excitation::TimeSample sample = excitation::sampleTime(t, max_duration, transition_duration);
        evaluateTrajectory(sample.phase, q, velocity_command);
        velocity_command *= sample.rate;

        outputValue->setData(&velocity_command);
    }

  private:
    typename Output<jv_type>::Value *outputValue;
    bool faulted;
    bool is_active;
    double max_duration;
    size_t fault_joint;
    double fault_position;
    jv_type velocity_command;

    jp_type lower_bounds;
    jp_type upper_bounds;
    const jv_type zero_vel = jv_type(0.0);
    static constexpr double transition_duration = 2.0;

    static constexpr int K_full = 7;
    static constexpr int K_used = 4;
    static constexpr int L = 5;

    static constexpr int OFFSET_B = K_full * L;
    static constexpr int OFFSET_Q0 = 2 * K_full * L;

    Eigen::Vector4d factors;
    Eigen::Matrix<double, K_used, L> A;
    Eigen::Matrix<double, K_used, L> B;
    Eigen::Matrix<double, K_used, 1> q0;
    Eigen::Matrix<double, L, 1> omegas;
};

template <size_t DOF> int wam_main(int argc, char **argv, ProductManager &pm, systems::Wam<DOF> &wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    std::string file_path = "rbtlog_4_dof.dat";
    if (argc > 1) {
        file_path = argv[1];
    }

    // Parse the directory component and create it
    size_t last_slash_idx = file_path.find_last_of('/');
    if (last_slash_idx != std::string::npos) {
        std::string dir_path = file_path.substr(0, last_slash_idx);
        std::string mkdir_cmd = "mkdir -p " + dir_path;
        if (system(mkdir_cmd.c_str()) != 0) {
            printf("Could not create directory structure %s\n", dir_path.c_str());
            return 1;
        }
    }

    // Initialize temporary binary file for real-time logging
    char tmpFile[] = "/tmp/btXXXXXX";
    if (mkstemp(tmpFile) == -1) {
        printf("ERROR: Couldn't create temporary file!\n");
        return 1;
    }

    wam.gravityCompensate();

    systems::Ramp time(pm.getExecutionManager(), 1.0);
    FourierTrajectory<DOF> trajGen;

    jp_type q_init;
    jv_type qdot_init;
    trajGen.evaluateTrajectory(0.0, q_init, qdot_init);

    wam.moveTo(q_init);
    btsleep(2.0);

    typedef boost::tuple<double, jp_type, jt_type> tuple_type;
    systems::TupleGrouper<double, jp_type, jt_type> tg;

    const size_t PERIOD_MULTIPLIER = 2;
    systems::PeriodicDataLogger<tuple_type> logger(
        pm.getExecutionManager(),
        new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
        PERIOD_MULTIPLIER);

    systems::connect(time.output, trajGen.timeInput);
    systems::connect(wam.jpOutput, trajGen.feedbackInput);

    systems::connect(time.output, tg.template getInput<0>());
    systems::connect(wam.jpOutput, tg.template getInput<1>());
    systems::connect(wam.jtSum.output, tg.template getInput<2>());

    printf("Press [Enter] to excite the wam");
    detail::waitForEnter();
    printf("Starting velocity excitation with 2 s startup/shutdown ramps and logging...\n");

    systems::connect(tg.output, logger.input);
    trajGen.start(60.0);
    wam.trackReferenceSignal(trajGen.referenceOutput);
    time.start();

    while (trajGen.isActive()) {
        btsleep(0.1);
    }

    printf("Stopping...\n");
    logger.closeLog();
    time.stop();

    double fault_position = 0.0;
    const size_t fault_joint = trajGen.getFault(fault_position);
    if (fault_joint != 0) {
        printf("Trajectory fault: Joint %zu at %.6f rad (out of bounds or non-finite).\n",
               fault_joint, fault_position);
    }
    wam.idle();
    if (fault_joint == 0) {
        wam.moveTo(jp_type(0.0));
        wam.moveHome();
    }

    log::Reader<tuple_type> lr(tmpFile);
    std::ofstream out(file_path);

    if (!out.is_open()) {
        printf("ERROR: Could not open %s for writing.\n", file_path.c_str());
    } else {
        out.precision(15);
        // NOTE: Temporarily save 4dof for demo. Use full DOF in future.
        const int DOF_WRITE = 4;

        for (size_t i = 0; i < lr.numRecords(); ++i) {
            tuple_type record = lr.getRecord();

            double t = boost::get<0>(record);
            jp_type pos = boost::get<1>(record);
            jt_type eff = boost::get<2>(record);

            out << t;
            for (size_t j = 0; j < DOF_WRITE; ++j)
                out << " " << pos[j];
            for (size_t j = 0; j < DOF_WRITE; ++j)
                out << " " << eff[j];
            out << "\n";
        }
        out.close();
        printf("Output written to %s.\n", file_path.c_str());
    }

    std::remove(tmpFile);
    wam.idle();

    return fault_joint == 0 ? 0 : 1;
}
