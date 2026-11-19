#include "hybrid_cartesian_dynamics.hpp"
#include "hybrid_cartesian_program.hpp"

#define BARRETT_SMF_VALIDATE_ARGS
#define BARRETT_SMF_DONT_WAIT_FOR_SHIFT_ACTIVATE
#include <barrett/standard_main_function.h>

namespace {
constexpr hybrid_cartesian_program::Program program{
    "hybrid_cartesian_dynamics", "identified beta",
    HYBRID_CARTESIAN_DYNAMICS_CONFIG, HYBRID_CARTESIAN_DATA_DIR
};
hybrid_cartesian_program::Settings settings;
} // namespace

bool validate_args(int argc, char** argv) {
    return hybrid_cartesian_program::validateArgs(argc, argv, program, settings);
}

template <size_t DOF>
int wam_main(int, char**, barrett::ProductManager& pm, barrett::systems::Wam<DOF>& wam) {
    return hybrid_cartesian_program::runController<hybrid_cartesian_dynamics::PlanarDynamics>(
        pm, wam, program, settings);
}
