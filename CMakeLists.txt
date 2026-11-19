cmake_minimum_required(VERSION 2.6)
project(contact_control)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Include project headers
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

## curses
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})

## Boost
# add iostreams for gnuplot-iostream
find_package(Boost 1.46 COMPONENTS system iostreams REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# -----------------------------
# Function: add_programs
# -----------------------------
function(add_programs)
  foreach(prog ${ARGV})
    add_executable(${prog} src/${prog}.cpp)
    target_link_libraries(${prog}
      ${BARRETT_LIBRARIES}
      ${CURSES_LIBRARIES}
    )
  endforeach()
endfunction()

# -----------------------------
# Executables (run on WAM)
# -----------------------------
add_programs(
  4dof_dyncomp
  4dof_gravcomp
  4dof_nominal_dyncomp
  4dof_nominal_gravcomp
  dynamics_excitation
  hybrid_control
  hybrid_fv_ft
  hybrid_fp_ft
  hybrid_cartesian_dynamics
  hybrid_cartesian_nominal
  hybrid_cartesian_static_check
)

add_executable(dyncomp_control src/dyncomp_control.cpp)
add_executable(comptorq_control src/comptorq_control.cpp)
target_link_libraries(dyncomp_control
  ${BARRETT_LIBRARIES}
  ${CURSES_LIBRARIES}
)
target_link_libraries(comptorq_control
  ${BARRETT_LIBRARIES}
  ${CURSES_LIBRARIES}
)

# Installed libbarrett/libconfig headers use pre-C++17 exception specifications.
set_property(TARGET dyncomp_control comptorq_control 4dof_dyncomp 4dof_gravcomp 4dof_nominal_dyncomp 4dof_nominal_gravcomp dynamics_excitation hybrid_control hybrid_fv_ft hybrid_fp_ft APPEND_STRING PROPERTY COMPILE_FLAGS " -std=c++14")
set_property(TARGET dyncomp_control comptorq_control APPEND PROPERTY COMPILE_DEFINITIONS
  JOINT_TRACKING_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
set_property(TARGET hybrid_fv_ft APPEND PROPERTY COMPILE_DEFINITIONS
  HYBRID_FV_FT_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/hybrid_fv_ft.conf")

set_property(TARGET hybrid_fp_ft APPEND PROPERTY COMPILE_DEFINITIONS
  HYBRID_FP_FT_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/hybrid_fp_ft.conf")

set_property(TARGET hybrid_cartesian_dynamics hybrid_cartesian_nominal hybrid_cartesian_static_check APPEND_STRING PROPERTY COMPILE_FLAGS " -std=c++14")
set_property(TARGET hybrid_cartesian_dynamics hybrid_cartesian_nominal APPEND PROPERTY COMPILE_DEFINITIONS
  HYBRID_CARTESIAN_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
set_property(TARGET hybrid_cartesian_dynamics APPEND PROPERTY COMPILE_DEFINITIONS
  HYBRID_CARTESIAN_DYNAMICS_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/hybrid_cartesian_dynamics.conf")
set_property(TARGET hybrid_cartesian_nominal APPEND PROPERTY COMPILE_DEFINITIONS
  HYBRID_CARTESIAN_NOMINAL_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/hybrid_cartesian_nominal.conf")
set_property(TARGET hybrid_cartesian_static_check APPEND PROPERTY COMPILE_DEFINITIONS
  HYBRID_CARTESIAN_STATIC_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

option(CONTACT_CONTROL_BUILD_TESTS "Build hardware-free controller tests" OFF)
if(CONTACT_CONTROL_BUILD_TESTS)
  enable_testing()
  add_executable(joint_tracking_log_test tests/joint_tracking_log.cpp)
  target_link_libraries(joint_tracking_log_test ${BARRETT_LIBRARIES})
  set_property(TARGET joint_tracking_log_test APPEND_STRING PROPERTY COMPILE_FLAGS " -std=c++14 -UNDEBUG")
  add_test(NAME joint_tracking_log COMMAND joint_tracking_log_test)
  add_executable(joint_computed_torque_test tests/joint_computed_torque.cpp)
  set_property(TARGET joint_computed_torque_test APPEND_STRING PROPERTY COMPILE_FLAGS " -std=c++14 -UNDEBUG")
  add_test(NAME joint_computed_torque COMMAND joint_computed_torque_test)
  add_executable(excitation_time_test tests/excitation_time.cpp)
  set_property(TARGET excitation_time_test APPEND_STRING PROPERTY COMPILE_FLAGS " -std=c++14 -UNDEBUG")
  add_test(NAME excitation_time COMMAND excitation_time_test)
  add_executable(force_low_pass_test tests/force_low_pass.cpp)
  target_link_libraries(force_low_pass_test ${BARRETT_LIBRARIES})
  set_property(TARGET force_low_pass_test APPEND_STRING PROPERTY COMPILE_FLAGS " -std=c++14 -UNDEBUG")
  add_test(NAME force_low_pass COMMAND force_low_pass_test)
  add_executable(velocity_reference_test tests/velocity_reference.cpp)
  target_link_libraries(velocity_reference_test ${BARRETT_LIBRARIES})
  set_property(TARGET velocity_reference_test APPEND_STRING PROPERTY COMPILE_FLAGS " -std=c++14 -UNDEBUG")
  add_test(NAME velocity_reference COMMAND velocity_reference_test)
  add_executable(force_position_test tests/force_position.cpp)
  target_link_libraries(force_position_test ${BARRETT_LIBRARIES})
  set_property(TARGET force_position_test APPEND_STRING PROPERTY COMPILE_FLAGS " -std=c++14 -UNDEBUG")
  add_test(NAME force_position COMMAND force_position_test
    "${CMAKE_CURRENT_SOURCE_DIR}/config/hybrid_fp_ft.conf")
  add_executable(planar_joint_torque_test tests/planar_joint_torque.cpp)
  target_link_libraries(planar_joint_torque_test ${BARRETT_LIBRARIES})
  set_property(TARGET planar_joint_torque_test APPEND_STRING PROPERTY COMPILE_FLAGS " -std=c++14 -UNDEBUG")
  add_test(NAME planar_joint_torque COMMAND planar_joint_torque_test)
  add_executable(controller_config_test tests/controller_config.cpp)
  target_link_libraries(controller_config_test ${BARRETT_LIBRARIES})
  set_property(TARGET controller_config_test APPEND_STRING PROPERTY COMPILE_FLAGS " -std=c++14")
  add_test(NAME controller_config COMMAND controller_config_test
    "${CMAKE_CURRENT_SOURCE_DIR}/config/hybrid_fv_ft.conf")
  add_executable(hybrid_cartesian_dynamics_test tests/hybrid_cartesian_dynamics.cpp)
  target_link_libraries(hybrid_cartesian_dynamics_test ${BARRETT_LIBRARIES})
  set_property(TARGET hybrid_cartesian_dynamics_test APPEND_STRING PROPERTY COMPILE_FLAGS " -std=c++14 -UNDEBUG")
  add_test(NAME hybrid_cartesian_dynamics_law COMMAND hybrid_cartesian_dynamics_test)
  set_tests_properties(hybrid_cartesian_dynamics_law PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config/slax")
  add_test(NAME hybrid_cartesian_dynamics_config COMMAND hybrid_cartesian_dynamics --check-config)
  add_test(NAME hybrid_cartesian_nominal_config COMMAND hybrid_cartesian_nominal --check-config)
  add_test(NAME hybrid_cartesian_static_prediction COMMAND hybrid_cartesian_static_check --predict-only)
  set_tests_properties(hybrid_cartesian_static_prediction PROPERTIES TIMEOUT 5)
endif()

# Plotting-only executable
# add_executable(compare_torque src/compare_torque.cpp)
# target_link_libraries(compare_torque ${Boost_LIBRARIES})
