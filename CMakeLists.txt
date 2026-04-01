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
  hybrid_control
)

# Plotting-only executable
# add_executable(compare_torque src/compare_torque.cpp)
# target_link_libraries(compare_torque ${Boost_LIBRARIES})