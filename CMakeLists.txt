project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/MpcSolver.cpp src/MpcEvaluator.cpp
            src/ModelPredictiveController.cpp src/main.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
#link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc ${sources})

target_link_libraries(mpc ipopt z ssl uv uWS)

# Makes boolean 'test' available
option(test "Build all tests" OFF)
# Testing
# ------------------------------------------------------------------------------
if (test)
  # Enable ExternalProject CMake module
  include(ExternalProject)

  # Download and install GoogleTest
  ExternalProject_Add(
    gtest
    URL https://github.com/google/googletest/archive/release-1.8.0.tar.gz
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gtest
    # Disable install step
    INSTALL_COMMAND ""
  )

  # Get GTest source and binary directories from CMake project
  ExternalProject_Get_Property(gtest source_dir binary_dir)

  # Create a libgtest target to be used as a dependency by test programs
  add_library(libgtest IMPORTED STATIC GLOBAL)
  add_dependencies(libgtest gtest)

  # Set libgtest properties
  set_target_properties(libgtest PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/gtest/libgtest.a"
  )

  # Create a libgmock target to be used as a dependency by test programs
  add_library(libgmock IMPORTED STATIC GLOBAL)
  add_dependencies(libgmock gtest)

  # Set libgmock properties
  set_target_properties(libgmock PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/libgmock.a"
  )

  include_directories("${source_dir}/googletest/include"
                      "${source_dir}/googlemock/include")

  # Components under test
  # ----------------------------------------------------------------------------
  add_library(mpc_evaluator_lib src/MpcEvaluator.cpp)
  add_library(mpc_solver_lib src/MpcSolver.cpp)

  enable_testing()

  # Unit Tests
  # ----------------------------------------------------------------------------
  add_executable(test_mpc_evaluator test/TestMpcEvaluator.cpp)
  add_executable(test_mpc_solver test/TestMpcSolver.cpp)

  # Standard linking to gtest stuff
  target_link_libraries(test_mpc_evaluator libgtest)
  target_link_libraries(test_mpc_solver libgtest)

  # Extra linking for the project
  target_link_libraries(test_mpc_evaluator mpc_evaluator_lib)
  target_link_libraries(test_mpc_solver mpc_solver_lib mpc_evaluator_lib ipopt)

  # Make tests running through 'make test'
  add_test(NAME test_mpc_evaluator COMMAND test_mpc_evaluator)
  add_test(NAME test_mpc_solver COMMAND test_mpc_solver)
endif()
