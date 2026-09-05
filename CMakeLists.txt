cmake_minimum_required(VERSION 3.16)
project(gz_humanoid_walking VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

find_package(gz-sim REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(Eigen3 REQUIRED)
# Pinocchio depends on Boost filesystem and serialization; discover via BoostConfig.cmake (Boost 1.90)
set(Boost_NO_BOOST_CMAKE OFF)
find_package(Boost REQUIRED COMPONENTS filesystem serialization)
# Pinocchio is vendored as a submodule under vendored/pinocchio and built as a separate CMake package
find_package(pinocchio REQUIRED)

include(CTest)

# Include directories
include_directories(
  src
  ${CMAKE_CURRENT_BINARY_DIR}/generated_include
  vendored/eiquadprog/include
  ${EIGEN3_INCLUDE_DIRS}
)

# Generate eiquadprog export/deprecation macros in binary dir so vendored submodule stays completely clean
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated_include/eiquadprog)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/generated_include/eiquadprog/config.hpp
  "#pragma once\n#define EIQUADPROG_DLLEXPORT\n#define EIQUADPROG_DLLIMPORT\n#define EIQUADPROG_DLLLOCAL\n#define EIQUADPROG_DLLAPI\n"
)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/generated_include/eiquadprog/deprecated.hpp
  "#pragma once\n#define EIQUADPROG_DEPRECATED [[deprecated]]\n#define EIQUADPROG_DEPRECATED_MESSAGE(msg) [[deprecated(msg)]]\n"
)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/generated_include/eiquadprog/warning.hpp
  "#pragma once\n#define EIQUADPROG_WARN(msg)\n"
)

# Vendored eiquadprog target (built directly as a static library to avoid JRL CMake packaging/FetchContent overhead)
add_library(eiquadprog STATIC
  vendored/eiquadprog/src/eiquadprog.cpp
  vendored/eiquadprog/src/eiquadprog-fast.cpp
)
set_target_properties(eiquadprog PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(eiquadprog PRIVATE "-Wno-sign-conversion" "-Wno-conversion")
target_link_libraries(eiquadprog PUBLIC Eigen3::Eigen)

# Core Walking & Kinematics Library
add_library(${PROJECT_NAME}_core
  src/LegIK.cc
  src/LIPMGenerator.cc
  src/WholeBodyQPController.cc
  src/PinocchioDynamicsWrapper.cc
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${PROJECT_NAME}_core
  eiquadprog
  Eigen3::Eigen
  pinocchio::pinocchio
)

# Gazebo Sim System Plugin
add_library(${PROJECT_NAME}_system SHARED
  src/HumanoidWalkingSystem.cc
)
target_link_libraries(${PROJECT_NAME}_system
  ${PROJECT_NAME}_core
  gz-sim::gz-sim
  gz-transport::gz-transport
  gz-msgs::gz-msgs
  gz-plugin::gz-plugin
  Eigen3::Eigen
  eiquadprog
  pinocchio::pinocchio
)

# Install plugin library
install(TARGETS ${PROJECT_NAME}_system ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install models, worlds, and scripts
install(DIRECTORY models worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

# Unit Tests
if(BUILD_TESTING)
  add_executable(test_leg_ik test/test_leg_ik.cc)
  target_link_libraries(test_leg_ik
    ${PROJECT_NAME}_core
    pinocchio::pinocchio
  )
  add_test(NAME test_leg_ik COMMAND test_leg_ik)

  add_executable(test_lipm_generator test/test_lipm_generator.cc)
  target_link_libraries(test_lipm_generator
    ${PROJECT_NAME}_core
    pinocchio::pinocchio
  )
  add_test(NAME test_lipm_generator COMMAND test_lipm_generator)

  add_executable(test_pinocchio_dynamics test/test_pinocchio_dynamics.cc)
  target_link_libraries(test_pinocchio_dynamics
    ${PROJECT_NAME}_core
    pinocchio::pinocchio
  )
  add_test(NAME test_pinocchio_dynamics COMMAND test_pinocchio_dynamics)
endif()
