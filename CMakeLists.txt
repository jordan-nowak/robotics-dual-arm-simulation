cmake_minimum_required( VERSION 3.20...4.2 )

project(robotics-dual-arm-simulation)

# PACKAGES
find_package(SFML 3 REQUIRED COMPONENTS Graphics Window System)

# LIBRARIES
add_library(shared-graphics SHARED src/graphics/plot2d.cpp)
target_link_libraries(shared-graphics PUBLIC SFML::Graphics SFML::Window SFML::System)
target_include_directories(shared-graphics PUBLIC ${PROJECT_SOURCE_DIR}/include PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(shared-core SHARED src/core/robot.cpp src/core/robot_config.cpp src/core/environment.cpp)
target_include_directories(shared-core PUBLIC ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/third_party/eigen PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(shared-modeling SHARED
    src/modeling/geometric/transform.cpp
    src/modeling/geometric/dh_model.cpp
    src/modeling/geometric/center_of_mass.cpp
    src/modeling/geometric/selection_matrix.cpp
    src/modeling/kinematic/jacobian_model.cpp
    src/modeling/kinematic/jacobian_com.cpp)

add_library(shared-control SHARED 
    src/control/pseudo_inverse.cpp)
target_link_libraries(shared-control PUBLIC shared-modeling shared-core)
target_include_directories(shared-control PUBLIC ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/third_party/eigen PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(shared-modeling PUBLIC shared-core)
target_include_directories(shared-modeling PUBLIC ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/third_party/eigen)

# EXECUTABLES
add_executable(app apps/demo.cpp)
target_link_libraries(app shared-graphics shared-core)

# TESTS
## control
add_executable(test-control-pseudo_inverse tests/unit/control/test_pseudo_inverse.cpp)
target_link_libraries(test-control-pseudo_inverse PRIVATE shared-control gtest gtest_main)

## graphics
add_executable(test-graphics tests/unit/graphics/test_plot2d.cpp)
target_link_libraries(test-graphics PRIVATE shared-graphics gtest gtest_main)

## core
add_executable(test-core-environment tests/unit/core/test_environment.cpp)
add_executable(test-core-robot tests/unit/core/test_robot.cpp)
add_executable(test-core-robot-config tests/unit/core/test_robot_config.cpp)
target_link_libraries(test-core-environment PRIVATE shared-core gtest gtest_main)
target_link_libraries(test-core-robot PRIVATE shared-core gtest gtest_main)
target_link_libraries(test-core-robot-config PRIVATE shared-core gtest gtest_main)

## modeling
add_executable (test-transform tests/unit/modeling/geometric/test_transform.cpp)
add_executable (test-dh-model tests/unit/modeling/geometric/test_dh_model.cpp)
add_executable (test-center-of-mass tests/unit/modeling/geometric/test_center_of_mass.cpp)
add_executable (test-selection-matrix tests/unit/modeling/geometric/test_selection_matrix.cpp)
add_executable (test-jacobian-model tests/unit/modeling/kinematic/test_jacobian_model.cpp)
add_executable (test-jacobian-com tests/unit/modeling/kinematic/test_jacobian_com.cpp)
target_link_libraries (test-transform PRIVATE shared-modeling gtest gtest_main)
target_link_libraries (test-center-of-mass PRIVATE shared-modeling shared-core gtest gtest_main)
target_link_libraries (test-selection-matrix PRIVATE shared-modeling shared-core gtest gtest_main)
target_link_libraries (test-dh-model PRIVATE shared-modeling gtest gtest_main)
target_link_libraries (test-jacobian-model PRIVATE shared-modeling gtest gtest_main)
target_link_libraries (test-jacobian-com PRIVATE shared-modeling gtest gtest_main)

## CMake testing with GoogleTest
enable_testing()
add_test(NAME Plot2DTests COMMAND test-graphics)
add_test(NAME EnvironmentTest COMMAND test-core-environment)
add_test(NAME RobotTest COMMAND test-core-robot)
add_test(NAME RobotConfigTest COMMAND test-core-robot-config)
add_test(NAME TransformTests COMMAND test-transform)
add_test(NAME DHModelTests COMMAND test-dh-model)
add_test(NAME CenterOfMassTests COMMAND test-center-of-mass)
add_test(NAME SelectionMatrixTests COMMAND test-selection-matrix)
add_test(NAME JacobianModelTests COMMAND test-jacobian-model)
add_test(NAME JacobianComTests COMMAND test-jacobian-com)
add_test(NAME PseudoInverseTests COMMAND test-control-pseudo_inverse)
