cmake_minimum_required( VERSION 3.20...4.2 )

project(robotics-dual-arm-simulation)

# PACKAGES
find_package(SFML 3 REQUIRED COMPONENTS Graphics Window System)

# LIBRARIES
add_library(shared-graphics SHARED src/graphics/plot2d.cpp)
target_link_libraries(shared-graphics PUBLIC SFML::Graphics SFML::Window SFML::System)
target_include_directories(shared-graphics PUBLIC ${PROJECT_SOURCE_DIR}/include PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(shared-core SHARED src/core/robot.cpp src/core/robot_config.cpp src/core/environment.cpp)
target_include_directories(shared-core PUBLIC ${PROJECT_SOURCE_DIR}/include PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# EXECUTABLES
add_executable(app apps/demo.cpp)
target_link_libraries(app shared-graphics shared-core)

# TESTS
add_executable(test-graphics tests/unit/graphics/test_plot2d.cpp)
target_link_libraries(test-graphics PRIVATE shared-graphics gtest gtest_main)

add_executable(test-core-environment tests/unit/core/test_environment.cpp)
target_link_libraries(test-core-environment PRIVATE shared-core gtest gtest_main)

add_executable(test-core-robot tests/unit/core/test_robot.cpp)
target_link_libraries(test-core-robot PRIVATE shared-core gtest gtest_main)

add_executable(test-core-robot-config tests/unit/core/test_robot_config.cpp)
target_link_libraries(test-core-robot-config PRIVATE shared-core gtest gtest_main)

enable_testing()
add_test(NAME Plot2DTests COMMAND test-graphics)
add_test(NAME EnvironmentTest COMMAND test-core-environment)
add_test(NAME RobotTest COMMAND test-core-robot)
add_test(NAME RobotConfigTest COMMAND test-core-robot-config)