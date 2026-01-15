cmake_minimum_required( VERSION 3.20...4.2 )

project(robotics-dual-arm-simulation)

# PACKAGES
find_package(SFML 3 REQUIRED COMPONENTS Graphics Window System)

# LIBRARIES
add_library(shared-graphics SHARED src/graphics/plot2d.cpp)
target_link_libraries(shared-graphics PUBLIC SFML::Graphics SFML::Window SFML::System)
target_include_directories(shared-graphics PUBLIC ${PROJECT_SOURCE_DIR}/include PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# EXECUTABLES
add_executable(app apps/demo.cpp)
target_link_libraries(app shared-graphics)

# TESTS
add_executable(test-graphics tests/unit/graphics/test_plot2d.cpp)
target_link_libraries(test-graphics PRIVATE shared-graphics gtest gtest_main)

enable_testing()
add_test(NAME Plot2DTests COMMAND test-graphics)