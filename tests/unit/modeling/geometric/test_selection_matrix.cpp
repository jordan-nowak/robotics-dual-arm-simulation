#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "core/robot_config.hpp"
#include "modeling/geometric/selection_matrix.hpp"

using namespace robot::modeling::geometric;

/**
 * @brief Test fixture for SelectionMatrix
 */
class SelectionMatrixTest : public ::testing::Test {
protected:
    std::unique_ptr<robot::core::RobotConfig> configtest;

    void SetUp() override {
        /*
            Structure:

                    0 (AB)
                   / \
                  1   3 -> link "3" without motor
                  |   |
                  2   4

            ID_parent = [ -1, 0, 1, 0, 3 ]
        */

        configtest = std::make_unique<robot::core::RobotConfig>(
            "test_robot",
            std::vector<robot::core::JointConfig>{
                { .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}, 
                { .name = "Joint_1", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f},
                { .name = "Joint_2", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f},
                { .name = "Joint_3", .minPosition = 0.0f,   .maxPosition = 0.0f,  .maxVelocity = 0.0f}, // no motor
                { .name = "Joint_4", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}
            },
            std::vector<robot::core::LinkConfig>{
                { .name = "AB", .a = 0, .alpha = 0, .d = 0, .theta = 0, .mass = 1.0, .com = Eigen::Vector3d::Zero(), .ID_link = 0, .ID_parent = -1, .motor = true}, 
                { .name = "A1", .a = 0, .alpha = 0, .d = 0, .theta = 0, .mass = 1.0, .com = Eigen::Vector3d::Zero(), .ID_link = 1, .ID_parent = 0, .motor = true}, 
                { .name = "A2", .a = 0, .alpha = 0, .d = 0, .theta = 0, .mass = 1.0, .com = Eigen::Vector3d::Zero(), .ID_link = 2, .ID_parent = 1, .motor = true}, 
                { .name = "B1", .a = 0, .alpha = 0, .d = 0, .theta = 0, .mass = 1.0, .com = Eigen::Vector3d::Zero(), .ID_link = 3, .ID_parent = 0, .motor = false}, 
                { .name = "B2", .a = 0, .alpha = 0, .d = 0, .theta = 0, .mass = 1.0, .com = Eigen::Vector3d::Zero(), .ID_link = 4, .ID_parent = 3, .motor = true}, 
            }
        );

        
    }
};

/**
 * @brief Test selection matrix from root to end-effector A
 */
TEST_F(SelectionMatrixTest, PathToEndEffectorA) {
    SelectionMatrix selector(*configtest);

    Eigen::MatrixXd selection_matrix = selector.fromEndEffector(2);

    Eigen::MatrixXd expected(5, 5);
    expected <<
        1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0;

    EXPECT_TRUE(selection_matrix.isApprox(expected));
}

/**
 * @brief Test selection matrix from root to end-effector B
 */
TEST_F(SelectionMatrixTest, PathToEndEffectorB) {
    SelectionMatrix selector(*configtest);

    Eigen::MatrixXd selection_matrix = selector.fromEndEffector(4);

    Eigen::MatrixXd expected(5, 5);
    expected <<
        1, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, // no motor on link "3"
        0, 0, 0, 0, 1;

    EXPECT_TRUE(selection_matrix.isApprox(expected));
}
