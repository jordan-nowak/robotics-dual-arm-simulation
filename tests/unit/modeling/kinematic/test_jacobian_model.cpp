#include <gtest/gtest.h>

#include "core/robot_config.hpp"
#include "modeling/geometric/dh_model.hpp"
#include "modeling/kinematic/jacobian_model.hpp"

using namespace robot::core;
using namespace robot::modeling::geometric;
using namespace robot::modeling::kinematic;

/// @brief Test JacobianModel construction from RobotConfig
TEST(JacobianModelTest, ConstructfromModifiedDH) {
    RobotConfig config(
        "test_robot",
        {{ .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}},
        {{ .name = "Link_0", .a = 1.f, .alpha = 2.f, .d = 3.f, .theta = 4.f, .mass = 5.0, .com = Eigen::Vector3d(0.0, 1.0, 2.0), .ID_link = 0, .ID_parent = -1, .motor = true}}
    );

    JacobianModel Jconfig(config);

    SUCCEED(); // construction is OK
}

/// @brief Test Jacobian dimensions (6xN)
TEST(JacobianModelTest, JacobianHasCorrectDimensions) {
    RobotConfig config(
        "test_robot",
        std::vector<robot::core::JointConfig>{
            { .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}, 
            { .name = "Joint_1", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f},
            { .name = "Joint_2", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f},
            { .name = "Joint_3", .minPosition = 0.0f,   .maxPosition = 0.0f,  .maxVelocity = 1.0f},
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

    JacobianModel Jconfig(config);

    std::vector<double> q = {0.f, 0.f, 0.f, 0.f, 0.f};

    Eigen::MatrixXd J = Jconfig.compute(config.jointCount()-1, q);

    EXPECT_EQ(J.rows(), 6);
    EXPECT_EQ(J.cols(), config.jointCount());
}