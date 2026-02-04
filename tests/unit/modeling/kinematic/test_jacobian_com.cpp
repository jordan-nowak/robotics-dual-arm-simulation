#include <gtest/gtest.h>

#include "core/robot_config.hpp"
#include "modeling/geometric/dh_model.hpp"
#include "modeling/kinematic/jacobian_model.hpp"
#include "modeling/kinematic/jacobian_com.hpp"

using namespace robot::core;
using namespace robot::modeling::geometric;
using namespace robot::modeling::kinematic;

/// @brief Test JacobianCom construction from RobotConfig
TEST(JacobianComTest, ConstructfromModifiedDH) {
    RobotConfig config(
            "test_robot",
            {{ .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}},
            {{ .name = "Link_0", .a = 1.f, .alpha = 2.f, .d = 3.f, .theta = 4.f, .mass = 5.0, .com = Eigen::Vector3d(0.0, 1.0, 2.0), .ID_link = 0, .ID_parent = -1, .motor = true}}
        );

    JacobianCom Jconfig(config);

    SUCCEED(); // construction is OK
}

/// @brief Test Jacobian CoM dimensions (6xN)
TEST(JacobianComTest, JacobianHasCorrectDimensions) {
    RobotConfig config(
            "test_robot",
            {{ .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}},
            {{ .name = "Link_0", .a = 1.f, .alpha = 2.f, .d = 3.f, .theta = 4.f, .mass = 5.0, .com = Eigen::Vector3d(0.0, 1.0, 2.0), .ID_link = 0, .ID_parent = -1, .motor = true}}
        );

    JacobianCom Jconfig(config);

    std::vector<double> q = {0.f};

    Eigen::MatrixXd J = Jconfig.compute(q);

    EXPECT_EQ(J.rows(), 3); // x, y and z linear velocity
    EXPECT_EQ(J.cols(), config.jointCount());
}