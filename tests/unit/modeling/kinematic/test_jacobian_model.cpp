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
        {"simple_robot"},
        {{"joint_1", -3.14f, 3.14f, 2.f}},
        {{"link_1", 1.f, 0.f, 0.f, 0.f, 0.f, 0.f}}
    );

    JacobianModel J(config);

    SUCCEED(); // construction is OK
}

/// @brief Test Jacobian dimensions (6xN)
TEST(JacobianModelTest, JacobianHasCorrectDimensions) {
    RobotConfig config(
        {"simple_robot"},
        {{"joint_1", -3.14f, 3.14f, 2.f}},
        {{"link_1", 1.f, 0.f, 0.f, 0.f, 0.f, 0.f}}
    );

    JacobianModel jacobian(config);

    std::vector<float> q = {0.f};

    Eigen::MatrixXf J = jacobian.compute(q);

    EXPECT_EQ(J.rows(), 6);
    EXPECT_EQ(J.cols(), config.jointCount());
}