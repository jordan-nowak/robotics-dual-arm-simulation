#include <gtest/gtest.h>
#include "core/robot.hpp"

using namespace robot::core;

/// @brief Construction of the robot from `RobotConfig`
TEST(RobotTest, ConstructFromConfiguration)
{
    RobotConfig config(
        "test_robot",
        {
            { .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}, 
            { .name = "Joint_1", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 2.0f}
        },
        {
            { .name = "Link_0", .a = 1.f, .alpha = 2.f, .d = 3.f, .theta = 4.f, .mass = 2.0, .com = Eigen::Vector3d(1.0, 0.0, 0.0), .ID_link = 0, .ID_parent = -1, .motor = true}, 
            { .name = "Link_1", .a = 4.f, .alpha = 5.f, .d = 6.f, .theta = 5.f, .mass = 1.0, .com = Eigen::Vector3d(0.0, 1.0, 0.0), .ID_link = 1, .ID_parent = 0, .motor = true}
        }
    );

    Robot robot(config);

    EXPECT_EQ(robot.config().name(), "test_robot");
    EXPECT_EQ(robot.config().jointCount(), 2);
    EXPECT_EQ(robot.config().linkCount(), 2);
}

/// @brief Verify that the state exists and is accessible in read only depending on the configuration.
TEST(RobotTest, ConstStateAccessIsReadOnly)
{
    RobotConfig config(
        "test_robot",
        {{ .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}},
        {{ .name = "Link_0", .a = 1.f, .alpha = 2.f, .d = 3.f, .theta = 4.f, .mass = 1.0, .com = Eigen::Vector3d(1.0, 0.0, 0.0), .ID_link = 0, .ID_parent = -1, .motor = true}}
    );

    Robot robot(config);
    const Robot& constRobot = robot;
    const RobotState& conststate = constRobot.state();

    EXPECT_EQ(conststate.jointPositions.size(), 1);
}

/// @brief Verify the read/write separation.
TEST(RobotTest, NonConstStateAccessAllowsModification)
{
    RobotConfig config(
        "test_robot",
        {{ .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}},
        {{ .name = "Link_0", .a = 1.f, .alpha = 2.f, .d = 3.f, .theta = 4.f, .mass = 1.0, .com = Eigen::Vector3d(1.0, 0.0, 0.0), .ID_link = 0, .ID_parent = -1, .motor = true}}
    );

    Robot robot(config);

    RobotState& state = robot.state();
    state.jointPositions[0] = 0.5f;

    EXPECT_FLOAT_EQ(robot.state().jointPositions[0], 0.5f);
}

