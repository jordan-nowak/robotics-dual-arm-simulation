#include <gtest/gtest.h>
#include "core/robot.hpp"

using namespace robot::core;

/// @brief Construction of the robot from `RobotConfig`
TEST(RobotTest, ConstructFromConfiguration)
{
    std::vector<JointConfig> joints = {
        {"joint_1", -1.0f, 1.0f, 2.0f},
        {"joint_2", -2.0f, 2.0f, 3.0f}
    };

    std::vector<LinkConfig> links = {
        {"link_1", 1.f, 2.f, 3.f, 4.f, 5.f, 6.f},
        {"link_2", 7.f, 8.f, 9.f, 10.f, 11.f, 12.f}
    };

    RobotConfig config("test_robot", joints, links);
    Robot robot(config);

    EXPECT_EQ(robot.config().name(), "test_robot");
    EXPECT_EQ(robot.config().jointCount(), 2);
    EXPECT_EQ(robot.config().linkCount(), 2);
}

/// @brief Verify that the state exists and is accessible in read only depending on the configuration.
TEST(RobotTest, ConstStateAccessIsReadOnly)
{
    std::vector<JointConfig> joints = {
        {"joint_1", -1.0f, 1.0f, 2.0f},
        {"joint_2", -2.0f, 2.0f, 3.0f}
    };
    
    std::vector<LinkConfig> links = {
        {"link_1", 1.f, 2.f, 3.f, 4.f, 5.f, 6.f},
    };

    RobotConfig config("test_robot", joints, links);

    Robot robot(config);
    const Robot& constRobot = robot;
    const RobotState& conststate = constRobot.state();

    EXPECT_EQ(conststate.jointPositions.size(), 2);
}

/// @brief Verify the read/write separation.
TEST(RobotTest, NonConstStateAccessAllowsModification)
{
    std::vector<JointConfig> joints = {
        {"joint_1", -1.0f, 1.0f, 2.0f}
    };
    
    std::vector<LinkConfig> links = {
        {"link_1", 1.f, 2.f, 3.f, 4.f, 5.f, 6.f},
    };

    Robot robot(RobotConfig("test_robot", joints, links));

    RobotState& state = robot.state();
    state.jointPositions[0] = 0.5f;

    EXPECT_FLOAT_EQ(robot.state().jointPositions[0], 0.5f);
}

