#include <gtest/gtest.h>
#include "core/robot_config.hpp"

using namespace robot::core;

/// @brief Construction of the robot 
TEST(RobotConfigTest, ConstructWithParameters)
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

    EXPECT_EQ(config.name(), "test_robot");
    EXPECT_EQ(config.jointCount(), 2);
    EXPECT_EQ(config.linkCount(), 2);
}

/// @brief Verify that the configuration is accessible
TEST(RobotConfigTest, ConfigurationIsAccessible)
{
    std::vector<JointConfig> joints = {
        {"joint_1", -1.0f, 1.0f, 2.0f}
    };
    
    std::vector<LinkConfig> links = {
        {"link_1", 1.f, 2.f, 3.f, 4.f, 5.f, 6.f},
    };

    RobotConfig config("test_robot", joints, links);

    const auto& cfgJoints = config.joints();
    const auto& cfgLinks = config.links();

    ASSERT_EQ(cfgJoints.size(), 1);
    EXPECT_EQ(cfgJoints[0].name, "joint_1");
    EXPECT_FLOAT_EQ(cfgJoints[0].minPosition, -1.0f);
    EXPECT_FLOAT_EQ(cfgJoints[0].maxPosition, 1.0f);
    EXPECT_FLOAT_EQ(cfgJoints[0].maxVelocity, 2.0f);
    
    EXPECT_EQ(cfgLinks[0].name,"link_1");
    EXPECT_FLOAT_EQ(cfgLinks[0].a, 1.f);
    EXPECT_FLOAT_EQ(cfgLinks[0].alpha, 2.f);
    EXPECT_FLOAT_EQ(cfgLinks[0].d, 3.f);
    EXPECT_FLOAT_EQ(cfgLinks[0].theta, 4.f);
    EXPECT_FLOAT_EQ(cfgLinks[0].mass, 5.f);
    EXPECT_FLOAT_EQ(cfgLinks[0].com, 6.f);
}

/// @brief Verify access via a constant reference (design guarantee)
TEST(RobotConfigTest, AccessThroughConstReference)
{
    std::vector<JointConfig> joints = {
        {"joint_1", -1.0f, 1.0f, 2.0f}
    };
    
    std::vector<LinkConfig> links = {
        {"link_1", 1.f, 2.f, 3.f, 4.f, 5.f, 6.f},
    };

    RobotConfig config("test_robot", joints, links);

    EXPECT_EQ(config.name(), "test_robot");
    EXPECT_EQ(config.jointCount(), 1);
    EXPECT_EQ(config.linkCount(), 1);
}
