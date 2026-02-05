#include <gtest/gtest.h>
#include "core/robot_config.hpp"

using namespace robot::core;

/// @brief Construction of the robot 
TEST(RobotConfigTests, ConstructWithParameters)
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

    EXPECT_EQ(config.name(), "test_robot");
    EXPECT_EQ(config.jointCount(), 2);
    EXPECT_EQ(config.linkCount(), 2);
}

/// @brief Verify that the chain is invalid for multiple link attached to the base
TEST(RobotConfigTests, MultipleBases) {
    EXPECT_THROW( 
        RobotConfig config(
            "test_robot",
            {
                { .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}, 
                { .name = "Joint_1", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f},
                { .name = "Joint_2", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}
            },
            {
                { .name = "Link_0", .a = 0.f, .alpha = 0.f, .d = 0.f, .theta = 0.f, .mass = 1.0, .com = Eigen::Vector3d(0.0, 0.0, 0.0), .ID_link = 0, .ID_parent = -1, .motor = true}, 
                { .name = "Link_1", .a = 0.f, .alpha = 0.f, .d = 0.f, .theta = 0.f, .mass = 1.0, .com = Eigen::Vector3d(0.0, 0.0, 0.0), .ID_link = 1, .ID_parent = -1, .motor = true},
                { .name = "Link_2", .a = 0.f, .alpha = 0.f, .d = 0.f, .theta = 0.f, .mass = 1.0, .com = Eigen::Vector3d(0.0, 0.0, 0.0), .ID_link = 2, .ID_parent = 1, .motor = true}
            }
        ),
        std::runtime_error
    );
}

/// @brief Verify that all link isn't orphan
TEST(RobotConfigValidation, OrphanLink) {
    EXPECT_THROW( 
        RobotConfig config(
            "test_robot",
            {
                { .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}, 
                { .name = "Joint_1", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f},
                { .name = "Joint_2", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}
            },
            {
                { .name = "Link_0", .a = 0.f, .alpha = 0.f, .d = 0.f, .theta = 0.f, .mass = 1.0, .com = Eigen::Vector3d(0.0, 0.0, 0.0), .ID_link = 0, .ID_parent = -1, .motor = true}, 
                { .name = "Link_1", .a = 0.f, .alpha = 0.f, .d = 0.f, .theta = 0.f, .mass = 1.0, .com = Eigen::Vector3d(0.0, 0.0, 0.0), .ID_link = 1, .ID_parent = 0, .motor = true},
                { .name = "Link_2", .a = 0.f, .alpha = 0.f, .d = 0.f, .theta = 0.f, .mass = 1.0, .com = Eigen::Vector3d(0.0, 0.0, 0.0), .ID_link = 2, .ID_parent = 42, .motor = true}
            }
        ),
        std::runtime_error
    );
}

/// @brief Verify that the chain is invalid if a cycle is detected
TEST(RobotConfigValidation, CycleDetected) {
    EXPECT_THROW( 
        RobotConfig config(
            "test_robot",
            {
                { .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}, 
                { .name = "Joint_1", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f},
            },
            {
                { .name = "Link_0", .a = 0.f, .alpha = 0.f, .d = 0.f, .theta = 0.f, .mass = 1.0, .com = Eigen::Vector3d(0.0, 0.0, 0.0), .ID_link = 0, .ID_parent = 1, .motor = true}, 
                { .name = "Link_1", .a = 0.f, .alpha = 0.f, .d = 0.f, .theta = 0.f, .mass = 1.0, .com = Eigen::Vector3d(0.0, 0.0, 0.0), .ID_link = 1, .ID_parent = 0, .motor = true},
            }
        ),
        std::runtime_error
    );
}

/// @brief Verify that the configuration is accessible
TEST(RobotConfigTests, ConfigurationIsAccessible)
{
    RobotConfig config(
        "test_robot",
        {{ .name = "joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}},
        {{ .name = "link_0", .a = 1.f, .alpha = 2.f, .d = 3.f, .theta = 4.f, .mass = 5.0, .com = Eigen::Vector3d(0.0, 1.0, 2.0), .ID_link = 0, .ID_parent = -1, .motor = true}}
    );

    const auto& cfgJoints = config.joints();
    const auto& cfgLinks = config.links();

    ASSERT_EQ(cfgJoints.size(), 1);
    EXPECT_EQ(cfgJoints[0].name, "joint_0");
    EXPECT_FLOAT_EQ(cfgJoints[0].minPosition, -3.14f);
    EXPECT_FLOAT_EQ(cfgJoints[0].maxPosition, 3.14f);
    EXPECT_FLOAT_EQ(cfgJoints[0].maxVelocity, 1.0f);
    
    EXPECT_EQ(cfgLinks[0].name,"link_0");
    EXPECT_FLOAT_EQ(cfgLinks[0].a, 1.f);
    EXPECT_FLOAT_EQ(cfgLinks[0].alpha, 2.f);
    EXPECT_FLOAT_EQ(cfgLinks[0].d, 3.f);
    EXPECT_FLOAT_EQ(cfgLinks[0].theta, 4.f);
    EXPECT_FLOAT_EQ(cfgLinks[0].mass, 5.f);
    EXPECT_FLOAT_EQ(cfgLinks[0].com.x(), 0.0f);
    EXPECT_FLOAT_EQ(cfgLinks[0].com.y(), 1.0f);
    EXPECT_FLOAT_EQ(cfgLinks[0].com.z(), 2.0f);
    EXPECT_EQ(cfgLinks[0].ID_link, 0);
    EXPECT_EQ(cfgLinks[0].ID_parent, -1);
    EXPECT_EQ(cfgLinks[0].motor, true);
}

/// @brief Verify access via a constant reference (design guarantee)
TEST(RobotConfigTests, AccessThroughConstReference)
{
     RobotConfig config(
        "test_robot",
        {{ .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}},
        {{ .name = "Link_0", .a = 1.f, .alpha = 2.f, .d = 3.f, .theta = 4.f, .mass = 5.0, .com = Eigen::Vector3d(0.0, 1.0, 2.0), .ID_link = 0, .ID_parent = -1, .motor = true}}
    );

    EXPECT_EQ(config.name(), "test_robot");
    EXPECT_EQ(config.jointCount(), 1);
    EXPECT_EQ(config.linkCount(), 1);
}