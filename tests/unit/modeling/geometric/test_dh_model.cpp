#include <gtest/gtest.h>
#include "core/robot_config.hpp"
#include "modeling/geometric/dh_model.hpp"

using namespace robot::modeling::geometric;
using namespace robot::core;

/// @brief This test verifies the DHModel can be constructed with a valid config
TEST(DHModelTest, ConstructWithValidRobotConfig) {
    RobotConfig config("test_robot", {}, {});
    EXPECT_NO_THROW(DHModel model(config));
}

/// @brief Verify DH transformation for a single link with pure translation
TEST(DHModelTest, SingleLinkPureTranslation) {
    RobotConfig config(
        "test_robot",
        {{"j1",-3.14f,3.14f,1}},
        {{"l1",1.0f,0.0f,0.0f,0.0f}}
    );

    DHModel model(config);
    
    std::vector<double> jointPositions = {0.0f};

    Transform T = model.baseToEndEffector(jointPositions);
    const auto& M = T.matrix();

    EXPECT_NEAR(M(0,0), 1.f, 1e-6);
    EXPECT_NEAR(M(1,1), 1.f, 1e-6);
    EXPECT_NEAR(M(2,2), 1.f, 1e-6);
    EXPECT_NEAR(M(3,3), 1.f, 1e-6);

    EXPECT_NEAR(M(0,3), 1.0f, 1e-6);
    EXPECT_NEAR(M(1,3), 0.0f, 1e-6);
    EXPECT_NEAR(M(2,3), 0.0f, 1e-6);
}

/// @brief Verify DH transformation for a two-link chain
TEST(DHModelTest, TwoLinksChain) {
    RobotConfig config(
        "test_robot",
        {{"j1",-3.14f,3.14f,1}, {"j2",-3.14f,3.14f,1}},
        {{"l1",1,0,0,0}, {"l2",1,0,0,0}}
    );

    DHModel model(config);
    Transform T = model.baseToEndEffector({0.f, 0.f});

    EXPECT_NEAR(T.matrix()(0,3), 2.f, 1e-6);
}

/// @brief Mismatched joint positions size throws exception
TEST(DHModelTest, ThrowsIfJointVectorSizeMismatch) {
    RobotConfig config(
        "test_robot",
        {{"j1",-3.14f,3.14f,1}, {"j2",-3.14f,3.14f,1}},
        {{"l1",1,0,0,0}, {"l2",1,0,0,0}}
    );

    DHModel model(config);
    
    std::vector<double> jointPositions = {0.f}; // Wrong size
    
    EXPECT_THROW(
        model.baseToEndEffector(jointPositions),
        std::invalid_argument
    );
}

/// @brief Invalid link index throws exception
TEST(DHModelTest, ThrowsIfLinkIndexOutOfRange) {
    RobotConfig config(
        "test_robot",
        {{"j1",-3.14f,3.14f,1}, {"j2",-3.14f,3.14f,1}},
        {{"l1",1,0,0,0}, {"l2",1,0,0,0}}
    );

    DHModel model(config);
    
    std::vector<double> jointPositions = {0.f, 0.f};
    
    EXPECT_THROW(
        model.baseToLink(999, jointPositions),
        std::out_of_range
    );
}

/// @brief Verify that frames() returns correct number of frames
TEST(DHModelTest, FramesCount) {
    RobotConfig config(
        "test_robot",
        {{"j1",-3.14f,3.14f,1}, {"j2",-3.14f,3.14f,1}},
        {{"l1",1,0,0,0}, {"l2",1,0,0,0}}
    );

    DHModel model(config);

    auto frames = model.frames({0.f, 0.f});

    EXPECT_EQ(frames.size(), config.linkCount());
}