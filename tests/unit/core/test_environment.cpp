#include <gtest/gtest.h>
#include "core/environment.hpp"

using namespace robot::core;

/// @brief Construction of the environment with default gravity
TEST(EnvironmentTest, ConstructWithDefaultGravity)
{
    EnvironmentConfig config;
    config.worldSize = {200.f, 200.f};
    config.timeStep = 0.01f;

    Environment env(config);

    EXPECT_FLOAT_EQ(env.gravity(), DefaultGravity);
}

/// @brief Construction of the environment with custom gravity
TEST(EnvironmentTest, ConstructWithCustomGravity)
{
    EnvironmentConfig config;
    config.worldSize = {10.f, 10.f};
    config.gravity = 3.721f; // Mars gravity
    config.timeStep = 0.01f;

    Environment env(config);

    EXPECT_FLOAT_EQ(env.gravity(), 3.721f);
}

/// @brief Verify that the worldsize value is accessible
TEST(EnvironmentTest, WorldSizeIsAccessible)
{
    EnvironmentConfig config;
    config.worldSize = {42.f, 24.f};
    config.timeStep = 0.01f;

    Environment env(config);

    EXPECT_FLOAT_EQ(env.worldSize().x, 42.f);
    EXPECT_FLOAT_EQ(env.worldSize().y, 24.f);
}

/// @brief Verify that the timestep value is accessible
TEST(EnvironmentTest, TimeStepIsAccessible)
{
    EnvironmentConfig config;
    config.worldSize = {10.f, 10.f};
    config.timeStep = 0.002f;

    Environment env(config);

    EXPECT_FLOAT_EQ(env.timeStep(), 0.002f);
}
