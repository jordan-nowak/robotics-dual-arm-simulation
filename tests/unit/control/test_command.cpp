#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "control/command.hpp"

namespace robot::control::test {

// ============================================================================
// 0. TEST FIXTURE
// ============================================================================

/// @brief Base class for shared test configuration
class CommandTestBase {
    protected:
        double m_tol_zero = 1e-12;
        double m_tol_numerical = 1e-10;

    /// @brief Create a simple 3-DOF velocity command
    Command createVelocityCommand() {
        Eigen::VectorXd values(3);
        values << 1.0, 2.0, 3.0;
        return Command(values, Command::Type::VELOCITY);
    }

    /// @brief Create a simple 2-DOF position command
    Command createPositionCommand() {
        Eigen::VectorXd values(2);
        values << 0.5, -0.5;
        return Command(values, Command::Type::POSITION);
    }
};

class CommandTest : public CommandTestBase,
                    public ::testing::Test {
};

// ============================================================================
// 1. CONSTRUCTOR & INITIALIZATION
// ============================================================================

/// @brief Verify constructor stores values and type correctly
TEST_F(CommandTest, Constructor_InitializesCorrectly) {
    Eigen::VectorXd values(3);
    values << 1.0, 2.0, 3.0;
    
    Command cmd(values, Command::Type::VELOCITY);
    
    EXPECT_EQ(cmd.getType(), Command::Type::VELOCITY);
    EXPECT_EQ(cmd.getDimension(), 3);
    EXPECT_TRUE(cmd.getValues().isApprox(values, m_tol_zero));
}

/// @brief Verify default type is VELOCITY
TEST_F(CommandTest, Constructor_UsesDefaultType) {
    Eigen::VectorXd values(2);
    values << 0.5, -0.5;
    
    Command cmd(values);
    
    EXPECT_EQ(cmd.getType(), Command::Type::VELOCITY);
}

// ============================================================================
// 2. NORMAL CASES OF USE
// ============================================================================

/// @brief Verify getter returns the constructor values
TEST_F(CommandTest, GetValues_ReturnsConstructorValues) {
    Eigen::VectorXd values(4);
    values << 1.5, -2.3, 4.7, 0.0;
    
    Command cmd(values, Command::Type::POSITION);
    
    EXPECT_TRUE(cmd.getValues().isApprox(values, m_tol_zero));
}

/// @brief Verify getDimension returns correct size
TEST_F(CommandTest, GetDimension_ReturnsCorrectSize) {
    Eigen::VectorXd values(5);
    values << 1.0, 2.0, 3.0, 4.0, 5.0;
    
    Command cmd(values);
    
    EXPECT_EQ(cmd.getDimension(), 5);
}

/// @brief Verify scale multiplies all values by factor
TEST_F(CommandTest, Scale_MultipliesValuesByFactor) {
    Eigen::VectorXd values(3);
    values << 1.0, 2.0, 3.0;
    Command cmd(values);
    
    cmd.scale(2.5);
    
    Eigen::VectorXd expected(3);
    expected << 2.5, 5.0, 7.5;
    EXPECT_TRUE(cmd.getValues().isApprox(expected, m_tol_numerical));
}

// ============================================================================
// 3. LIMITS MANAGEMENT
// ============================================================================

/// @brief Verify setLimits stores limits correctly
TEST_F(CommandTest, SetLimits_StoresLimitsCorrectly) {
    Command cmd = createVelocityCommand();
    
    Eigen::VectorXd lower(3);
    lower << -1.0, -2.0, -3.0;
    Eigen::VectorXd upper(3);
    upper << 1.0, 2.0, 3.0;
    
    EXPECT_NO_THROW(cmd.setLimits(lower, upper));
}

/// @brief Verify setLimits throws if dimensions mismatch
TEST_F(CommandTest, SetLimits_ThrowsOnDimensionMismatch) {
    Command cmd = createVelocityCommand();
    
    Eigen::VectorXd lower(2);  // Wrong dimension
    lower << -1.0, -2.0;
    Eigen::VectorXd upper(3);
    upper << 1.0, 2.0, 3.0;
    
    EXPECT_THROW(cmd.setLimits(lower, upper), std::invalid_argument);
}

/// @brief Verify isValid returns true when no limits are set
TEST_F(CommandTest, IsValid_TrueWhenNoLimitsSet) {
    Command cmd = createVelocityCommand();
    
    EXPECT_TRUE(cmd.isValid());
}

/// @brief Verify isValid returns true when within limits
TEST_F(CommandTest, IsValid_TrueWhenWithinLimits) {
    Eigen::VectorXd values(3);
    values << 0.5, 1.0, -0.5;
    Command cmd(values);
    
    Eigen::VectorXd lower(3);
    lower << -1.0, -1.0, -1.0;
    Eigen::VectorXd upper(3);
    upper << 2.0, 2.0, 2.0;
    cmd.setLimits(lower, upper);
    
    EXPECT_TRUE(cmd.isValid());
}

/// @brief Verify isValid returns false when exceeding upper limit
TEST_F(CommandTest, IsValid_FalseWhenExceedingUpperLimit) {
    Eigen::VectorXd values(3);
    values << 0.5, 3.0, -0.5;  // Second value exceeds upper limit
    Command cmd(values);
    
    Eigen::VectorXd lower(3);
    lower << -1.0, -1.0, -1.0;
    Eigen::VectorXd upper(3);
    upper << 2.0, 2.0, 2.0;
    cmd.setLimits(lower, upper);
    
    EXPECT_FALSE(cmd.isValid());
}

/// @brief Verify isValid returns false when below lower limit
TEST_F(CommandTest, IsValid_FalseWhenBelowLowerLimit) {
    Eigen::VectorXd values(3);
    values << 0.5, 1.0, -2.0;  // Third value below lower limit
    Command cmd(values);
    
    Eigen::VectorXd lower(3);
    lower << -1.0, -1.0, -1.0;
    Eigen::VectorXd upper(3);
    upper << 2.0, 2.0, 2.0;
    cmd.setLimits(lower, upper);
    
    EXPECT_FALSE(cmd.isValid());
}

// ============================================================================
// 4. SATURATION
// ============================================================================

/// @brief Verify saturate returns false when no limits are set
TEST_F(CommandTest, Saturate_ReturnsFalseWhenNoLimitsSet) {
    Command cmd = createVelocityCommand();
    
    EXPECT_FALSE(cmd.saturate());
}

/// @brief Verify saturate returns false when already within limits
TEST_F(CommandTest, Saturate_ReturnsFalseWhenWithinLimits) {
    Eigen::VectorXd values(3);
    values << 0.5, 1.0, -0.5;
    Command cmd(values);
    
    Eigen::VectorXd lower(3);
    lower << -1.0, -1.0, -1.0;
    Eigen::VectorXd upper(3);
    upper << 2.0, 2.0, 2.0;
    cmd.setLimits(lower, upper);
    
    EXPECT_FALSE(cmd.saturate());
}

/// @brief Verify saturate clamps values exceeding upper limit
TEST_F(CommandTest, Saturate_ClampsUpperLimit) {
    Eigen::VectorXd values(3);
    values << 0.5, 3.0, -0.5;  // Second value exceeds upper limit
    Command cmd(values);
    
    Eigen::VectorXd lower(3);
    lower << -1.0, -1.0, -1.0;
    Eigen::VectorXd upper(3);
    upper << 2.0, 2.0, 2.0;
    cmd.setLimits(lower, upper);
    
    EXPECT_TRUE(cmd.saturate());
    
    Eigen::VectorXd expected(3);
    expected << 0.5, 2.0, -0.5;  // Second value clamped to 2.0
    EXPECT_TRUE(cmd.getValues().isApprox(expected, m_tol_zero));
}

/// @brief Verify saturate clamps values below lower limit
TEST_F(CommandTest, Saturate_ClampsLowerLimit) {
    Eigen::VectorXd values(3);
    values << 0.5, 1.0, -2.0;  // Third value below lower limit
    Command cmd(values);
    
    Eigen::VectorXd lower(3);
    lower << -1.0, -1.0, -1.0;
    Eigen::VectorXd upper(3);
    upper << 2.0, 2.0, 2.0;
    cmd.setLimits(lower, upper);
    
    EXPECT_TRUE(cmd.saturate());
    
    Eigen::VectorXd expected(3);
    expected << 0.5, 1.0, -1.0;  // Third value clamped to -1.0
    EXPECT_TRUE(cmd.getValues().isApprox(expected, m_tol_zero));
}

/// @brief Verify saturate clamps multiple values
TEST_F(CommandTest, Saturate_ClampsMultipleValues) {
    Eigen::VectorXd values(3);
    values << 5.0, -5.0, 0.0;  // First and second exceed limits
    Command cmd(values);
    
    Eigen::VectorXd lower(3);
    lower << -1.0, -1.0, -1.0;
    Eigen::VectorXd upper(3);
    upper << 2.0, 2.0, 2.0;
    cmd.setLimits(lower, upper);
    
    EXPECT_TRUE(cmd.saturate());
    
    Eigen::VectorXd expected(3);
    expected << 2.0, -1.0, 0.0;
    EXPECT_TRUE(cmd.getValues().isApprox(expected, m_tol_zero));
}

/// @brief Verify isValid returns true after saturation
TEST_F(CommandTest, Saturate_MakesCommandValid) {
    Eigen::VectorXd values(3);
    values << 5.0, -5.0, 0.0;
    Command cmd(values);
    
    Eigen::VectorXd lower(3);
    lower << -1.0, -1.0, -1.0;
    Eigen::VectorXd upper(3);
    upper << 2.0, 2.0, 2.0;
    cmd.setLimits(lower, upper);
    
    EXPECT_FALSE(cmd.isValid());
    cmd.saturate();
    EXPECT_TRUE(cmd.isValid());
}

// ============================================================================
// 5. PARAMETERIZED TESTS
// ============================================================================

struct CommandTypeTestCase {
    Command::Type type;
    std::string description;
};

class CommandTypeTest : public CommandTestBase,
                        public ::testing::TestWithParam<CommandTypeTestCase> {
};

/// @brief Verify command works correctly for all types
TEST_P(CommandTypeTest, Constructor_WorksForAllTypes) {
    auto testCase = GetParam();
    
    Eigen::VectorXd values(3);
    values << 1.0, 2.0, 3.0;
    
    Command cmd(values, testCase.type);
    
    EXPECT_EQ(cmd.getType(), testCase.type) << "Failed for " << testCase.description;
    EXPECT_EQ(cmd.getDimension(), 3) << "Failed for " << testCase.description;
    EXPECT_TRUE(cmd.getValues().isApprox(values, m_tol_zero)) << "Failed for " << testCase.description;
}

INSTANTIATE_TEST_SUITE_P(
    AllCommandTypes,
    CommandTypeTest,
    ::testing::Values(
        CommandTypeTestCase{Command::Type::VELOCITY, "Velocity command"},
        CommandTypeTestCase{Command::Type::POSITION, "Position command"},
        CommandTypeTestCase{Command::Type::ACCELERATION, "Acceleration command"}
    )
);

// ============================================================================

struct ScaleFactorTestCase {
    double factor;
    Eigen::VectorXd initial;
    Eigen::VectorXd expected;
    std::string description;
};

class CommandScaleTest : public CommandTestBase,
                         public ::testing::TestWithParam<ScaleFactorTestCase> {
};

/// @brief Verify scale works correctly with various factors
TEST_P(CommandScaleTest, Scale_VariousFactors) {
    auto testCase = GetParam();
    
    Command cmd(testCase.initial);
    cmd.scale(testCase.factor);
    
    EXPECT_TRUE(cmd.getValues().isApprox(testCase.expected, m_tol_numerical))
        << "Failed for " << testCase.description;
}

INSTANTIATE_TEST_SUITE_P(
    VariousScaleFactors,
    CommandScaleTest,
    ::testing::Values(
        ScaleFactorTestCase{1.0, Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(1.0, 2.0, 3.0), "Identity (factor=1)"},
        ScaleFactorTestCase{0.0, Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(0.0, 0.0, 0.0), "Zero (factor=0)"},
        ScaleFactorTestCase{2.0, Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(2.0, 4.0, 6.0), "Double (factor=2)"},
        ScaleFactorTestCase{0.5, Eigen::Vector3d(2.0, 4.0, 6.0), Eigen::Vector3d(1.0, 2.0, 3.0), "Half (factor=0.5)"},
        ScaleFactorTestCase{-1.0, Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(-1.0, -2.0, -3.0), "Negate (factor=-1)"},
        ScaleFactorTestCase{-2.5, Eigen::Vector3d(1.0, -2.0, 3.0), Eigen::Vector3d(-2.5, 5.0, -7.5), "Negative scale"}
    )
);

// ============================================================================
// 6. EDGE CASES & BOUNDARY CONDITIONS
// ============================================================================

/// @brief Verify command with zero dimension
TEST_F(CommandTest, Constructor_ZeroDimension) {
    Eigen::VectorXd values(0);

    EXPECT_THROW(Command cmd(values), std::invalid_argument);
}

/// @brief Verify command with single value
TEST_F(CommandTest, Constructor_SingleValue) {
    Eigen::VectorXd values(1);
    values << 5.0;
    
    Command cmd(values);
    
    EXPECT_EQ(cmd.getDimension(), 1);
    EXPECT_NEAR(cmd.getValues()(0), 5.0, m_tol_zero);
}

/// @brief Verify saturation at exact boundary (lower)
TEST_F(CommandTest, Saturate_ExactLowerBoundary) {
    Eigen::VectorXd values(1);
    values << -1.0;
    Command cmd(values);
    
    Eigen::VectorXd lower(1);
    lower << -1.0;
    Eigen::VectorXd upper(1);
    upper << 1.0;
    cmd.setLimits(lower, upper);
    
    EXPECT_FALSE(cmd.saturate());  // Already at boundary
    EXPECT_TRUE(cmd.isValid());
}

/// @brief Verify saturation at exact boundary (upper)
TEST_F(CommandTest, Saturate_ExactUpperBoundary) {
    Eigen::VectorXd values(1);
    values << 1.0;
    Command cmd(values);
    
    Eigen::VectorXd lower(1);
    lower << -1.0;
    Eigen::VectorXd upper(1);
    upper << 1.0;
    cmd.setLimits(lower, upper);
    
    EXPECT_FALSE(cmd.saturate());  // Already at boundary
    EXPECT_TRUE(cmd.isValid());
}

/// @brief Verify scale with very small factor
TEST_F(CommandTest, Scale_VerySmallFactor) {
    Eigen::VectorXd values(2);
    values << 1000.0, -1000.0;
    Command cmd(values);
    
    cmd.scale(1e-10);
    
    Eigen::VectorXd expected(2);
    expected << 1e-7, -1e-7;
    EXPECT_TRUE(cmd.getValues().isApprox(expected, m_tol_numerical));
}

/// @brief Verify scale with very large factor
TEST_F(CommandTest, Scale_VeryLargeFactor) {
    Eigen::VectorXd values(2);
    values << 1e-3, -1e-3;
    Command cmd(values);
    
    cmd.scale(1e6);
    
    Eigen::VectorXd expected(2);
    expected << 1e3, -1e3;
    EXPECT_TRUE(cmd.getValues().isApprox(expected, m_tol_numerical));
}

// ============================================================================
// 7. STABILITY & ROBUSTNESS
// ============================================================================

/// @brief Verify repeated scaling is stable
TEST_F(CommandTest, Scale_RepeatedCalls_Stable) {
    Eigen::VectorXd values(3);
    values << 1.0, 2.0, 3.0;
    Command cmd(values);
    
    // Scale up and down repeatedly
    for (int i = 0; i < 100; ++i) {
        cmd.scale(2.0);
        cmd.scale(0.5);
    }
    
    // Should return to original values
    EXPECT_TRUE(cmd.getValues().isApprox(values, m_tol_numerical));
}

/// @brief Verify repeated saturation doesn't change valid values
TEST_F(CommandTest, Saturate_RepeatedCalls_Idempotent) {
    Eigen::VectorXd values(3);
    values << 5.0, -5.0, 0.0;
    Command cmd(values);
    
    Eigen::VectorXd lower(3);
    lower << -1.0, -1.0, -1.0;
    Eigen::VectorXd upper(3);
    upper << 2.0, 2.0, 2.0;
    cmd.setLimits(lower, upper);

    cmd.saturate();
    Eigen::VectorXd after_first = cmd.getValues();
    
    for (int i = 0; i < 100; ++i)
        cmd.saturate();
    
    Eigen::VectorXd after_repeat = cmd.getValues();
    
    // Should be identical after first saturation
    EXPECT_TRUE(after_first.isApprox(after_repeat, m_tol_zero));
}

} // namespace robot::control::test