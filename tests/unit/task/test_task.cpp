// ============================================================================
// UNIT TESTS FOR TASK BASE CLASS
// ============================================================================
// Purpose: Validate the abstract Task interface and its common functionality
// Focus: isAchieved() method with various error norms and precision levels
// ============================================================================

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "task/task.hpp"
#include "core/robot.hpp"

namespace robot::task::test {

// ============================================================================
// TEST FIXTURE
// ============================================================================

/**
 * @brief Concrete implementation of Task for testing purposes
 * 
 * This mock task allows explicit control of error vectors to validate
 * the base class behavior without dependencies on real implementations.
 */
class MockTask : public Task {
    public:
        MockTask(const std::string& name, int priority, const Eigen::VectorXd& error)
            : Task(name, priority), m_error(error) {}

        Eigen::VectorXd computeError(const core::Robot& robot) const override {
            return m_error;
        }

        Eigen::MatrixXd getJacobian(const core::Robot& robot) const override {
            return Eigen::MatrixXd::Identity(m_error.size(), m_error.size());
        }

        int getDimension() const override {
            return static_cast<int>(m_error.size());
        }

        void setError(const Eigen::VectorXd& error) {
            m_error = error;
        }

    private:
        Eigen::VectorXd m_error;
};

class TaskTest : public ::testing::Test {
protected:
    std::unique_ptr<robot::core::Robot> m_robottest;       

    void SetUp() override {
        // Numerical tolerances
        m_tolerance_achievement = 1e-6;
        m_tolerance_comparison = 1e-10;

        // Default constructed robot
        robot::core::RobotConfig configtest = {
            "test_robot",
            std::vector<robot::core::JointConfig>{
                { .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}, 
                { .name = "Joint_1", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}
            },
            std::vector<robot::core::LinkConfig>{
                { .name = "Link_0", .a = 0, .alpha = 0, .d = 0, .theta = 0, .mass = 2.0, .com = Eigen::Vector3d(1.0, 0.0, 0.0), .ID_link = 0, .ID_parent = -1, .motor = true}, 
                { .name = "Link_1", .a = 0, .alpha = 0, .d = 0, .theta = 0, .mass = 1.0, .com = Eigen::Vector3d(0.0, 1.0, 0.0), .ID_link = 1, .ID_parent = 0, .motor = true}
            }
        };
        
        m_robottest = std::make_unique<robot::core::Robot>(configtest);
    }

    // Test parameters
    double m_tolerance_achievement;
    double m_tolerance_comparison;
};

// ============================================================================
// 1. NOMINAL CASES - isAchieved() METHOD
// ============================================================================

TEST_F(TaskTest, IsAchieved_ZeroError) {
    // Zero error should always be achieved regardless of precision
    Eigen::VectorXd zero_error = Eigen::VectorXd::Zero(3);
    MockTask task("test", 0, zero_error);

    EXPECT_TRUE(task.isAchieved(*m_robottest, 1e-3));
    EXPECT_TRUE(task.isAchieved(*m_robottest, 1e-6));
    EXPECT_TRUE(task.isAchieved(*m_robottest, 1e-12));
}

TEST_F(TaskTest, IsAchieved_SmallErrorWithinPrecision) {
    // Error norm = 1e-4, precision = 1e-3 → should be achieved
    Eigen::VectorXd error(3);
    error << 5e-5, 5e-5, 5e-5;  // norm ≈ 8.66e-5 < 1e-3
    
    MockTask task("test", 0, error);

    EXPECT_TRUE(task.isAchieved(*m_robottest, 1e-3));
    EXPECT_LT(error.norm(), 1e-3);  // Verify assumption
}

TEST_F(TaskTest, IsAchieved_ErrorExceedsPrecision) {
    // Error norm = 1e-2, precision = 1e-3 → should NOT be achieved
    Eigen::VectorXd error(3);
    error << 5e-3, 5e-3, 5e-3;  // norm ≈ 8.66e-3 > 1e-3
    
    MockTask task("test", 0, error);

    EXPECT_FALSE(task.isAchieved(*m_robottest, 1e-3));
    EXPECT_GT(error.norm(), 1e-3);  // Verify assumption
}

// ============================================================================
// 2. BOUNDARY CASES
// ============================================================================

TEST_F(TaskTest, IsAchieved_ErrorExactlyEqualsPrecision) {
    // Mathematical property: norm < precision (strict inequality)
    // When norm == precision, task is NOT achieved
    double precision = 1e-3;
    Eigen::VectorXd error(1);
    error << precision;  // norm exactly equals precision
    
    MockTask task("test", 0, error);

    EXPECT_FALSE(task.isAchieved(*m_robottest, precision));
    EXPECT_NEAR(error.norm(), precision, m_tolerance_comparison);
}

TEST_F(TaskTest, IsAchieved_ErrorJustBelowPrecision) {
    // Error slightly below precision threshold
    double precision = 1e-3;
    Eigen::VectorXd error(1);
    error << precision * 0.99;  // 99% of precision
    
    MockTask task("test", 0, error);

    EXPECT_TRUE(task.isAchieved(*m_robottest, precision));
    EXPECT_LT(error.norm(), precision);
}

TEST_F(TaskTest, IsAchieved_ErrorJustAbovePrecision) {
    // Error slightly above precision threshold
    double precision = 1e-3;
    Eigen::VectorXd error(1);
    error << precision * 1.01;  // 101% of precision
    
    MockTask task("test", 0, error);

    EXPECT_FALSE(task.isAchieved(*m_robottest, precision));
    EXPECT_GT(error.norm(), precision);
}

// ============================================================================
// 3. DIMENSION INDEPENDENCE
// ============================================================================

TEST_F(TaskTest, IsAchieved_ScalarError) {
    // Test with 1D error vector
    Eigen::VectorXd error(1);
    error << 5e-4;
    
    MockTask task("test", 0, error);

    EXPECT_TRUE(task.isAchieved(*m_robottest, 1e-3));
    EXPECT_FALSE(task.isAchieved(*m_robottest, 1e-4));
}

TEST_F(TaskTest, IsAchieved_HighDimensionalError) {
    // Test with 6D error (typical operational space dimension)
    // Verify that Euclidean norm is used correctly
    Eigen::VectorXd error(6);
    error << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    double expected_norm = std::sqrt(6.0) * 1e-4;  // ≈ 2.45e-4
    
    MockTask task("test", 0, error);

    EXPECT_NEAR(error.norm(), expected_norm, m_tolerance_comparison);
    EXPECT_TRUE(task.isAchieved(*m_robottest, 5e-4));
    EXPECT_FALSE(task.isAchieved(*m_robottest, 2e-4));
}

// ============================================================================
// 4. SIGN INDEPENDENCE
// ============================================================================

TEST_F(TaskTest, IsAchieved_NegativeError) {
    // Norm is invariant to sign → ||-e|| = ||e||
    Eigen::VectorXd positive_error(3);
    positive_error << 1e-4, 2e-4, 3e-4;
    
    Eigen::VectorXd negative_error = -positive_error;
    
    MockTask task_pos("test_pos", 0, positive_error);
    MockTask task_neg("test_neg", 0, negative_error);

    double precision = 5e-4;
    
    EXPECT_EQ(task_pos.isAchieved(*m_robottest, precision), 
              task_neg.isAchieved(*m_robottest, precision));
    EXPECT_NEAR(positive_error.norm(), negative_error.norm(), m_tolerance_comparison);
}

// ============================================================================
// 5. PRECISION SENSITIVITY
// ============================================================================

TEST_F(TaskTest, IsAchieved_VaryingPrecision) {
    // Fixed error, varying precision levels
    // Tests monotonicity: tighter precision → harder to achieve
    Eigen::VectorXd error(3);
    error << 1e-3, 1e-3, 1e-3;
    double error_norm = error.norm();  // ≈ 1.73e-3
    
    MockTask task("test", 0, error);

    // Loose precision → achieved
    EXPECT_TRUE(task.isAchieved(*m_robottest, 1e-2));
    
    // Medium precision → achieved
    EXPECT_TRUE(task.isAchieved(*m_robottest, 2e-3));
    
    // Tight precision (just below error) → not achieved
    EXPECT_FALSE(task.isAchieved(*m_robottest, 1e-3));
    
    // Very tight precision → not achieved
    EXPECT_FALSE(task.isAchieved(*m_robottest, 1e-6));
}

// ============================================================================
// 6. GETTERS AND SETTERS
// ============================================================================

TEST_F(TaskTest, NameGetterSetter) {
    Eigen::VectorXd error = Eigen::VectorXd::Zero(3);
    MockTask task("initial_name", 0, error);
    
    EXPECT_EQ(task.getName(), "initial_name");
    
    task.setName("new_name");
    EXPECT_EQ(task.getName(), "new_name");
}

TEST_F(TaskTest, PriorityGetterSetter) {
    Eigen::VectorXd error = Eigen::VectorXd::Zero(3);
    MockTask task("test", 5, error);
    
    EXPECT_EQ(task.getPriority(), 5);
    
    task.setPriority(10);
    EXPECT_EQ(task.getPriority(), 10);
}

// ============================================================================
// 7. EDGE CASES
// ============================================================================

TEST_F(TaskTest, IsAchieved_VeryLargePrecision) {
    // Precision larger than any reasonable error
    Eigen::VectorXd error(3);
    error << 1.0, 1.0, 1.0;  // norm ≈ 1.73
    
    MockTask task("test", 0, error);

    EXPECT_TRUE(task.isAchieved(*m_robottest, 1e3));  // Huge precision
}

TEST_F(TaskTest, IsAchieved_VerySmallPrecision) {
    // Very tight precision requires near-zero error
    Eigen::VectorXd tiny_error(3);
    tiny_error << 1e-11, 1e-11, 1e-11;  // norm ≈ 1.73e-11
    
    Eigen::VectorXd small_error(3);
    small_error << 1e-9, 1e-9, 1e-9;  // norm ≈ 1.73e-9
    
    MockTask task_tiny("tiny", 0, tiny_error);
    MockTask task_small("small", 0, small_error);

    double ultra_tight_precision = 1e-10;
    
    EXPECT_TRUE(task_tiny.isAchieved(*m_robottest, ultra_tight_precision));
    EXPECT_FALSE(task_small.isAchieved(*m_robottest, ultra_tight_precision));
}

} // namespace robot::task::test