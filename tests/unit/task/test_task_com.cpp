#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "task/task_com.hpp"
#include "core/robot.hpp"

namespace robot::task::test {

// ============================================================================
// 0. TEST FIXTURE
// ============================================================================

/// @brief Base class for shared robot configuration
class TaskCoMTestBase {
    protected:
        std::unique_ptr<robot::core::Robot> m_robot;
        double m_tol_zero = 1e-12;
        double m_tol_numerical = 1e-10;

        void SetUpConfig() {
            // Simple 2-link robot for deterministic testing
            robot:core::RobotConfig config = {
                "test_robot",
                std::vector<robot::core::JointConfig>{
                    { .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}, 
                    { .name = "Joint_1", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}
                },
                std::vector<robot::core::LinkConfig>{
                    { .name = "Link_0", .a = 0, .alpha = 0, .d = 0, .theta = 0, 
                    .mass = 2.0, .com = Eigen::Vector3d(1.0, 0.0, 0.0), 
                    .ID_link = 0, .ID_parent = -1, .motor = true}, 
                    { .name = "Link_1", .a = 0, .alpha = 0, .d = 0, .theta = 0, 
                    .mass = 1.0, .com = Eigen::Vector3d(0.0, 1.0, 0.0), 
                    .ID_link = 1, .ID_parent = 0, .motor = true}
                }
            };
            
            m_robot = std::make_unique<robot::core::Robot>(config);
        }
};

class TaskCoMTest : public TaskCoMTestBase, 
                    public ::testing::Test {
protected:
    void SetUp() override {
        SetUpConfig();
    }
};

// ============================================================================
// 1. CONSTRUCTOR & INITIALIZATION
// ============================================================================

/// @brief Verify constructor stores parameters correctly
TEST_F(TaskCoMTest, Constructor_InitializesCorrectly) {
    Eigen::Vector3d desired_com(1.0, 2.0, 3.0);
    
    TaskCoM task(desired_com, m_robot->config(), "TestCoM", 5);
    
    EXPECT_EQ(task.getName(), "TestCoM");
    EXPECT_EQ(task.getPriority(), 5);
    EXPECT_EQ(task.getDimension(), 1);  // X-axis only
}

/// @brief Verify default parameters are set correctly
TEST_F(TaskCoMTest, Constructor_UsesDefaultParameters) {
    Eigen::Vector3d desired_com(0.0, 0.0, 0.0);
    
    TaskCoM task(desired_com, m_robot->config());
    
    EXPECT_EQ(task.getName(), "CoM");
    EXPECT_EQ(task.getPriority(), 1);
}

// ============================================================================
// 2. NORMAL CASES OF USE
// ============================================================================

/// @brief Verify getter returns the constructor value
TEST_F(TaskCoMTest, GetDesiredCoM_ReturnsConstructorValue) {
    Eigen::Vector3d desired(1.5, -2.3, 4.7);
    TaskCoM task(desired, m_robot->config());
    
    EXPECT_TRUE(task.getDesiredCoM().isApprox(desired, m_tol_zero));
}

/// @brief Verify setter updates the internal state
TEST_F(TaskCoMTest, SetDesiredCoM_UpdatesValue) {
    Eigen::Vector3d initial(1.0, 0.0, 0.0);
    TaskCoM task(initial, m_robot->config());
    
    Eigen::Vector3d updated(3.0, 5.0, -2.0);
    task.setDesiredCoM(updated);
    
    EXPECT_TRUE(task.getDesiredCoM().isApprox(updated, m_tol_zero));
}

// ============================================================================
// 3. DIMENSION & STRUCTURE
// ============================================================================

/// @brief Verify task dimension is always 1 (X-axis only)
TEST_F(TaskCoMTest, GetDimension_AlwaysReturnsOne) {
    Eigen::Vector3d desired(0.0, 0.0, 0.0);
    TaskCoM task(desired, m_robot->config());
    
    EXPECT_EQ(task.getDimension(), 1);
}

/// @brief Verify error vector has correct dimension
TEST_F(TaskCoMTest, ComputeError_Returns1DVector) {
    Eigen::Vector3d desired(1.0, 0.0, 0.0);
    TaskCoM task(desired, m_robot->config());
    
    Eigen::VectorXd error = task.computeError(*m_robot);
    
    EXPECT_EQ(error.size(), 1);
    EXPECT_TRUE(std::isfinite(error(0)));
}

/// @brief Verify Jacobian matrix has correct dimensions (1 x num_joints)
TEST_F(TaskCoMTest, GetJacobian_CorrectDimensions) {
    Eigen::Vector3d desired(1.0, 0.0, 0.0);
    TaskCoM task(desired, m_robot->config());
    
    Eigen::MatrixXd J = task.getJacobian(*m_robot);
    
    EXPECT_EQ(J.rows(), 1);
    EXPECT_EQ(J.cols(), m_robot->state().jointPositions.size());
}

// ============================================================================
// 4. ERROR PROPERTIES
// ============================================================================

/// @brief Verify only X-coordinate affects the error
TEST_F(TaskCoMTest, ComputeError_YZComponentsIgnored) {
    Eigen::Vector3d desired_1(2.0, 0.0, 0.0);
    Eigen::Vector3d desired_2(2.0, 100.0, -50.0);
    
    TaskCoM task1(desired_1, m_robot->config());
    TaskCoM task2(desired_2, m_robot->config());
    
    Eigen::VectorXd error1 = task1.computeError(*m_robot);
    Eigen::VectorXd error2 = task2.computeError(*m_robot);
    
    EXPECT_NEAR(error1.norm(), error2.norm(), m_tol_numerical);
}

/// @brief Verify error changes when desired position changes
TEST_F(TaskCoMTest, ComputeError_ChangesErrorCorrectly) {
    Eigen::Vector3d initial_desired(1.0, 0.0, 0.0);
    TaskCoM task(initial_desired, m_robot->config());
    
    Eigen::VectorXd error_before = task.computeError(*m_robot);
    
    Eigen::Vector3d new_desired(4.0, 0.0, 0.0);
    task.setDesiredCoM(new_desired);
    
    Eigen::VectorXd error_after = task.computeError(*m_robot);
    
    // Error change should equal desired position change
    double expected_change = new_desired.x() - initial_desired.x();
    double actual_change = error_after(0) - error_before(0);
    
    EXPECT_NEAR(actual_change, expected_change, m_tol_numerical);
}

// ============================================================================
// 5. PARAMETERIZED TESTS
// ============================================================================

struct DesiredPositionTestCase {
    double x_position;
    std::string description;
};

class TaskCoMDesiredPositionTest : public TaskCoMTestBase,
                                   public ::testing::TestWithParam<DesiredPositionTestCase> {
    protected:
        void SetUp() override {
            SetUpConfig();
        }
};

/// @brief Verify computeError handles various desired positions correctly
TEST_P(TaskCoMDesiredPositionTest, ComputeError_VariousPositions) {
    auto testCase = GetParam();
    
    Eigen::Vector3d desired(testCase.x_position, 0.0, 0.0);
    TaskCoM task(desired, m_robot->config());
    
    Eigen::VectorXd error = task.computeError(*m_robot);
    
    // Error should always be finite and 1D
    EXPECT_EQ(error.size(), 1) << "Failed for " << testCase.description;
    EXPECT_TRUE(std::isfinite(error(0))) << "Failed for " << testCase.description;
}

INSTANTIATE_TEST_SUITE_P(
    VariousDesiredPositions,
    TaskCoMDesiredPositionTest,
    ::testing::Values(
        DesiredPositionTestCase{0.0, "Zero position"},
        DesiredPositionTestCase{1.0, "Small positive"},
        DesiredPositionTestCase{-1.0, "Small negative"},
        DesiredPositionTestCase{1e-6, "Very small positive"},
        DesiredPositionTestCase{-1e-6, "Very small negative"},
        DesiredPositionTestCase{1e6, "Very large positive"},
        DesiredPositionTestCase{-1e6, "Very large negative"}
    )
);

// ============================================================================
// 6. STABILITY & ROBUSTNESS
// ============================================================================

/// @brief Verify repeated changes to desired position don't cause instability
TEST_F(TaskCoMTest, SetDesiredCoM_RepeatedChanges_Stable) {
    Eigen::Vector3d desired(1.0, 0.0, 0.0);
    TaskCoM task(desired, m_robot->config());
    
    for (int i = 0; i < 100; ++i) {
        double x = static_cast<double>(i) / 10.0;
        task.setDesiredCoM(Eigen::Vector3d(x, 0.0, 0.0));
        
        Eigen::VectorXd error = task.computeError(*m_robot);
        EXPECT_TRUE(std::isfinite(error(0)));
    }
}

/// @brief Verify Jacobian remains consistent across multiple calls
TEST_F(TaskCoMTest, GetJacobian_ConsistentAcrossMultipleCalls) {
    Eigen::Vector3d desired(1.0, 0.0, 0.0);
    TaskCoM task(desired, m_robot->config());
    
    Eigen::MatrixXd J_first = task.getJacobian(*m_robot);
    
    for (int i = 0; i < 100; ++i) {
        Eigen::MatrixXd J = task.getJacobian(*m_robot);
        EXPECT_TRUE(J_first.isApprox(J, m_tol_zero));
    }
}

/// @brief Verify repeated computations give identical results
TEST_F(TaskCoMTest, ComputeError_DeterministicResults) {
    Eigen::Vector3d desired(5.0, 0.0, 0.0);
    TaskCoM task(desired, m_robot->config());
    
    Eigen::VectorXd error_first = task.computeError(*m_robot);

    for (int i = 0; i < 100; ++i) {
        Eigen::VectorXd error = task.computeError(*m_robot);
        EXPECT_TRUE(error_first.isApprox(error, m_tol_zero));
    }
}

} // namespace robot::task::test