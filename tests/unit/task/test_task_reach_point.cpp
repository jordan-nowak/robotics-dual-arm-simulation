#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "task/task_reach_point.hpp"
#include "core/robot.hpp"

namespace robot::task::test {

// ============================================================================
// 0. TEST FIXTURE
// ============================================================================

/// @brief Base class for shared robot configuration
class TaskReachPointTestBase {
protected:
    std::unique_ptr<robot::core::Robot> m_robot;
    double m_tol_zero = 1e-12;
    double m_tol_numerical = 1e-10;
    int m_link_index = 1;  // Default test link

    void SetUpConfig() {
        // Simple 2-link robot for deterministic testing
        robot::core::RobotConfig config = {
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

class TaskReachPointTest : public TaskReachPointTestBase,
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
TEST_F(TaskReachPointTest, Constructor_InitializesCorrectly) {
    Eigen::Vector3d desired(1.0, 2.0, 3.0);
    
    TaskReachPoint task(desired, m_robot->config(), m_link_index, "TestEE", 3);
    
    EXPECT_EQ(task.getName(), "TestEE");
    EXPECT_EQ(task.getPriority(), 3);
    EXPECT_EQ(task.getDimension(), 3);
    EXPECT_EQ(task.getLinkIndex(), m_link_index);
}

/// @brief Verify default parameters are set correctly
TEST_F(TaskReachPointTest, Constructor_UsesDefaultParameters) {
    Eigen::Vector3d desired(0.0, 0.0, 0.0);
    
    TaskReachPoint task(desired, m_robot->config(), m_link_index);
    
    EXPECT_EQ(task.getName(), "EndEffector");
    EXPECT_EQ(task.getPriority(), 0);
}

// ============================================================================
// 2. NORMAL CASES OF USE
// ============================================================================

/// @brief Verify getter returns the constructor value
TEST_F(TaskReachPointTest, GetDesiredPosition_ReturnsConstructorValue) {
    Eigen::Vector3d desired(1.5, -2.3, 4.7);
    TaskReachPoint task(desired, m_robot->config(), m_link_index);
    
    EXPECT_TRUE(task.getDesiredPosition().isApprox(desired, m_tol_zero));
}

/// @brief Verify setter updates the internal state
TEST_F(TaskReachPointTest, SetDesiredPosition_UpdatesValue) {
    Eigen::Vector3d initial(1.0, 0.0, 0.0);
    TaskReachPoint task(initial, m_robot->config(), m_link_index);
    
    Eigen::Vector3d updated(3.0, 5.0, -2.0);
    task.setDesiredPosition(updated);
    
    EXPECT_TRUE(task.getDesiredPosition().isApprox(updated, m_tol_zero));
}

/// @brief Verify link index getter returns constructor value
TEST_F(TaskReachPointTest, GetLinkIndex_ReturnsConstructorValue) {
    Eigen::Vector3d desired(1.0, 0.0, 0.0);
    int test_link = 0;
    
    TaskReachPoint task(desired, m_robot->config(), test_link);
    
    EXPECT_EQ(task.getLinkIndex(), test_link);
}

/// @brief Verify link index setter updates the value
TEST_F(TaskReachPointTest, SetLinkIndex_UpdatesValue) {
    Eigen::Vector3d desired(1.0, 0.0, 0.0);
    TaskReachPoint task(desired, m_robot->config(), 0);
    
    task.setLinkIndex(1);
    
    EXPECT_EQ(task.getLinkIndex(), 1);
}

// ============================================================================
// 3. DIMENSION & STRUCTURE
// ============================================================================

/// @brief Verify task dimension is always 3 (3D Cartesian position)
TEST_F(TaskReachPointTest, GetDimension_AlwaysReturnsThree) {
    Eigen::Vector3d desired(0.0, 0.0, 0.0);
    TaskReachPoint task(desired, m_robot->config(), m_link_index);
    
    EXPECT_EQ(task.getDimension(), 3);
}

/// @brief Verify error vector has correct dimension
TEST_F(TaskReachPointTest, ComputeError_Returns3DVector) {
    Eigen::Vector3d desired(1.0, 2.0, 3.0);
    TaskReachPoint task(desired, m_robot->config(), m_link_index);
    
    Eigen::VectorXd error = task.computeError(*m_robot);
    
    EXPECT_EQ(error.size(), 3);
    EXPECT_TRUE(std::isfinite(error.norm()));
}

/// @brief Verify Jacobian matrix has correct dimensions (3 x num_joints)
TEST_F(TaskReachPointTest, GetJacobian_CorrectDimensions) {
    Eigen::Vector3d desired(1.0, 0.0, 0.0);
    TaskReachPoint task(desired, m_robot->config(), m_link_index);
    
    Eigen::MatrixXd J = task.getJacobian(*m_robot);
    
    EXPECT_EQ(J.rows(), 3);
    EXPECT_EQ(J.cols(), m_robot->state().jointPositions.size());
}

// ============================================================================
// 4. ERROR PROPERTIES
// ============================================================================

/// @brief Verify error changes by expected amount when desired position changes
TEST_F(TaskReachPointTest, SetDesiredPosition_ChangesErrorCorrectly) {
    Eigen::Vector3d initial(1.0, 2.0, 3.0);
    TaskReachPoint task(initial, m_robot->config(), m_link_index);
    
    Eigen::VectorXd error_before = task.computeError(*m_robot);
    
    Eigen::Vector3d new_desired(4.0, 5.0, 6.0);
    task.setDesiredPosition(new_desired);
    
    Eigen::VectorXd error_after = task.computeError(*m_robot);
    
    Eigen::Vector3d expected_change = new_desired - initial;
    Eigen::Vector3d actual_change = error_after - error_before;
    
    EXPECT_TRUE(actual_change.isApprox(expected_change, m_tol_numerical));
}

/// @brief Verify changing one coordinate affects only that error component
TEST_F(TaskReachPointTest, ComputeError_ComponentsIndependent) {
    Eigen::Vector3d desired_base(1.0, 2.0, 3.0);
    Eigen::Vector3d desired_x_changed(5.0, 2.0, 3.0);  // Only X changed
    
    TaskReachPoint task_base(desired_base, m_robot->config(), m_link_index);
    TaskReachPoint task_x(desired_x_changed, m_robot->config(), m_link_index);
    
    Eigen::VectorXd error_base = task_base.computeError(*m_robot);
    Eigen::VectorXd error_x = task_x.computeError(*m_robot);
    
    // Y and Z errors should be identical
    EXPECT_NEAR(error_base(1), error_x(1), m_tol_numerical);
    EXPECT_NEAR(error_base(2), error_x(2), m_tol_numerical);
    
    // X error should differ by the change amount
    double expected_diff = desired_x_changed.x() - desired_base.x();
    double actual_diff = error_x(0) - error_base(0);
    EXPECT_NEAR(actual_diff, expected_diff, m_tol_numerical);
}

// ============================================================================
// 5. PARAMETERIZED TESTS
// ============================================================================

struct DesiredPositionTestCase {
    Eigen::Vector3d position;
    std::string description;
};

class TaskReachPointDesiredPositionTest : public TaskReachPointTestBase,
                                          public ::testing::TestWithParam<DesiredPositionTestCase> {
protected:
    void SetUp() override {
        SetUpConfig();
    }
};

/// @brief Verify computeError handles various desired positions correctly
TEST_P(TaskReachPointDesiredPositionTest, ComputeError_VariousPositions) {
    auto testCase = GetParam();
    
    TaskReachPoint task(testCase.position, m_robot->config(), m_link_index);
    
    Eigen::VectorXd error = task.computeError(*m_robot);
    
    // Error should always be finite and 3D
    EXPECT_EQ(error.size(), 3) << "Failed for " << testCase.description;
    EXPECT_TRUE(std::isfinite(error.norm())) << "Failed for " << testCase.description;
}

INSTANTIATE_TEST_SUITE_P(
    VariousDesiredPositions,
    TaskReachPointDesiredPositionTest,
    ::testing::Values(
        DesiredPositionTestCase{Eigen::Vector3d(0.0, 0.0, 0.0), "Zero position"},
        DesiredPositionTestCase{Eigen::Vector3d(1.0, 0.0, 0.0), "Positive X only"},
        DesiredPositionTestCase{Eigen::Vector3d(0.0, 1.0, 0.0), "Positive Y only"},
        DesiredPositionTestCase{Eigen::Vector3d(0.0, 0.0, 1.0), "Positive Z only"},
        DesiredPositionTestCase{Eigen::Vector3d(1.0, 1.0, 1.0), "Small positive"},
        DesiredPositionTestCase{Eigen::Vector3d(-1.0, -1.0, -1.0), "Small negative"},
        DesiredPositionTestCase{Eigen::Vector3d(1e-6, 1e-6, 1e-6), "Very small positive"},
        DesiredPositionTestCase{Eigen::Vector3d(-1e-6, -1e-6, -1e-6), "Very small negative"},
        DesiredPositionTestCase{Eigen::Vector3d(1e3, 1e3, 1e3), "Very large positive"},
        DesiredPositionTestCase{Eigen::Vector3d(-1e3, -1e3, -1e3), "Very large negative"}
    )
);

// ============================================================================
// 6. STABILITY & ROBUSTNESS
// ============================================================================

/// @brief Verify repeated changes to desired position don't cause instability
TEST_F(TaskReachPointTest, SetDesiredPosition_RepeatedChanges_Stable) {
    Eigen::Vector3d desired(1.0, 0.0, 0.0);
    TaskReachPoint task(desired, m_robot->config(), m_link_index);
    
    for (int i = 0; i < 100; ++i) {
        double t = static_cast<double>(i) / 10.0;
        task.setDesiredPosition(Eigen::Vector3d(t, t * 2, t * 3));
        
        Eigen::VectorXd error = task.computeError(*m_robot);
        EXPECT_TRUE(std::isfinite(error.norm()));
    }
}

/// @brief Verify Jacobian remains consistent across multiple calls
TEST_F(TaskReachPointTest, GetJacobian_ConsistentAcrossMultipleCalls) {
    Eigen::Vector3d desired(1.0, 2.0, 3.0);
    TaskReachPoint task(desired, m_robot->config(), m_link_index);
    
    Eigen::MatrixXd J_first = task.getJacobian(*m_robot);
    
    for (int i = 0; i < 100; ++i) {
        Eigen::MatrixXd J = task.getJacobian(*m_robot);
        EXPECT_TRUE(J_first.isApprox(J, m_tol_zero));
    }
}

/// @brief Verify repeated computations give identical results
TEST_F(TaskReachPointTest, ComputeError_DeterministicResults) {
    Eigen::Vector3d desired(1.0, 2.0, 3.0);
    TaskReachPoint task(desired, m_robot->config(), m_link_index);
    
    Eigen::VectorXd error1 = task.computeError(*m_robot);
    Eigen::VectorXd error2 = task.computeError(*m_robot);
    
    EXPECT_TRUE(error1.isApprox(error2, m_tol_zero));
}

} // namespace robot::task::test