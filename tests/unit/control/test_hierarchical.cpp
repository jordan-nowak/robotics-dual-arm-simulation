#include <gtest/gtest.h>
#include "control/hierarchical.hpp"

using namespace robot::control;

/**
 * @brief Test Fixture for common setup
 */
class HierarchicalTests : public ::testing::Test {
    protected:
        const double tolerance_test = 1e-6;
        const double precision_test = 1e-9;
        
        Hierarchical hierarchical;
};

// ============================================================================
// 1. PROJECTION MATRIX - NOMINAL CASES
// ============================================================================

/**
 * @brief Test projection matrix on a square full-rank matrix
 */
TEST_F(HierarchicalTests, ProjectionMatrixSquareFullRank) {
    Eigen::MatrixXd J(3, 3);
    J << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    
    Eigen::MatrixXd P = hierarchical.projectionMatrix(J, tolerance_test);
    
    // For identity matrix, P should be zero (no nullspace)
    EXPECT_LT(P.norm(), precision_test);
}

/**
 * @brief Test projection matrix on a rectangular matrix
 */
TEST_F(HierarchicalTests, ProjectionMatrixRectangular) {
    Eigen::MatrixXd J(2, 4);
    J << 1, 0, 0, 0,
         0, 1, 0, 0;
    
    Eigen::MatrixXd P = hierarchical.projectionMatrix(J, tolerance_test);
    
    // Verify dimensions
    EXPECT_EQ(P.rows(), 4);
    EXPECT_EQ(P.cols(), 4);
    
    // P should be symmetric
    EXPECT_TRUE(P.isApprox(P.transpose(), precision_test));
    
    // P should be idempotent: P * P = P
    Eigen::MatrixXd P_squared = P * P;
    EXPECT_TRUE(P_squared.isApprox(P, precision_test));
}

/**
 * @brief Test projection matrix projects into nullspace
 */
TEST_F(HierarchicalTests, ProjectionMatrixNullspace) {
    Eigen::MatrixXd J(1, 3);
    J << 1, 0, 0;  // Constrains only first DOF
    
    Eigen::MatrixXd P = hierarchical.projectionMatrix(J, tolerance_test);
    
    // Expected projection: [0, 0, 0; 0, 1, 0; 0, 0, 1]
    Eigen::MatrixXd expected(3, 3);
    expected << 0, 0, 0,
                0, 1, 0,
                0, 0, 1;
    
    EXPECT_TRUE(P.isApprox(expected, precision_test));
    
    // Verify J * P = 0 (P projects into nullspace of J)
    Eigen::MatrixXd result = J * P;
    EXPECT_LT(result.norm(), precision_test);
}

// ============================================================================
// 2. PROJECTION MATRIX - LIMIT CASES
// ============================================================================

/**
 * @brief Test projection matrix on empty matrix
 */
TEST_F(HierarchicalTests, ProjectionMatrixEmptyMatrix) {
    Eigen::MatrixXd J;
    
    Eigen::MatrixXd P = hierarchical.projectionMatrix(J, tolerance_test);
    
    // Should return empty matrix
    EXPECT_EQ(P.size(), 0);
}

/**
 * @brief Test projection matrix on zero matrix
 */
TEST_F(HierarchicalTests, ProjectionMatrixZeroMatrix) {
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(2, 3);
    
    Eigen::MatrixXd P = hierarchical.projectionMatrix(J, tolerance_test);
    
    // For zero matrix, P should be identity (entire space is nullspace)
    EXPECT_TRUE(P.isApprox(Eigen::MatrixXd::Identity(3, 3), precision_test));
}

/**
 * @brief Test projection matrix on single row
 */
TEST_F(HierarchicalTests, ProjectionMatrixSingleRow) {
    Eigen::MatrixXd J(1, 4);
    J << 1, 2, 3, 4;
    
    Eigen::MatrixXd P = hierarchical.projectionMatrix(J, tolerance_test);
    
    // Verify dimensions
    EXPECT_EQ(P.rows(), 4);
    EXPECT_EQ(P.cols(), 4);
    
    // Verify J * P is close to zero
    Eigen::MatrixXd result = J * P;
    EXPECT_LT(result.norm(), precision_test);
}

// ============================================================================
// 3. PROJECTION MATRIX - SPECIAL CASES
// ============================================================================

/**
 * @brief Test projection matrix on rank-deficient matrix
 */
TEST_F(HierarchicalTests, ProjectionMatrixRankDeficient) {
    Eigen::MatrixXd J(3, 3);
    J << 1, 2, 3,
         2, 4, 6,  // Second row = 2 * first row
         0, 0, 0;  // Third row is zero
    
    Eigen::MatrixXd P = hierarchical.projectionMatrix(J, tolerance_test);
    
    // P should be symmetric
    EXPECT_TRUE(P.isApprox(P.transpose(), precision_test));
    
    // P should be idempotent
    Eigen::MatrixXd P_squared = P * P;
    EXPECT_TRUE(P_squared.isApprox(P, precision_test));
    
    // Verify J * P is close to zero
    Eigen::MatrixXd result = J * P;
    EXPECT_LT(result.norm(), precision_test);
}

// ============================================================================
// 4. PROJECTION MATRIX - MATHEMATICAL PROPERTIES
// ============================================================================

/**
 * @brief Test that projection matrix is idempotent: P * P = P
 */
TEST_F(HierarchicalTests, ProjectionMatrixIdempotent) {
    Eigen::MatrixXd J(2, 5);
    J << 1, 2, 0, 0, 0,
         0, 0, 3, 4, 0;
    
    Eigen::MatrixXd P = hierarchical.projectionMatrix(J, tolerance_test);
    Eigen::MatrixXd P_squared = P * P;
    
    EXPECT_TRUE(P_squared.isApprox(P, precision_test));
}

/**
 * @brief Test that projection matrix is symmetric: transpose(P) = P
 */
TEST_F(HierarchicalTests, ProjectionMatrixSymmetric) {
    Eigen::MatrixXd J(3, 5);
    J << 1, 0, 0, 0, 0,
         0, 1, 0, 0, 0,
         0, 0, 1, 0, 0;
    
    Eigen::MatrixXd P = hierarchical.projectionMatrix(J, tolerance_test);
    
    EXPECT_TRUE(P.isApprox(P.transpose(), precision_test));
}

/**
 * @brief Test that J * P = 0 (projection into nullspace)
 */
TEST_F(HierarchicalTests, ProjectionMatrixOrthogonality) {
    Eigen::MatrixXd J(2, 4);
    J << 1, 2, 3, 4,
         5, 6, 7, 8;
    
    Eigen::MatrixXd P = hierarchical.projectionMatrix(J, tolerance_test);
    Eigen::MatrixXd result = J * P;
    
    EXPECT_LT(result.norm(), precision_test);

}

// ============================================================================
// 5. HIERARCHICAL STEP - NOMINAL CASES
// ============================================================================

/**
 * @brief Test hierarchical step with no previous projection (first command)
 */
TEST_F(HierarchicalTests, HierarchicalStepFirstTask) {
    Eigen::MatrixXd J(2, 3);
    J << 1, 0, 0,
         0, 1, 0;
    
    Eigen::VectorXd error(2);
    error << 1.0, 2.0;
    
    Eigen::VectorXd u_prev = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd P_prev = Eigen::MatrixXd::Identity(3, 3);
    
    Eigen::VectorXd delta_u = hierarchical.hierarchicalStep(J, error, u_prev, P_prev, tolerance_test);
    
    // For first task with identity P_prev, should be simple J_pinv * error
    EXPECT_EQ(delta_u.size(), 3);
    EXPECT_NEAR(delta_u(0), 1.0, precision_test);
    EXPECT_NEAR(delta_u(1), 2.0, precision_test);
    EXPECT_NEAR(delta_u(2), 0.0, precision_test);
}

/**
 * @brief Test hierarchical step with previous command
 */
TEST_F(HierarchicalTests, HierarchicalStepWithPreviousCommand) {
    Eigen::MatrixXd J(1, 3);
    J << 1, 1, 1;
    
    Eigen::VectorXd error(1);
    error << 3.0;
    
    Eigen::VectorXd u_prev(3);
    u_prev << 1.0, 0.0, 0.0;
    
    Eigen::MatrixXd P_prev = Eigen::MatrixXd::Identity(3, 3);

    // Verify initial state: J * u_prev = 1.0
    EXPECT_NEAR((J * u_prev)(0), 1.0, precision_test);
    
    // Residual error: 3.0 - J * u_prev = 3.0 - 1.0 = 2.0
    // Solution should compensate this residual error
    Eigen::VectorXd delta_u = hierarchical.hierarchicalStep(J, error, u_prev, P_prev, tolerance_test);
    Eigen::VectorXd total_u = u_prev + delta_u;
    
    // Verify that the total command produces the desired error.
    // J * total_u should give 3.0, the desired error
    double result_error = (J * total_u)(0);
    EXPECT_NEAR(result_error, 3.0, precision_test);
}

/**
 * @brief Test hierarchical step with constrained nullspace
 */
TEST_F(HierarchicalTests, HierarchicalStepConstrainedNullspace) {
    // First task constrains first DOF
    Eigen::MatrixXd J1(1, 3);
    J1 << 1, 0, 0;
    
    // Get projection matrix from first task
    Eigen::MatrixXd P1 = hierarchical.projectionMatrix(J1, tolerance_test);
    
    // Second task tries to move in all directions
    Eigen::MatrixXd J2(1, 3);
    J2 << 1, 1, 1;
    
    Eigen::VectorXd error2(1);
    error2 << 3.0;
    
    Eigen::VectorXd u_prev = Eigen::VectorXd::Zero(3);
    
    Eigen::VectorXd delta_u = hierarchical.hierarchicalStep(J2, error2, u_prev, P1, tolerance_test);
    
    // delta_u should be in nullspace of J1 (first component should be ~0)
    EXPECT_NEAR(delta_u(0), 0.0, precision_test);
    
    // Other components should be non-zero
    EXPECT_GT(std::abs(delta_u(1)) + std::abs(delta_u(2)), precision_test);
}

// ============================================================================
// 6. HIERARCHICAL STEP - LIMIT CASES
// ============================================================================

/**
 * @brief Test hierarchical step with zero error
 */
TEST_F(HierarchicalTests, HierarchicalStepZeroError) {
    Eigen::MatrixXd J(2, 3);
    J << 1, 0, 0,
         0, 1, 0;
    
    Eigen::VectorXd error = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u_prev = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd P_prev = Eigen::MatrixXd::Identity(3, 3);
    
    Eigen::VectorXd delta_u = hierarchical.hierarchicalStep(J, error, u_prev, P_prev, tolerance_test);
    
    // Should return zero vector
    EXPECT_LT(delta_u.norm(), precision_test);
}

/**
 * @brief Test hierarchical step with zero projection (fully constrained)
 */
TEST_F(HierarchicalTests, HierarchicalStepZeroProjection) {
    Eigen::MatrixXd J(3, 3);
    J << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    
    Eigen::VectorXd error(3);
    error << 1.0, 1.0, 1.0;
    
    Eigen::VectorXd u_prev = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd P_prev = Eigen::MatrixXd::Zero(3, 3);  // Fully constrained (no available DOF)
    
    Eigen::VectorXd delta_u = hierarchical.hierarchicalStep(J, error, u_prev, P_prev, tolerance_test);
    
    // Should return zero
    EXPECT_LT(delta_u.norm(), precision_test);
}

// ============================================================================
// 7. HIERARCHICAL STEP - DIMENSION CONSISTENCY
// ============================================================================

/**
 * @brief Test that hierarchical step preserves dimensions
 */
TEST_F(HierarchicalTests, HierarchicalStepDimensionConsistency) {
    Eigen::MatrixXd J(3, 5);
    J << 1, 0, 0, 0, 0,
         0, 1, 0, 0, 0,
         0, 0, 1, 0, 0;
    
    Eigen::VectorXd error(3);
    error << 1.0, 2.0, 3.0;
    
    Eigen::VectorXd u_prev(5);
    u_prev << 0.5, 0.5, 0.5, 0.5, 0.5;
    
    Eigen::MatrixXd P_prev = Eigen::MatrixXd::Identity(5, 5);
    
    Eigen::VectorXd delta_u = hierarchical.hierarchicalStep(J, error, u_prev, P_prev, tolerance_test);
    
    // Output dimension should match DOF (columns of J)
    EXPECT_EQ(delta_u.size(), 5);
}

// ============================================================================
// 8. INTEGRATION TEST - MULTI-TASK HIERARCHY
// ============================================================================

/**
 * @brief Test two-level task hierarchy
 */
TEST_F(HierarchicalTests, TwoLevelTaskHierarchy) {
    // Task 1 (higher priority): constrain first two DOFs
    // --------------------------------------------------
    Eigen::MatrixXd J1(3, 4);
    J1 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 0, 0;
    
    Eigen::VectorXd error1(3);
    error1 << 1.0, 2.0, 0.0;
    
    // Compute first task solution
    Eigen::VectorXd u_prev = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd P_prev = Eigen::MatrixXd::Identity(4, 4);
    Eigen::VectorXd u1 = hierarchical.hierarchicalStep(J1, error1, u_prev, P_prev, tolerance_test);
    
    // Task 2 (lower priority): use remaining DOFs
    // -------------------------------------------
    Eigen::MatrixXd J2(3, 4);
    J2 << 0, 0, 1, 0,
          0, 0, 0, 0,
          0, 0, 0, 1;
    
    Eigen::VectorXd error2(3);
    error2 << 4.0, 0.0, 3.0;

    // Compute projection of task 1 for task 2
    Eigen::MatrixXd P1 = hierarchical.projectionMatrix(J1, tolerance_test);
    
    // Compute second task solution
    Eigen::VectorXd u2 = hierarchical.hierarchicalStep(J2, error2, u1, P1, tolerance_test);
    
    // Total command
    // -------------
    Eigen::VectorXd u_total = u1 + u2;
    
    // Verify task 1 is satisfied
    Eigen::VectorXd result1 = J1 * u_total;
    EXPECT_TRUE(result1.isApprox(error1, precision_test));
    
    // Verify task 2 is satisfied (in nullspace of task 1)
    Eigen::VectorXd result2 = J2 * u_total;
    EXPECT_TRUE(result2.isApprox(error2, precision_test));
    
    // Verify u2 is in nullspace of J1 (does not affect task 1)
    Eigen::VectorXd check_nullspace = J1 * u2;
    EXPECT_LT(check_nullspace.norm(), precision_test);
}

/**
 * @brief Test combined projection matrix
 */
TEST_F(HierarchicalTests, CombinedProjectionMatrix) {
    // Task 1: constrain DOF 0
    Eigen::MatrixXd J1(1, 4);
    J1 << 1, 0, 0, 0;
    
    // Task 2: constrain DOF 1
    Eigen::MatrixXd J2(1, 4);
    J2 << 0, 1, 0, 0;
    
    // Compute projections
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd P1 = hierarchical.projectionMatrix(J1, tolerance_test);
    Eigen::MatrixXd P2_combined = hierarchical.combinedProjectionMatrix(J2, P1, tolerance_test);
    
    // Verify P2_combined projects into nullspace of both J1 and J2
    Eigen::VectorXd test_vector(4);
    test_vector << 1, 2, 3, 4;
    
    Eigen::VectorXd projected = P2_combined * test_vector;
    
    // Should be in nullspace of J1
    EXPECT_NEAR((J1 * projected)(0), 0.0, precision_test);
    
    // Should be in nullspace of J2
    EXPECT_NEAR((J2 * projected)(0), 0.0, precision_test);
    
    // Only DOFs 2 and 3 should be non-zero
    EXPECT_NEAR(projected(0), 0.0, precision_test);
    EXPECT_NEAR(projected(1), 0.0, precision_test);
    // projected(2) and projected(3) can be non-zero
}

/**
 * @brief Test three-level task hierarchy
 */
TEST_F(HierarchicalTests, ThreeLevelTaskHierarchy) {
    const int dof = 6;
    
    // Task 1: constrain DOF 0
    Eigen::MatrixXd J1(1, dof);
    J1 << 1, 0, 0, 0, 0, 0;
    Eigen::VectorXd error1(1);
    error1 << 1.0;
    
    // Task 2: constrain DOF 1-2
    Eigen::MatrixXd J2(2, dof);
    J2 << 0, 1, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0;
    Eigen::VectorXd error2(2);
    error2 << 2.0, 3.0;
    
    // Task 3: use remaining DOFs
    Eigen::MatrixXd J3(1, dof);
    J3 << 0, 0, 0, 1, 1, 1;
    Eigen::VectorXd error3(1);
    error3 << 6.0;
    
    // Solve hierarchy
    Eigen::VectorXd u_prev = Eigen::VectorXd::Zero(dof);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(dof, dof);
    
    Eigen::VectorXd u1 = hierarchical.hierarchicalStep(J1, error1, u_prev, P0, tolerance_test);
    Eigen::MatrixXd P1 = hierarchical.projectionMatrix(J1, tolerance_test);
    
    Eigen::VectorXd u2 = hierarchical.hierarchicalStep(J2, error2, u1, P1, tolerance_test);
    Eigen::MatrixXd P2 = hierarchical.combinedProjectionMatrix(J2, P1, tolerance_test);
    
    Eigen::VectorXd u3 = hierarchical.hierarchicalStep(J3, error3, u1 + u2, P2, tolerance_test);
    
    Eigen::VectorXd u_total = u1 + u2 + u3;
    
    // Verify all tasks
    EXPECT_NEAR((J1 * u_total)(0), 1.0, precision_test);
    EXPECT_TRUE((J2 * u_total).isApprox(error2, precision_test));
    EXPECT_NEAR((J3 * u_total)(0), 6.0, precision_test);
}