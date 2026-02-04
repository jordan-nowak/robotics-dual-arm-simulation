#include <gtest/gtest.h>
#include "control/pseudo_inverse.hpp"

using namespace robot::control;

/**
 * @brief Test Fixture for common setup
 */
class PseudoInverseTests : public ::testing::Test {
     protected:
          const double tolerance_test = 1e-6;
          const double precision_test = 1e-9;
};

// ============================================================================
// 1. NOMINAL CASES
// ============================================================================

/**
 * @brief Test pseudo-inverse on a square invertible matrix
 */
TEST_F(PseudoInverseTests, SquareInvertibleMatrix) {
    Eigen::MatrixXd A(3, 3);
    A << 1, 2, 3,
         0, 1, 4,
         5, 6, 0;
    
    Eigen::MatrixXd A_pinv = PseudoInverse::compute(A, tolerance_test);
    
    EXPECT_EQ(A_pinv.rows(), 3);
    EXPECT_EQ(A_pinv.cols(), 3);
    
    Eigen::MatrixXd identity = A * A_pinv;
    EXPECT_TRUE(identity.isApprox(Eigen::MatrixXd::Identity(3, 3), precision_test));
    
    identity = A_pinv * A;
    EXPECT_TRUE(identity.isApprox(Eigen::MatrixXd::Identity(3, 3), precision_test));
}

/**
 * @brief Test pseudo-inverse on a horizontal rectangular matrix
 */
TEST_F(PseudoInverseTests, HorizontalRectangularMatrix) {
    Eigen::MatrixXd A(2, 3);
    A << 1, 2, 3,
         4, 5, 6;
    
    Eigen::MatrixXd A_pinv = PseudoInverse::compute(A, tolerance_test);
    
    EXPECT_EQ(A_pinv.rows(), 3);
    EXPECT_EQ(A_pinv.cols(), 2);
    
    // 1st property of Moore-Penrose: A * A_pinv * A = A
    Eigen::MatrixXd result = A * A_pinv * A;
    EXPECT_TRUE(result.isApprox(A, precision_test));
    
    // 2nd property of Moore-Penrose: A_pinv * A * A_pinv = A_pinv
    result = A_pinv * A * A_pinv;
    EXPECT_TRUE(result.isApprox(A_pinv, precision_test));
}

/**
 * @brief Test pseudo-inverse on a vertical rectangular matrix
 */
TEST_F(PseudoInverseTests, VerticalRectangularMatrix) {
    Eigen::MatrixXd A(4, 2);
    A << 1, 2,
         3, 4,
         5, 6,
         7, 8;
    
    Eigen::MatrixXd A_pinv = PseudoInverse::compute(A, tolerance_test);
    
    EXPECT_EQ(A_pinv.rows(), 2);
    EXPECT_EQ(A_pinv.cols(), 4);
    
    // 1st property of Moore-Penrose: A * A_pinv * A = A
    Eigen::MatrixXd result = A * A_pinv * A;
    EXPECT_TRUE(result.isApprox(A, precision_test));
    
    // 2nd property of Moore-Penrose: A_pinv * A * A_pinv = A_pinv
    result = A_pinv * A * A_pinv;
    EXPECT_TRUE(result.isApprox(A_pinv, precision_test));
}

/**
 * @brief Test pseudo-inverse on an identity matrix
 */
TEST_F(PseudoInverseTests, IdentityMatrix) {
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
    
    Eigen::MatrixXd I_pinv = PseudoInverse::compute(I, tolerance_test);
    
    EXPECT_TRUE(I_pinv.isApprox(I, precision_test));
}

// ============================================================================
// 2. LIMITS CASES
// ============================================================================

/**
 * @brief Test pseudo-inverse on an empty matrix
 */
TEST_F(PseudoInverseTests, EmptyMatrix) {
    Eigen::MatrixXd empty;
    
    Eigen::MatrixXd result = PseudoInverse::compute(empty, tolerance_test);
    
    EXPECT_EQ(result.size(), 0);
}

/**
 * @brief Test pseudo-inverse on a single element matrix
 */
TEST_F(PseudoInverseTests, SingleElementMatrix) {
    Eigen::MatrixXd A(1, 1);
    A << 5.0;
    
    Eigen::MatrixXd A_pinv = PseudoInverse::compute(A, tolerance_test);
    
    EXPECT_EQ(A_pinv.rows(), 1);
    EXPECT_EQ(A_pinv.cols(), 1);
    EXPECT_NEAR(A_pinv(0, 0), 0.2, precision_test); // -> 1/5 = 0.2
}

/**
 * @brief Test pseudo-inverse on a zero matrix
 */
TEST_F(PseudoInverseTests, ZeroMatrix) {
    Eigen::MatrixXd zeros = Eigen::MatrixXd::Zero(3, 3);
    
    Eigen::MatrixXd zeros_pinv = PseudoInverse::compute(zeros, tolerance_test);
    
    EXPECT_EQ(zeros_pinv.rows(), 3);
    EXPECT_EQ(zeros_pinv.cols(), 3);
    EXPECT_TRUE(zeros_pinv.isApprox(Eigen::MatrixXd::Zero(3, 3), precision_test));
}

// ============================================================================
// 3. SPECIAL CASES
// ============================================================================

/**
 * @brief Test pseudo-inverse on a rank-deficient matrix
 */
TEST_F(PseudoInverseTests, RankDeficientMatrix) {
    // Arrange: Matrice 3x3 de rang 2 (ligne 3 = ligne 1 + ligne 2)
    Eigen::MatrixXd A(3, 3);
    A << 1, 2, 3,
         4, 5, 6,
         5, 7, 9;  // Combinaison linéaire des deux premières lignes
    
    Eigen::MatrixXd A_pinv = PseudoInverse::compute(A, tolerance_test);
    
    // 1st property of Moore-Penrose: A * A_pinv * A = A
    Eigen::MatrixXd result = A * A_pinv * A;
    EXPECT_TRUE(result.isApprox(A, precision_test));
    
    // 2nd property of Moore-Penrose: A_pinv * A * A_pinv = A_pinv
    result = A_pinv * A * A_pinv;
    EXPECT_TRUE(result.isApprox(A_pinv, precision_test));
}

/**
 * @brief Test pseudo-inverse on single row matrice
 */
TEST_F(PseudoInverseTests, SingleRowMatrix) {
    Eigen::MatrixXd row(1, 4);
    row << 1, 2, 3, 4;
    
    Eigen::MatrixXd row_pinv = PseudoInverse::compute(row, tolerance_test);
    
    EXPECT_EQ(row_pinv.rows(), 4);
    EXPECT_EQ(row_pinv.cols(), 1);
    
    double result = (row * row_pinv)(0, 0);
    EXPECT_NEAR(result, 1.0, precision_test); // scalar
}

/**
 * @brief Test pseudo-inverse on single column matrice
 */
TEST_F(PseudoInverseTests, SingleColumnMatrix) {
    Eigen::MatrixXd col(4, 1);
    col << 1, 2, 3, 4;

    Eigen::MatrixXd col_pinv = PseudoInverse::compute(col, tolerance_test);
    
    EXPECT_EQ(col_pinv.rows(), 1);
    EXPECT_EQ(col_pinv.cols(), 4);
    
    double result = (col_pinv * col)(0, 0);
    EXPECT_NEAR(result, 1.0, precision_test); // scalar
}

// ============================================================================
// 4. TOLERANCE
// ============================================================================

/**
 * @brief Test pseudo-inverse on varying tolerance levels
 */
TEST_F(PseudoInverseTests, ToleranceEffect) {
    Eigen::MatrixXd A(3, 3);
    A << 1.0,     0.0,     0.0,
         0.0,     1e-8,   0.0,
         0.0,     0.0,     1e-10;
    
    // With strict tolerance_test
    Eigen::MatrixXd A_pinv_strict = PseudoInverse::compute(A, 1e-9);
    
    // With loose tolerance_test
    Eigen::MatrixXd A_pinv_loose = PseudoInverse::compute(A, 1e-7);
    
    // The two results should differ
    EXPECT_FALSE(A_pinv_strict.isApprox(A_pinv_loose, 1e-5));
    
    // The strict tolerance should ignore the small singular value
    // So A_pinv_loose should have an element (1,1) equal to zero.
    EXPECT_NEAR(A_pinv_loose(1, 1), 0.0, precision_test);
    EXPECT_GT(std::abs(A_pinv_strict(1, 1)), 1e6); // High value because 1/1e-8
}

/**
 * @brief Test pseudo-inverse on negative tolerance
 */
TEST_F(PseudoInverseTests, NegativeTolerance) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
    
    Eigen::MatrixXd A_pinv = PseudoInverse::compute(A, -1.0);
    
    // Should be equivalent to regular inverse
    EXPECT_TRUE(A_pinv.isApprox(A, precision_test));
}

// ============================================================================
// 5. MATHEMATICAL PROPERTIES
// ============================================================================

/**
 * @brief Test pseudo-inverse on the first Moore-Penrose property
 * 
 * A * A_pinv * A = A
 */
TEST_F(PseudoInverseTests, MoorePenroseFirstProperty) {
    Eigen::MatrixXd A(3, 4);
    A << 1, 2, 3, 4,
         5, 6, 7, 8,
         9, 10, 11, 12;
    
    Eigen::MatrixXd A_pinv = PseudoInverse::compute(A, tolerance_test);
    Eigen::MatrixXd result = A * A_pinv * A;
    
    EXPECT_TRUE(result.isApprox(A, precision_test));
}

/**
 * @brief Test pseudo-inverse on the second Moore-Penrose property
 * 
 * A_pinv * A * A_pinv = A_pinv
 */
TEST_F(PseudoInverseTests, MoorePenroseSecondProperty) {
    Eigen::MatrixXd A(3, 4);
    A << 1, 2, 3, 4,
         5, 6, 7, 8,
         9, 10, 11, 12;
    
    Eigen::MatrixXd A_pinv = PseudoInverse::compute(A, tolerance_test);
    Eigen::MatrixXd result = A_pinv * A * A_pinv;
    
    EXPECT_TRUE(result.isApprox(A_pinv, precision_test));
}

/**
 * @brief Test pseudo-inverse on the third Moore-Penrose property
 * 
 * (A * A_pinv)^T = A * A_pinv (symmetry)
 */
TEST_F(PseudoInverseTests, MoorePenroseThirdProperty) {
    Eigen::MatrixXd A(3, 4);
    A << 1, 2, 3, 4,
         5, 6, 7, 8,
         9, 10, 11, 12;
    
    Eigen::MatrixXd A_pinv = PseudoInverse::compute(A, tolerance_test);
    Eigen::MatrixXd product = A * A_pinv;
    
    EXPECT_TRUE(product.isApprox(product.transpose(), precision_test));
}

/**
 * @brief Test pseudo-inverse on the fourth Moore-Penrose property
 * 
 * (A_pinv * A)^T = A_pinv * A (symmetry)
 */
TEST_F(PseudoInverseTests, MoorePenroseFourthProperty) {
    Eigen::MatrixXd A(3, 4);
    A << 1, 2, 3, 4,
         5, 6, 7, 8,
         9, 10, 11, 12;
    
    Eigen::MatrixXd A_pinv = PseudoInverse::compute(A, tolerance_test);
    Eigen::MatrixXd product = A_pinv * A;
    
    EXPECT_TRUE(product.isApprox(product.transpose(), precision_test));
}

// ============================================================================
// 6. PERFORMANCE / ROBUSTNESS
// ============================================================================

/**
 * @brief Test pseudo-inverse on a large matrix to assess performance and robustness
 */
TEST_F(PseudoInverseTests, LargeMatrix) {
    // Create a large random matrix of 100 lines and 50 columns
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(100, 50);
    
    Eigen::MatrixXd A_pinv = PseudoInverse::compute(A, tolerance_test);
    
    // Verify the dimensions
    EXPECT_EQ(A_pinv.rows(), 50);
    EXPECT_EQ(A_pinv.cols(), 100);
    
    // 1st property of Moore-Penrose: A * A_pinv * A = A
    Eigen::MatrixXd result = A * A_pinv * A;
    EXPECT_TRUE(result.isApprox(A, 1e-6)); // Greater tolerance for large matrices
}