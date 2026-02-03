#include <gtest/gtest.h>
#include "modeling/geometric/transform.hpp"

using namespace robot::modeling::geometric;

/// @brief Construction of identity transform
TEST(TransformTest, ConstructIdentity) {
    Transform T;

    const Eigen::Matrix4d& mat = T.matrix();

    EXPECT_FLOAT_EQ(mat(0,0), 1.f);
    EXPECT_FLOAT_EQ(mat(1,1), 1.f);
    EXPECT_FLOAT_EQ(mat(2,2), 1.f);
    EXPECT_FLOAT_EQ(mat(3,3), 1.f);

    EXPECT_FLOAT_EQ(mat(0,1), 0.f);
    EXPECT_FLOAT_EQ(mat(0,2), 0.f);
    EXPECT_FLOAT_EQ(mat(0,3), 0.f);
    EXPECT_FLOAT_EQ(mat(1,0), 0.f);
    EXPECT_FLOAT_EQ(mat(1,2), 0.f);
    EXPECT_FLOAT_EQ(mat(1,3), 0.f);
    EXPECT_FLOAT_EQ(mat(2,0), 0.f);
    EXPECT_FLOAT_EQ(mat(2,1), 0.f);
    EXPECT_FLOAT_EQ(mat(2,3), 0.f);
    EXPECT_FLOAT_EQ(mat(3,0), 0.f);
    EXPECT_FLOAT_EQ(mat(3,1), 0.f);
    EXPECT_FLOAT_EQ(mat(3,2), 0.f);
}

/// @brief Construction of transform from DH parameters
TEST(TransformTest, ConstructfromModifiedDH) {
    const float a = 1.f;
    const float alpha = 0.5f; // rad
    const float d = 2.f;
    const float theta = 1.f; // rad

    Transform T = Transform::fromModifiedDH(a, alpha, d, theta);

    const Eigen::Matrix4d& mat = T.matrix();

    // Expected values computed externally
    EXPECT_NEAR(mat(0,0), 0.5403023f, 1e-5f);
    EXPECT_NEAR(mat(0,1), -0.841471f, 1e-5f);
    EXPECT_NEAR(mat(0,2), 0.f, 1e-5f);
    EXPECT_NEAR(mat(0,3), 1.f, 1e-5f);

    EXPECT_NEAR(mat(1,0), 0.73846, 1e-5f);
    EXPECT_NEAR(mat(1,1), 0.47416, 1e-5f);
    EXPECT_NEAR(mat(1,2), -0.479426, 1e-5f);
    EXPECT_NEAR(mat(1,3), -0.958851, 1e-5f);

    EXPECT_NEAR(mat(2,0), 0.403423, 1e-5f);
    EXPECT_NEAR(mat(2,1), 0.259035, 1e-5f);
    EXPECT_NEAR(mat(2,2), 0.877583, 1e-5f);
    EXPECT_NEAR(mat(2,3), 1.75517, 1e-5f);

    EXPECT_FLOAT_EQ(mat(3,0), 0.f);
    EXPECT_FLOAT_EQ(mat(3,1), 0.f);
    EXPECT_FLOAT_EQ(mat(3,2), 0.f);
    EXPECT_FLOAT_EQ(mat(3,3), 1.f);
}

/// @brief Composition of two transforms
TEST(TransformTest, ComposeTransforms) {
    Transform T1 = Transform::fromModifiedDH(1.f, 0.f, 0.f, 0.f); // Translation along x by 1
    Transform T2 = Transform::fromModifiedDH(0.f, 0.f, 2.f, 0.f); // Translation along z by 2

    Transform T3 = T1 * T2;

    const Eigen::Matrix4d& mat = T3.matrix();

    EXPECT_FLOAT_EQ(mat(0,0), 1.f);
    EXPECT_FLOAT_EQ(mat(1,1), 1.f);
    EXPECT_FLOAT_EQ(mat(2,2), 1.f);
    EXPECT_FLOAT_EQ(mat(3,3), 1.f);

    EXPECT_FLOAT_EQ(mat(0,1), 0.f);
    EXPECT_FLOAT_EQ(mat(0,2), 0.f);
    EXPECT_FLOAT_EQ(mat(0,3), 1.f);
    EXPECT_FLOAT_EQ(mat(1,0), 0.f);
    EXPECT_FLOAT_EQ(mat(1,2), 0.f);
    EXPECT_FLOAT_EQ(mat(1,3), 0.f);
    EXPECT_FLOAT_EQ(mat(2,0), 0.f);
    EXPECT_FLOAT_EQ(mat(2,1), 0.f);
    EXPECT_FLOAT_EQ(mat(2,3), 2.f);
    EXPECT_FLOAT_EQ(mat(3,0), 0.f);
    EXPECT_FLOAT_EQ(mat(3,1), 0.f);
    EXPECT_FLOAT_EQ(mat(3,2), 0.f);
}

/// @brief Access to the transformation matrix
TEST(TransformTest, AccessMatrix) {
    Transform T = Transform::fromModifiedDH(1.f, 0.f, 0.f, 0.f);

    const Eigen::Matrix4d& mat = T.matrix();

    EXPECT_FLOAT_EQ(mat(0,0), 1.f);
    EXPECT_FLOAT_EQ(mat(1,1), 1.f);
    EXPECT_FLOAT_EQ(mat(2,2), 1.f);
    EXPECT_FLOAT_EQ(mat(3,3), 1.f);

    EXPECT_FLOAT_EQ(mat(0,3), 1.f); // x translation
    EXPECT_FLOAT_EQ(mat(1,3), 0.f); // y translation
    EXPECT_FLOAT_EQ(mat(2,3), 0.f); // z translation
}

/// @brief Extraction of position from homogeneous transformation matrix
TEST(TransformTest, PositionExtraction) {
    Transform T = Transform::fromModifiedDH(1.f, 0.f, 0.f, 0.f);

    Eigen::Vector3d position = T.position();
    
    EXPECT_FLOAT_EQ(position(0), 1.f);
    EXPECT_FLOAT_EQ(position(1), 0.f);
    EXPECT_FLOAT_EQ(position(2), 0.f);
}

/// @brief Extraction of rotation from homogeneous transformation matrix
TEST(TransformTest, RotationIsOrthonormal) {
    Transform T = Transform::fromModifiedDH(1.f, 0.f, 0.f, 0.f);
    
    Eigen::Matrix3d rotation = T.rotation();

    EXPECT_FLOAT_EQ(rotation(0,0), 1.f);
    EXPECT_FLOAT_EQ(rotation(0,1), 0.f);
    EXPECT_FLOAT_EQ(rotation(0,2), 0.f);
    EXPECT_FLOAT_EQ(rotation(1,0), 0.f);
    EXPECT_FLOAT_EQ(rotation(1,1), 1.f);
    EXPECT_FLOAT_EQ(rotation(1,2), 0.f);
    EXPECT_FLOAT_EQ(rotation(2,0), 0.f);
    EXPECT_FLOAT_EQ(rotation(2,1), 0.f);
    EXPECT_FLOAT_EQ(rotation(2,2), 1.f);
}
