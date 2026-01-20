#pragma once

#include <Eigen/Dense>

namespace robot::modeling::geometric {

    /**
     * @brief Rigid body transformation in space
     *
     * This class represents a homogeneous transformation matrix (4x4)
     * used to describe the position and orientation of a coordinate frame
     * with respect to another frame.
     *
     * It proposes:
     *  - initialize the transformation matrix
     *  - build the homogeneous transformation from Denavit–Hartenberg parameters
     *  - perform the composition of transformations
     */
    class Transform {
    public:
        /**
         * @brief Construct an identity matrix
         *
         * Initializes the transformation matrix to the identity matrix,
         * meaning no rotation and no translation.
         */
        Transform();

        /**
         * @brief Construct a transformation from Denavit–Hartenberg parameters (modified DH convention)
         *
         * Builds the homogeneous transformation corresponding to a single
         * Denavit–Hartenberg link using the modified DH convention.
         *
         * @param _a     Link length -> translation along the x_{i-1} axis (i.e., the previous frame)
         * @param _alpha Link twist (rad) -> rotation around the x_{i-1} axis (i.e., the previous frame)
         * @param _d     Link offset -> translation along the z_i axis (i.e., the current frame)
         * @param _theta Joint angle (rad) -> rotation around the z_i axis (i.e., the current frame)
         * 
         *
         * @return Transformation from the previous frame to the next frame
         */
        static Transform fromModifiedDH(float _a, float _alpha, float _d, float _theta);

        /**
         * @brief Compose two transformations
         *
         * Performs the composition of this transformation with another one.
         * The result corresponds to applying this transform first, followed
         * by the `_other` transform.
         *
         * Mathematically:
         * T_result = this * _other
         *
         * @param _other Transformation to apply after this one
         * @return Composed transformation
         */
        Transform operator*(const Transform& _other) const;

        /**
         * @brief Access to the homogeneous transformation matrix
         *
         * @return 4x4 homogeneous transformation matrix
         */
        const Eigen::Matrix4f& matrix() const noexcept;

        /**
         * @brief Position (translation) part of the transformation matrix
         *
         * @return 3x1 translation vector
         */
        Eigen::Vector3f position() const noexcept;

        /**
         * @brief Orientation (rotation matrix) part of the transformation matrix
         *
         * @return 3x3 rotation matrix
         */
        Eigen::Matrix3f rotation() const noexcept;
        
    private:
        Eigen::Matrix4f m_T;  /*!< Homogeneous transformation matrix */
    };

} // namespace robot::modeling::geometric
