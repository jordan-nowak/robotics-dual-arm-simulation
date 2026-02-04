#pragma once

#include <Eigen/SVD>
#include <Eigen/Dense>

namespace robot::control {

    /**
     * @brief Pseudo-inverse of a matrix
     *
     * This class represents the pseudo-inverse of a matrix.
     *
     * It proposes:
     *  - compute the pseudo-inverse of a matrix
     *  - compute the pseudo-inverse with a tolerance
     */
    class PseudoInverse {
        public:
            /**
             * @brief Compute the Moore-Penrose pseudo-inverse of a matrix using SVD
             *
             * This function computes the pseudo-inverse `A_pinv` of a matrix `A` such that:
             *  - A_pinv = pseudo_inverse(A)
             *  - A * A_pinv * A = A
             *  - A_pinv * A * A_pinv = A_pinv
             *
             * It is robust to:
             *  - non-square matrices
             *  - rank-deficient matrices
             *  - ill-conditioned matrices (via tolerance)
             * 
             * Noted:
             *  Small singular values correspond to directions that are numerically 
             *  unstable or non-observable. Ignoring them avoids exploding joint velocities.
             * 
             * @param _matrix    Input matrix
             * @param _tolerance Threshold below which singular values are considered zero
             * @return Eigen::MatrixXd The pseudo-inverse of the input matrix
             */
            static Eigen::MatrixXd compute(
                const Eigen::MatrixXd& _matrix,
                double _tolerance = 1e-6
            );
            
    };

}; // namespace robot::control