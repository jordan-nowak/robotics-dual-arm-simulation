#pragma once

#include "control/pseudo_inverse.hpp"

namespace robot::control {

    /**
     * @brief Hierarchical control functions
     *
     * This class represents the hierarchical control functions.
     *
     * It proposes:
     *  - compute the projection matrix of a matrix
     *  - compute a hierarchical step
     */
    class Hierarchical {
        public:
            /**
             * @brief Compute the projection matrix of a matrix using its pseudo-inverse
             *
             * This function computes the projection matrix `P` of a matrix `J` such that:
             *  - P = I - J_pinv * J
             * 
             * @param _J_matrix    Input matrix `J`
             * @param _tolerance Threshold below which singular values are considered zero
             * @return Eigen::MatrixXd The projection matrix of the input matrix
             */
            Eigen::MatrixXd projectionMatrix(
                const Eigen::MatrixXd& _J_matrix,
                double _tolerance = 1e-6
            );

            /**
             * @brief Compute combined projection matrix for hierarchical control
             * 
             * Combines the projection of the current task with projections from
             * all higher-priority tasks to maintain the full task hierarchy.
             * 
             * @param _J_matrix    Jacobian of current task
             * @param _P_prev      Combined projection from higher-priority tasks
             * @param _tolerance   Tolerance for pseudo-inverse
             * @return Combined projection: P_current * P_prev
             */
            Eigen::MatrixXd combinedProjectionMatrix(
                const Eigen::MatrixXd& _J_matrix,
                const Eigen::MatrixXd& _P_prev,
                double _tolerance = 1e-6
            );

            /**
             * @brief Compute a hierarchical control step
             *
             * This function computes a hierarchical control step `delta_u` such that:
             *  - delta_u = J_eff_pinv * (error - J * u_prev)
             *  - J_eff = J * P_prev
             * 
             * @param _J_matrix    Jacobian matrix of the current task
             * @param _error     Error vector of the current task
             * @param _u_prev    Previous control input vector
             * @param _P_prev    Previous projection matrix
             * @param _tolerance Threshold below which singular values are considered zero
             * @return Eigen::VectorXd The hierarchical control at the current step
             */
            Eigen::VectorXd hierarchicalStep(
                const Eigen::MatrixXd& _J_matrix,
                const Eigen::VectorXd& _error,
                const Eigen::VectorXd& _u_prev,
                const Eigen::MatrixXd& P_prev ,
                double _tolerance = 1e-6
            );

    };
}; // namespace robot::control
