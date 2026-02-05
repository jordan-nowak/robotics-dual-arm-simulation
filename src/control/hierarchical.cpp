#include "control/hierarchical.hpp"

namespace robot::control {

Eigen::MatrixXd Hierarchical::projectionMatrix(
    const Eigen::MatrixXd& _J_matrix,
    double _tolerance
) {
    const Eigen::MatrixXd J_pinv = PseudoInverse::compute(_J_matrix, _tolerance);
    const int dof = _J_matrix.cols();
    return Eigen::MatrixXd::Identity(dof, dof) - J_pinv * _J_matrix;
}

Eigen::MatrixXd Hierarchical::combinedProjectionMatrix(
    const Eigen::MatrixXd& _J_matrix,
    const Eigen::MatrixXd& _P_prev,
    double _tolerance
) {
    Eigen::MatrixXd P_current = projectionMatrix(_J_matrix, _tolerance);
    return P_current * _P_prev;
}

Eigen::VectorXd Hierarchical::hierarchicalStep(
    const Eigen::MatrixXd& _J_matrix,
    const Eigen::VectorXd& _error,
    const Eigen::VectorXd& _u_prev,
    const Eigen::MatrixXd& _P_prev,
    double tolerance
) {
    // Projected (effective) Jacobian:
    // Jacobian of task i projected into the nullspace
    // of higher-priority tasks
    Eigen::MatrixXd J_eff = _J_matrix * _P_prev;
    Eigen::MatrixXd J_eff_pinv = PseudoInverse::compute(J_eff, tolerance);

    // Residual task error (compensation term):
    // Remaining task error after previous command
    Eigen::VectorXd rhs = _error - _J_matrix * _u_prev;

    return J_eff_pinv * rhs;
}

} // namespace robot::control
