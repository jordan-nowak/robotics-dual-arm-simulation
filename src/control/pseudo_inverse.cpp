#include "control/pseudo_inverse.hpp"

namespace robot::control {

    Eigen::MatrixXd PseudoInverse::compute(
        const Eigen::MatrixXd& _matrix,
        double _tolerance)
    {
        // If the matrix is empty, return an empty matrix
        if (_matrix.size() == 0)
            return Eigen::MatrixXd();

        // Compute the Singular Value Decomposition (SVD)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(
            _matrix,
            Eigen::ComputeThinU | Eigen::ComputeThinV
        );

        // Retrieve the singular values
        const auto& singularValues = svd.singularValues();

        // Build the pseudo-inverse of the singular value matrix S
        Eigen::MatrixXd S_pinv = Eigen::MatrixXd::Zero(
            svd.matrixV().cols(),
            svd.matrixU().cols()
        );

        // Invert singular values above the tolerance
        for (int i = 0; i < singularValues.size(); ++i) {
            if (singularValues(i) > _tolerance)
                S_pinv(i, i) = 1.0 / singularValues(i);
        }

        // Return the Moore-Penrose pseudo-inverse: A_pinv = V * S_pinv * U^T
        return svd.matrixV() * S_pinv * svd.matrixU().transpose();
    }

} // namespace robot::control
