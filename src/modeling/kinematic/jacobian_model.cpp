#include "modeling/kinematic/jacobian_model.hpp"

namespace robot::modeling::kinematic {

    using robot::modeling::geometric::Transform;

    JacobianModel::JacobianModel(const robot::core::RobotConfig& _config)
        : m_config(_config)
        , m_dhModel(_config) {
    }

    Eigen::MatrixXd JacobianModel::compute(
        const std::vector<double>& _jointPositions) const {

        const std::size_t nbJoints = m_config.jointCount();

        if (_jointPositions.size() != nbJoints)
            throw std::invalid_argument("JacobianModel::compute: '_jointPositions' vector size mismatch");

        // Jacobian 6xN
        Eigen::MatrixXd Jacobian(6, nbJoints);
        Jacobian.setZero();

        // // Transformation from base to end-effector
        Transform T_ee = m_dhModel.baseToEndEffector(_jointPositions);
        Eigen::Vector3d p_ee = T_ee.position();

        for (std::size_t i = 0; i < nbJoints; ++i) {

            // Transformation from base to joint i
            Transform T_i = m_dhModel.baseToLink(i, _jointPositions);

            // Joint position
            Eigen::Vector3d p_i = T_i.position();

            // Rotation around Z-axis
            Eigen::Vector3d z_i = T_i.rotation().col(2);

            // Linear part of Jacobian
            Eigen::Vector3d Jv = z_i.cross(p_ee - p_i);

            // Angular part of Jacobian
            Eigen::Vector3d Jw = z_i;

            // Fill Jacobian
            Jacobian.block<3,1>(0, i) = Jv;
            Jacobian.block<3,1>(3, i) = Jw;
        }

        return Jacobian;
    }

} // namespace robot::modeling::kinematic
