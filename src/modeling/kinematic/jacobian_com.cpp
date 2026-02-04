#include "modeling/kinematic/jacobian_com.hpp"

namespace robot::modeling::kinematic {

    using robot::modeling::geometric::Transform;
    using robot::modeling::geometric::CenterOfMass;
    using robot::modeling::geometric::SelectionMatrix;
    
    JacobianCom::JacobianCom(const robot::core::RobotConfig& _config)
        : m_config(_config)
        , m_dhModel(_config) {
    }

    Eigen::MatrixXd JacobianCom::compute(
        const std::vector<double>& _jointPositions) const {

        const std::size_t nbJoints = m_config.jointCount();

        if (_jointPositions.size() != nbJoints)
            throw std::invalid_argument("JacobianCom::compute: '_jointPositions' vector size mismatch");

        // Transformation from base to Centre of Mass
        CenterOfMass com_config(m_config);
        
        // Compute global CoM position
        Eigen::Vector3d com = com_config.getGlobalCoM(_jointPositions);

        // Compute the Jacobian for Center of Mass
        Eigen::MatrixXd J_CoM_total = Eigen::MatrixXd::Zero(3, nbJoints);
        SelectionMatrix selection_matrix(m_config);
        for (std::size_t link_index = 0; link_index < m_config.linkCount(); ++link_index) {
            // Get mass and CoM position of the link in base frame
            Eigen::Vector3d com_link = com_config.getLinkCoM(link_index, _jointPositions);
            double mass_link = m_config.links()[link_index].mass;
            
            // Get selection matrix for each link CoM to compute J_CoM_i
            Eigen::MatrixXd S_CoM_i = selection_matrix.fromEndEffector(link_index);

            for (std::size_t joint_index = 0; joint_index < nbJoints; ++joint_index) {
                if (S_CoM_i(joint_index, joint_index) == 0)
                    continue;

                // Transformation from base to joint j
                Transform T_j = m_dhModel.baseToLink(joint_index, _jointPositions);
                Eigen::Vector3d p_j = T_j.position();
                Eigen::Vector3d z_j = T_j.rotation().col(2);

                // Compute j_com_ij = z_j cross (CoM_i - p_j)
                Eigen::Vector3d j_com_ij = z_j.cross(com_link - p_j);

                // Weighted sum by mass
                J_CoM_total.col(joint_index) += mass_link * j_com_ij;
            }
        }
        
        // Normalize by total mass
        J_CoM_total /= com_config.getTotalMass();

        return J_CoM_total;
    }

} // namespace robot::modeling::kinematic