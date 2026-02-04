#pragma once

#include <vector>
#include <Eigen/Dense>

#include "core/robot_config.hpp"
#include "modeling/geometric/center_of_mass.hpp"
#include "modeling/geometric/dh_model.hpp"
#include "modeling/geometric/selection_matrix.hpp"

namespace robot::modeling::kinematic {

    /**
     * @brief Geometric Jacobian model of a robot
     *
     * This class computes the Jacobian matrix for the Center of Mass
     * that relates joint velocities to Cartesian velocities (angular + linear)
     * of the end-effector.
     *
     * The Jacobian is expressed in the base frame.
     * It uses the Modified Denavit–Hartenberg convention.
     *
     * J_CoM(q) * q_dot = [ Jv ; Jw ]
     */
    class JacobianCom {
    public:
        /**
         * @brief Construct a Jacobian model for the Centre of Mass
         *
         * @param _config Robot configuration
         */
        explicit JacobianCom(const robot::core::RobotConfig& _config);

        /**
         * @brief Compute the geometric Jacobian at the global Centre of Mass
         *
         * @param _jointPositions Joint positions (rad)
         * @return 6xN Jacobian matrix (N = number of joints)
         * 
         * The Jacobian rows are organized as follows:
         *  - [0:2] Jv = linear velocity
         *  - [3:5] Jw = angular velocity
         */
        Eigen::MatrixXd compute(
            const std::vector<double>& _jointPositions) const;

    private:
        const robot::core::RobotConfig& m_config;
        robot::modeling::geometric::DHModel m_dhModel;
    };

} // namespace robot::modeling::kinematic
