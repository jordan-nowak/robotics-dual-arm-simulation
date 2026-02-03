#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>

#include "core/robot_config.hpp"
#include "modeling/geometric/dh_model.hpp"

namespace robot::modeling::geometric {

/**
 * @brief Utility class to compute center of mass related quantities.
 *
 * This class provides methods to compute:
 *  - the center of mass of each link expressed in the world frame
 *  - the total mass of the robot
 *  - the global center of mass of the robot
 *
 * It relies on the robot kinematic model (forward kinematics),
 * but does not perform any control or optimization.
 */
class CenterOfMass {
public:
    /** 
    * @brief Construct a CenterOfMass solver for a given robot configuration.
    * @param _config Configuration of the robot.
    */
    explicit CenterOfMass(const robot::core::RobotConfig& _config);

    /**
     * @brief Get the total mass of the robot.
     *
     * @param _config The robot configuration containing link masses.
     * @return Total mass of the robot.
     */
    const double getTotalMass() const noexcept;

    /**
     * @brief Compute the center of mass of a given link in the world frame.
     *
     * The center of mass position is computed as:
     * CoM_i = T_i * [com_x, com_y, com_z, 1]^T
     *
     * @param _jointPositions The joint positions.
     * @param _linkIndex Index of the link.
     * @return 3D position of the link center of mass in the world frame.
     */
    Eigen::Vector3d getLinkCoM(
        std::size_t _linkIndex,
        const std::vector<double>& _jointPositions) const;

    /**
     * @brief Compute the global center of mass of the robot.
     *
     * The global center of mass is computed as:
     * CoM = (1 / Mass_total) * sum_i ( mass_i * CoM_i )
     *
     * @param _jointPositions The joint positions.
     * @return 3D position of the global center of mass in the world frame.
     */
    Eigen::Vector3d getGlobalCoM(
        const std::vector<double>& _jointPositions) const;

    private:
        const robot::core::RobotConfig& m_config; /*!< Reference to robot static configuration */
        double m_totalMass {0.0}; /*!< Total mass of the robot */
};

} // robot::modeling::geometric
