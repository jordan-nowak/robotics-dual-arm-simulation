#pragma once

#include <Eigen/Dense>
#include <vector>

#include "core/robot_config.hpp"

namespace robot::modeling::geometric {

/**
 * @brief Builds joint-space selection matrices from kinematic paths.
 *
 * A path is defined as the set of joints between the root
 * and a given end-effector.
 */
class SelectionMatrix {
public:
    /**
     * @brief Construct a selection matrix helper.
     *
     * @param _config Robot configuration.
     */
    explicit SelectionMatrix(const robot::core::RobotConfig& _config);

    /**
     * @brief Build a selection matrix for a root-to-end-effector path.
     *
     * @param endEffectorIndex Index of the end-effector link.
     * @return Selection matrix (n_dof x n_dof).
     */
    Eigen::MatrixXd fromEndEffector(std::size_t _endEffectorIndex) const;

private:
    const robot::core::RobotConfig& m_config; /*!< Reference to robot static configuration */
};

} // namespace robot::modeling::geometric