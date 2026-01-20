#pragma once

#include <vector>
#include <cstddef>

#include "core/robot_config.hpp"
#include "modeling/geometric/transform.hpp"

namespace robot::modeling::geometric {

    /**
     * @brief Denavit–Hartenberg geometric model of a robot
     *
     * This class implements the geometric (forward kinematics) model
     * of a robot using the Modified Denavit–Hartenberg convention.
     *
     * It computes rigid body transformations between the robot base
     * frame and any intermediate link or the end-effector, based on:
     *  - the robot static configuration (DH parameters)
     *  - the current joint positions
     *
     * This class is stateless and performs only mathematical computations.
     */
    class DHModel {
    public:
        /**
         * @brief Construct a DH geometric model
         *
         * The model references the robot configuration, which contains
         * the static DH parameters defining the robot structure.
         *
         * @param _config Robot static configuration
         */
        explicit DHModel(const robot::core::RobotConfig& _config);

        /**
         * @brief Compute the transformation from base to a given link
         *
         * Computes the homogeneous transformation from the robot base
         * frame to the frame attached to the specified link index.
         *
         * @param _linkIndex Index of the link (0-based)
         * @param _jointPositions Current joint positions (rad)
         *
         * @return Transformation from base frame to the link frame
         */
        Transform baseToLink(
            std::size_t _linkIndex,
            const std::vector<float>& _jointPositions) const;

        /**
         * @brief Compute the transformation from base to end-effector
         *
         * Computes the homogeneous transformation from the robot base
         * frame to the end-effector frame by chaining all DH transformations.
         *
         * @param _jointPositions Current joint positions (rad)
         *
         * @return Transformation from base frame to the end-effector frame
         */
        Transform baseToEndEffector(
            const std::vector<float>& _jointPositions) const;

        /**
         * @brief Compute all intermediate frames from base to end-effector
         *
         * @param _jointPositions Current joint positions (rad)
         * @return Vector of transforms:
         *         frames[i] = base -> link[i]
         */
        std::vector<Transform> frames(
            const std::vector<float>& _jointPositions) const;

    private:
        const robot::core::RobotConfig& m_config; /*!< Reference to robot static configuration */
    };

} // namespace robot::modeling::geometric
