#pragma once

#include <vector>

namespace robot::core {

/**
 * @brief Current state of the robot at a given time
 */
struct RobotState {
    std::vector<double> jointPositions;   /*!< Current joint positions */
    std::vector<double> jointVelocities;  /*!< Current joint velocities */
};

} // namespace robot::core
