#pragma once

#include <vector>

namespace robot::core {

/**
 * @brief Current state of the robot at a given time
 */
struct RobotState {
    std::vector<float> jointPositions;   /*!< Current joint positions */
    std::vector<float> jointVelocities;  /*!< Current joint velocities */
};

} // namespace robot::core
