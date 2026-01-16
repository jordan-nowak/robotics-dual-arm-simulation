#pragma once

#include "core/robot_config.hpp"
#include "core/robot_state.hpp"

namespace robot::core {

/**
 * @brief Robot holding configuration and current state
 *
 * This class acts as a data container shared across tasks.
 */
class Robot {
public:
    /**
     * @brief Construct a robot instance
     *
     * Initializes the robot with a static configuration and
     * creates a coherent initial dynamic state.
     *
     * @param _config Static robot configuration
     */
    explicit Robot(RobotConfig _config);

    /**
     * @brief Access the robot static configuration
     *
     * @return Constant reference to the robot configuration
     */
    const RobotConfig& config() const noexcept;

    /**
     * @brief Access the current robot state (read-only)
     *
     * Allows read-only access to the current dynamic state
     * of the robot. This overload is used when the Robot
     * instance itself is const.
     *
     * @return Constant reference to the robot state
     */
    const RobotState& state() const noexcept;

    /**
     * @brief Access the current robot state (read/write)
     *
     * Allows modification of the current dynamic state
     * of the robot. This overload is used when the Robot
     * instance is non-const.
     *
     * @return Reference to the robot state
     */
    RobotState& state() noexcept;

private:
    RobotConfig m_config;  /*!< Static robot configuration */
    RobotState m_state;    /*!< Current robot state */
};

} // namespace robot::core
