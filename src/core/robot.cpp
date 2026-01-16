#include "core/robot.hpp"

robot::core::Robot::Robot(RobotConfig _config)
: m_config(std::move(_config))
{
    // Initialize robot state based on configuration
    m_state.jointPositions.resize(m_config.jointCount(), 0.f);
    m_state.jointVelocities.resize(m_config.jointCount(), 0.f);
}

const robot::core::RobotConfig& robot::core::Robot::config() const noexcept {
    return m_config;
}

const robot::core::RobotState& robot::core::Robot::state() const noexcept {
    return m_state;
}

robot::core::RobotState& robot::core::Robot::state() noexcept {
    return m_state;
}
