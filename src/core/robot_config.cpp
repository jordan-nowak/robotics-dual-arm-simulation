#include "core/robot_config.hpp"

robot::core::RobotConfig::RobotConfig(std::string _name,
                         std::vector<JointConfig> _joints,
                         std::vector<LinkConfig> _links)
: m_name(std::move(_name))
, m_joints(std::move(_joints))
, m_links(std::move(_links))
{
}

const std::string& robot::core::RobotConfig::name() const noexcept {
    return m_name;
}

std::size_t robot::core::RobotConfig::jointCount() const noexcept {
    return m_joints.size();
}

const std::vector<robot::core::JointConfig>& robot::core::RobotConfig::joints() const noexcept {
    return m_joints;
}

std::size_t robot::core::RobotConfig::linkCount() const noexcept {
    return m_links.size();
}

const std::vector<robot::core::LinkConfig>& robot::core::RobotConfig::links() const noexcept {
    return m_links;
}
