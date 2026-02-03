#include "core/robot_config.hpp"

robot::core::RobotConfig::RobotConfig(std::string _name,
                         std::vector<JointConfig> _joints,
                         std::vector<LinkConfig> _links)
: m_name(std::move(_name))
, m_joints(std::move(_joints))
, m_links(std::move(_links))
{
    if (!_hasSingleBase())
        throw std::runtime_error("RobotConfig: multiple bases detected");

    if (!_allLinksReachBase())
        throw std::runtime_error("RobotConfig: Some links are not connected to the base");
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

bool robot::core::RobotConfig::_hasSingleBase() const {
    int count = 0;
    for (const auto& link : m_links) {
        if (link.ID_parent == -1)
            ++count;
    }

    if (count != 1)
        std::cout << "[ ERROR ] RobotConfig::hasSingleBase: Robot should have exactly one base link. Found " << count << " base links." << std::endl;

    return count == 1;
}

bool robot::core::RobotConfig::_allLinksReachBase() const {
    for (const auto& link : m_links) {
        int current = link.ID_link;
        std::unordered_set<int> visited;

        while (true) {
            if (current == -1)
                break;
    
            // Again the same ID -> infinite loop
            if (visited.count(current))
                return false; 

            // Save the current ID as visited
            visited.insert(current);

            // Again parent not found
            int parent = m_links[current].ID_parent;
            if (parent == ORIGIN)
                break;
            
            if (parent >= static_cast<int>(m_links.size()) || parent < 0) {
                std::cout << "[ ERROR ] RobotConfig::_hasExistingLink: Parent link index " << m_links[current].ID_parent << " is out of bounds (max: " << m_links.size() - 1 << ")." << std::endl;
                return false;
            }

            // Move to parent
            current = m_links[current].ID_parent;
        }
    }
    return true;
}