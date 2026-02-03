#include "modeling/geometric/center_of_mass.hpp"

namespace robot::modeling::geometric {
    
CenterOfMass::CenterOfMass(const robot::core::RobotConfig& _config)
: m_config(_config) {
    m_totalMass = 0.0;
    for (const auto& link : _config.links())
        m_totalMass += link.mass;
}

const double CenterOfMass::getTotalMass() const noexcept {
    return m_totalMass;
}

Eigen::Vector3d CenterOfMass::getLinkCoM(
    std::size_t _linkIndex,
    const std::vector<double>& _jointPositions) const 
{
    if (_linkIndex >= m_config.linkCount())
            throw std::out_of_range("CenterOfMass::getLinkCoM: Link index out of range");
    
    if (_jointPositions.size() != m_config.jointCount())
        throw std::invalid_argument("CenterOfMass::getLinkCoM: '_jointPositions' vector size mismatch");
        
    const auto& link = m_config.links()[_linkIndex];

    Eigen::Vector4d com_static(
        link.com.x(),
        link.com.y(),
        link.com.z(),
        1.0
    );

    DHModel model(m_config);
    Transform T_link = model.baseToLink(_linkIndex, _jointPositions);

    Eigen::Vector4d com_link = T_link.matrix() * com_static;

    return com_link.head<3>();
}

Eigen::Vector3d CenterOfMass::getGlobalCoM(
    const std::vector<double>& _jointPositions) const
{
    if (_jointPositions.size() != m_config.jointCount())
        throw std::invalid_argument("CenterOfMass::getGlobalCoM: '_jointPositions' vector size mismatch");
            
    Eigen::Vector3d com = Eigen::Vector3d::Zero();

    for (std::size_t i = 0; i < m_config.links().size(); ++i)
        com += m_config.links()[i].mass * getLinkCoM(i, _jointPositions);

    return com / m_totalMass;
}

} // namespace kinematics
