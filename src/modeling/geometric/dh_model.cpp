#include "modeling/geometric/dh_model.hpp"
#include <stdexcept>

namespace robot::modeling::geometric {

    DHModel::DHModel(const robot::core::RobotConfig& _config)
        : m_config(_config) 
    {
    }

    Transform DHModel::baseToLink(
        std::size_t _linkIndex,
        const std::vector<double>& _jointPositions) const
    {
        if (_linkIndex >= m_config.linkCount())
            throw std::out_of_range("DHModel::baseToLink: Link index out of range");
    
        if (_jointPositions.size() != m_config.jointCount())
            throw std::invalid_argument("DHModel::baseToLink: '_jointPositions' vector size mismatch");
        
        return frames(_jointPositions).at(_linkIndex);
    }

    Transform DHModel::baseToEndEffector(
        const std::vector<double>& _jointPositions) const 
    {
        if (_jointPositions.size() != m_config.jointCount())
            throw std::invalid_argument("DHModel::baseToEndEffector: '_jointPositions' vector size mismatch");
        
        return baseToLink(m_config.linkCount() - 1, _jointPositions);
    }

    std::vector<Transform> DHModel::frames(
        const std::vector<double>& _jointPositions) const
    {
        if (_jointPositions.size() != m_config.jointCount())
            throw std::invalid_argument("DHModel::frames: '_jointPositions' vector size mismatch");

        std::vector<Transform> frames;
        frames.reserve(m_config.linkCount());

        Transform T_current;

        for (std::size_t i = 0; i < m_config.linkCount(); ++i) {
            const auto& dh = m_config.links()[i];
            float theta = dh.theta + _jointPositions[i];

            Transform T_i = Transform::fromModifiedDH(
                dh.a, dh.alpha, dh.d, theta
            );

            T_current = T_current * T_i;
            frames.push_back(T_current);
        }

        return frames;
    }

} // namespace robot::modeling::geometric