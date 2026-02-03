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

        std::vector<Transform> frames(m_config.linkCount());
        std::vector<Transform> intermediateFrames(m_config.linkCount());
        Transform T_current;

        // First, compute all individual link transformations
        for (std::size_t i = 0; i < m_config.linkCount(); ++i) {
            const auto& dh = m_config.links()[i];
            
            Transform T_i = Transform::fromModifiedDH(
                dh.a, dh.alpha, dh.d, dh.theta + _jointPositions[i]
            );

            // Store the intermediate transformation
            intermediateFrames[i] = T_i;
        }
        
        // Then, compute the cumulative transformations from base to each link
        for (std::size_t i = 0; i < m_config.linkCount(); ++i) {
            int current = static_cast<int>(i);
            
            // Traverse from base to the `i` link
            while (current != -1) {
                frames[i] = intermediateFrames[current] * frames[i];

                // Take the parent link for the next iteration
                current = m_config.links()[current].ID_parent;
            }
        }
        return frames;
    }

} // namespace robot::modeling::geometric