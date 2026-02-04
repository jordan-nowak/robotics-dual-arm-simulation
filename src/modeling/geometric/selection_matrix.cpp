#include "modeling/geometric/selection_matrix.hpp"

namespace robot::modeling::geometric {

SelectionMatrix::SelectionMatrix(const robot::core::RobotConfig& _config)
    : m_config(_config)
{
}

Eigen::MatrixXd SelectionMatrix::fromEndEffector(std::size_t _endEffectorIndex) const
{
    const auto& links = m_config.links();

    // Mark active links on the path to the root
    std::vector<bool> active(links.size(), false);

    int current = static_cast<int>(_endEffectorIndex);
    while (current != -1) {
        active[current] = true;
        current = links[current].ID_parent;
    }

    // Build selection matrix in joint space
    Eigen::MatrixXd selection_matrix = Eigen::MatrixXd::Zero(m_config.jointCount(), m_config.jointCount());

    std::size_t jointIndex = 0;
    for (std::size_t i = 0; i < links.size(); ++i) {
        // Important: keep `false` if the link does not have a motor.
        if (active[i] && links[i].motor)
            selection_matrix(jointIndex, jointIndex) = 1.0;

        ++jointIndex;
    }

    return selection_matrix;
}

} // namespace robot::modeling::geometric
