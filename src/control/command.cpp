#include "control/command.hpp"

namespace robot::control {

Command::Command(const Eigen::VectorXd& _values, Type _type)
    : m_values(_values), m_type(_type) {
        if (_values.size() == 0)
            throw std::invalid_argument("Command::Command: Invalid command dimension in the constructor");
    }

void Command::setLimits(
    const Eigen::VectorXd& _lower,
    const Eigen::VectorXd& _upper
) {
    if (_lower.size() != m_values.size() || _upper.size() != m_values.size())
        throw std::invalid_argument("Command::setLimits: Limit dimensions must match command dimension");
    
    m_lower_limits = _lower;
    m_upper_limits = _upper;
}

bool Command::saturate() {
    if (!m_lower_limits || !m_upper_limits)
        return false; // No limits set

    bool saturated = false;

    for (size_t i = 0; i < m_values.size(); ++i) {
        if (m_values(i) < (*m_lower_limits)(i)) {
            m_values(i) = (*m_lower_limits)(i);
            saturated = true;
        } else if (m_values(i) > (*m_upper_limits)(i)) {
            m_values(i) = (*m_upper_limits)(i);
            saturated = true;
        }
    }
    return saturated;
}

bool Command::isValid() const {
    if (!m_lower_limits || !m_upper_limits)
        return true; // No limits set

    for (size_t i = 0; i < m_values.size(); ++i) {
        if (m_values(i) < (*m_lower_limits)(i) || m_values(i) > (*m_upper_limits)(i)) {
            return false;
        }
    }
    return true;
}

void Command::scale(double _factor) {
    m_values *= _factor;
}

} // namespace robot::control