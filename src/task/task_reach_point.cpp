#include "task/task_reach_point.hpp"

namespace robot::task {

TaskReachPoint::TaskReachPoint(
    Eigen::Vector3d& _desired_position,
    const core::RobotConfig& _config,
    int _link_index,
    const std::string& _name,
    int _priority    
)
    : Task(_name, _priority),
      m_desired_position(_desired_position),
      m_dh_model(_config),
      m_link_index(_link_index),
      m_jacobian(_config)
{
}

Eigen::VectorXd TaskReachPoint::computeError(
    const core::Robot& _robot
) const {
    _robot.state();
    Eigen::Vector3d current_position = m_dh_model.baseToLink(static_cast<int>(m_link_index), _robot.state().jointPositions).position();
    return m_desired_position - current_position;
}

Eigen::MatrixXd TaskReachPoint::getJacobian(
    const core::Robot& _robot
) const {
    Eigen::MatrixXd J_full = m_jacobian.compute(
        m_link_index,
        _robot.state().jointPositions
    );
    return J_full.topRows(3); // cartesian position only
}

void TaskReachPoint::setDesiredPosition(const Eigen::Vector3d& position) {
    m_desired_position = position;
}

} // namespace robot::task