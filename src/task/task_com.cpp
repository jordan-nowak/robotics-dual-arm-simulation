#include "task/task_com.hpp"

namespace robot::task {

TaskCoM::TaskCoM(
    Eigen::Vector3d& _desired_com,
    const core::RobotConfig& _config,
    const std::string& _name,
    int _priority
)
    : Task(_name, _priority),
      m_desired_com(_desired_com),
      m_com(_config),
      m_jacobian(_config)
{
}

Eigen::VectorXd TaskCoM::computeError(
    const core::Robot& _robot
) const {
    Eigen::Vector3d current_com = m_com.getGlobalCoM(_robot.state().jointPositions);

    // Return only the X component
    Eigen::VectorXd error(1);
    error(0) = m_desired_com.x() - current_com.x();

    return error;
}

Eigen::MatrixXd TaskCoM::getJacobian(
    const core::Robot& _robot
) const {
    Eigen::MatrixXd J_full = m_jacobian.compute(_robot.state().jointPositions);
    return J_full.topRows(1); // position on the X-axis only is control 
    // return m_jacobian.compute(_robot.state().jointPositions);
}

void TaskCoM::setDesiredCoM(const Eigen::Vector3d& _com) {
    m_desired_com = _com;
}

} // namespace robot::task