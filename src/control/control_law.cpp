#include "control/control_law.hpp"

namespace robot::control {

ControlLaw::ControlLaw(double _tolerance)
    : m_tolerance(_tolerance) {}

void ControlLaw::addTask(std::shared_ptr<task::Task> task) {
    m_tasks.push_back(task);
    _sortTasksByPriority();
}

void ControlLaw::clearTasks() {
    m_tasks.clear();
}

void ControlLaw::_sortTasksByPriority() {
    std::sort(m_tasks.begin(), m_tasks.end(),
        [](const auto& a, const auto& b) {
            return a->getPriority() < b->getPriority();
        });
}

Command ControlLaw::compute(
    const core::Robot& _robot) {
    if (m_tasks.empty())
        return Command(Eigen::VectorXd::Zero(_robot.config().jointCount()));

    const int dof = _robot.config().jointCount();
    Eigen::VectorXd u_total = Eigen::VectorXd::Zero(dof);
    Eigen::MatrixXd P_prev = Eigen::MatrixXd::Identity(dof, dof);

    // Solve tasks hierarchically
    for (const auto& task : m_tasks) {
        // Get task Jacobian and error
        Eigen::MatrixXd J = task->getJacobian(_robot);
        Eigen::VectorXd error = task->computeError(_robot);

        // Compute hierarchical step
        Eigen::VectorXd delta_u = m_hierarchical_solver.hierarchicalStep(J, error, u_total, P_prev, m_tolerance);

        // Update total command
        u_total += delta_u;

        // Update projection for next task
        P_prev = m_hierarchical_solver.combinedProjectionMatrix(J, P_prev, m_tolerance);
    }

    return Command(u_total);
}

bool ControlLaw::areAllTasksAchieved(
    const core::Robot& _robot,
    double _precision
) const {
    if (m_tasks.empty())
        return true;

    for (const auto& task : m_tasks) {
        if (!task->isAchieved(_robot, _precision))
            return false;
    }
    return true;
}

bool ControlLaw::isTaskAchieved(
    std::size_t _task_index,
    const core::Robot& _robot,
    double _precision
) const {
    if (_task_index >= m_tasks.size())
        throw std::out_of_range("ControlLaw::isTaskAchieved: Task index out of range");

    return m_tasks[_task_index]->isAchieved(_robot, _precision);
}

std::vector<TaskStatus> ControlLaw::getTasksStatus(
    const core::Robot& _robot,
    double _precision
) const {
    std::vector<TaskStatus> statuses;
    statuses.reserve(m_tasks.size());

    for (const auto& task : m_tasks) {
        Eigen::VectorXd error = task->computeError(_robot);
        TaskStatus status;
        status.task_name = task->getName();
        status.priority = task->getPriority();
        status.error_norm = error.norm();
        status.achieved = status.error_norm < _precision;
        
        statuses.push_back(status);
    }

    return statuses;
}

void ControlLaw::setTolerance(double _tolerance) {
    m_tolerance = _tolerance;
}

} // namespace robot::control