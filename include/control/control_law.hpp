#pragma once

#include <vector>
#include <memory>
#include <algorithm>
#include "task/task.hpp"
#include "control/hierarchical.hpp"
#include "control/command.hpp"

namespace robot::control {

/**
 * @brief Status of task achievement
 */
struct TaskStatus {
    std::string task_name;
    int priority;
    double error_norm;
    bool achieved;
};

/**
 * @brief Control law that computes joint commands from tasks
 * 
 * Uses hierarchical task control to compute joint velocities/positions
 * that satisfy multiple tasks according to their priority.
 */
class ControlLaw {
public:
    ControlLaw(double _tolerance = 1e-6);

    /**
     * @brief Add a task to the control law
     * @param _task Shared pointer to task
     */
    void addTask(std::shared_ptr<task::Task> _task);

    /**
     * @brief Remove all tasks
     */
    void clearTasks();

    /**
     * @brief Compute control command from current state
     * @param _current_state Current robot state
     * @return Command to apply
     */
    Command compute(const core::Robot& _robot);

    /**
     * @brief Check if all tasks are achieved
     * @param _robot Configuration and current robot state
     * @param _precision Tolerance for task achievement
     * @return True if all tasks are achieved within precision
     */
    bool areAllTasksAchieved(const core::Robot& _robot, double _precision) const;

    /**
     * @brief Check if a specific task is achieved
     * @param _task_index Index of the task in the task list
     * @param _robot Configuration and current robot state
     * @param _precision Tolerance for task achievement
     * @return True if task is achieved within precision
     */
    bool isTaskAchieved(
        std::size_t _task_index,
        const core::Robot& _robot,
        double _precision
    ) const;

    /**
     * @brief Get status of all tasks
     * @param _robot Configuration and current robot state
     * @param _precision Tolerance for achievement check
     * @return Vector of task statuses
     */
    std::vector<TaskStatus> getTasksStatus(
        const core::Robot& _robot,
        double _precision
    ) const;

    /**
     * @brief Get number of tasks
     */
    std::size_t getTaskCount() const { return m_tasks.size(); }
    
    /**
     * @brief Set solver tolerance
     */
    void setTolerance(double _tolerance);

private:
    /**
     * @brief Sort tasks by priority (lower priority value = higher importance)
     */
    void _sortTasksByPriority();

    std::vector<std::shared_ptr<task::Task>> m_tasks;
    Hierarchical m_hierarchical_solver;
    double m_tolerance;
};

} // namespace robot::control