#pragma once

#include <Eigen/Dense>
#include "core/robot.hpp"
#include "modeling/geometric/dh_model.hpp"

namespace robot::task {

    /**
     * @brief Base class for control tasks
     */
    class Task {
    public:
        /**
         * @brief Construct a task
         * @param _name Task name
         * @param _priority Task priority (lower = higher priority)
         */
        Task(const std::string& _name, int _priority)
            : m_name(_name), m_priority(_priority)
        {}

        virtual ~Task() = default;

        virtual Eigen::VectorXd computeError(
            const core::Robot& _robot
        ) const = 0;

        virtual Eigen::MatrixXd getJacobian(
            const core::Robot& _robot
        ) const = 0;

        virtual int getDimension() const = 0;

        virtual bool isAchieved(
            const core::Robot& _robot,
            double _precision
        ) const;

        std::string getName() const { return m_name; }

        int getPriority() const { return m_priority; }

        void setName(const std::string& _name) { m_name = _name; }

        void setPriority(int _priority) { m_priority = _priority; }

    protected:
        std::string m_name;      /*!< Task name */
        int m_priority;          /*!< Task priority (0 = highest) */
    };

} // namespace robot::task