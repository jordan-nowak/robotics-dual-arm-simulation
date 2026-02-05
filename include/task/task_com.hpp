#pragma once

#include "task/task.hpp"
#include "modeling/kinematic/jacobian_com.hpp"
#include "modeling/geometric/center_of_mass.hpp"

namespace robot::task {

    class TaskCoM : public Task {
        public:
            /**
             * @brief Construct CoM task
             * @param _desired_com Target CoM position
             * @param _config Robot configuration
             * @param _name Task name
             * @param _priority Task priority
             */
            TaskCoM(
                Eigen::Vector3d& _desired_com,
                const core::RobotConfig& _config,
                const std::string& _name = "CoM",
                int _priority = 1
            );

            Eigen::VectorXd computeError(
                const core::Robot& _robot
            ) const override;

            // return only the position
            Eigen::MatrixXd getJacobian(
                const core::Robot& _robot
            ) const override;

            int getDimension() const override { return 1; }

            // Specific methods
            Eigen::Vector3d getDesiredCoM() const { return m_desired_com; }
            void setDesiredCoM(const Eigen::Vector3d& _newDesiredCom);

        private:
            Eigen::Vector3d m_desired_com;
            modeling::geometric::CenterOfMass m_com;
            modeling::kinematic::JacobianCom m_jacobian;
    };

} // namespace robot::task