#pragma once

#include "task/task.hpp"
#include "modeling/kinematic/jacobian_model.hpp"

namespace robot::task {

    class TaskReachPoint : public Task {
        public:
            /**
             * @brief Construct end-effector task
             * @param _desired_position Target position
             * @param _config Robot configuration
             * @param _link_index Link to control
             * @param _name Task name
             * @param _priority Task priority
             */
            TaskReachPoint(
                Eigen::Vector3d& _desired_position,
                const core::RobotConfig& _config,
                int _link_index,
                const std::string& _name = "EndEffector",
                int _priority = 0
            );

            Eigen::VectorXd computeError(
                const core::Robot& _robot
            ) const override;

            // return only the position
            Eigen::MatrixXd getJacobian(
                const core::Robot& _robot
            ) const override;

            int getDimension() const override { return 3; }

            int getLinkIndex() const { return m_link_index; }

            void setLinkIndex(int _link_index) { m_link_index = _link_index; }
            
            Eigen::Vector3d getDesiredPosition() const { return m_desired_position; }

            void setDesiredPosition(const Eigen::Vector3d& _position);

        private:
            Eigen::Vector3d m_desired_position;             /*!< Desired position to reach */
            int m_link_index;                               /*!< Link index to control */
            modeling::geometric::DHModel m_dh_model;       /*!< ... */
            modeling::kinematic::JacobianModel m_jacobian;  /*!< ... */
    };

} // namespace robot::task