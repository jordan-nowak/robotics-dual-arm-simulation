#include "task/task.hpp"

namespace robot::task {

    bool Task::isAchieved(
        const core::Robot& _robot,
        double _precision
    ) const {
        Eigen::VectorXd error = computeError(_robot);
        return error.norm() < _precision;
    }

} // namespace robot::task