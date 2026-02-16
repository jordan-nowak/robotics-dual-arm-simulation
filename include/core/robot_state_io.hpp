#pragma once

#include <vector>
#include <ostream>
#include "core/robot_state.hpp"

namespace robot::core {

    inline std::ostream& operator<<(
        std::ostream& os,
        const RobotState& state
    ) {
        os << "Joint positions:  [ ";
        for (auto q : state.jointPositions)
            os << q << " ";
        os << "]\n";
        os << "Joint velocities: [ ";
        for (auto dq : state.jointVelocities)
            os << dq << " ";
        os << "]";
        return os;
    }

} // namespace robot::core