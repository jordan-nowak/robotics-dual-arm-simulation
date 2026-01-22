#pragma once

#include <string>
#include "modeling/geometric/transform.hpp"

namespace robot::modeling::geometric {

    /**
     * @brief Named coordinate frame in space
     *
     * A Frame represents a coordinate system attached to a physical
     * or logical entity (robot base, link, end-effector, sensor, etc.).
     *
     * It associates:
     *  - a semantic name
     *  - a rigid body transformation describing its pose
     *
     * The Frame struct is defined in the header file and contains only data members.
     * This class is mainly used for representation, debugging and visualization.
     */
    struct Frame {
        std::string m_name;   /*!< Frame identifier (e.g. "base", "link_1", "tool") */
        Transform m_pose;     /*!< Pose of the frame with respect to a reference frame */
    };

} // namespace robot::modeling::geometric