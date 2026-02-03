#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <unordered_set>
#include <Eigen/Dense>

namespace robot::core {

    #define ORIGIN -1 /*!< Origin of the coordinate system */

    /**
     * @brief Static configuration of a robot joint
     */
    struct JointConfig {
        std::string name;   /*!< Joint name */
        float minPosition;  /*!< Min joint position (rad) */
        float maxPosition;  /*!< Max joint position (rad) */
        float maxVelocity;  /*!< Max joint velocity (rad/s) */
    };

    /**
     * @brief Configuration of a robot link using DH parameters
     */
    struct LinkConfig {
        std::string name;       /*!< Link name */

        // --- Kinematic parameters (DH)
        float a;                /*!< Link length */
        float alpha;            /*!< Link twist */
        float d;                /*!< Link offset */
        float theta;            /*!< Joint offset */

        // --- Physical properties
        float mass;             /*!< Link mass */
        Eigen::Vector3d com;    /*!< Center of mass position to the link reference frame */
        
        // --- Structure
        int ID_link;            /*!< Index of current link */
        int ID_parent = ORIGIN; /*!< Index of parent link (`ORIGIN` if root) */
        bool motor;             /*!< Whether the link has a motor */
    };

    /**
     * @brief Static robot configuration
     *
     * Describes the robot structure and limits.
     * This class is immutable during simulation.
     */
    class RobotConfig {
    public:
        /** 
         * @brief Construct a robot configuration
         * @param _name Robot name
         * @param _joints Joint configuration list
         * @param _links Link configuration list
         */
        RobotConfig(std::string _name, std::vector<JointConfig> _joints, std::vector<LinkConfig> _links);

        /** @return Robot name */
        const std::string& name() const noexcept;
        
        /** @return Number of joints */
        std::size_t jointCount() const noexcept;
        
        /** @return Joint configurations */
        const std::vector<JointConfig>& joints() const noexcept;
        
        /** @return Number of links */
        std::size_t linkCount() const noexcept;

        /** @return Link configurations */
        const std::vector<LinkConfig>& links() const noexcept;

    private:
        std::string m_name; /*!< Robot name */
        std::vector<JointConfig> m_joints; /*!< Joint configurations */
        std::vector<LinkConfig> m_links; /*!< Joint configurations */
        
        /** 
         * @brief Check if the robot has a single base link
         * @return True if there is exactly one base link
         */
        bool _hasSingleBase() const;

        /** 
         * @brief Check if all links can reach the base link
         * @return True if the robot is fully connected
         */
        bool _allLinksReachBase() const;
    };

} // namespace robot::core