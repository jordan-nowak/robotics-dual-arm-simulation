#pragma once

#include <SFML/System/Vector2.hpp>
#include <vector>

namespace robot::core {

    constexpr float DefaultGravity = 9.81f; /*!< Earth gravity (m/s²) */

    /**
     * @brief Environment configuration for the simulation
     */
    struct EnvironmentConfig {
        sf::Vector2f worldSize; /*!< World dimensions (width, height) */
        float gravity = DefaultGravity; /*!< Gravity acceleration (m/s²) */
        float timeStep; /*!< Simulation timestep (s) */
    };

    /**
     * @brief Global environment parameters shared by the simulation.
     */
    class Environment {
    public:
        explicit Environment(EnvironmentConfig config);

        /**
         * @brief Access the current robot state (read/write)
         *
         * Allows modification of the current dynamic state
         * of the robot. This overload is used when the Robot
         * instance is non-const.
         *
         * @return World size
         */
        sf::Vector2f worldSize() const noexcept;

        /** @return Gravity value */
        float gravity() const noexcept;

        /** @return Simulation timestep */
        float timeStep() const noexcept;

    private:
        EnvironmentConfig m_config; /*!< Environment configuration */
    };

} // namespace robot::core