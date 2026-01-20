#pragma once

#include <SFML/System/Vector2.hpp>
#include <vector>


namespace robot::core {
    
    #define EARTH_GRAVITY 9.81f /*!< Standard gravity on Earth (m/s^2) */
    
    /**
     * @brief Environment configuration for the simulation
     */
    struct EnvironmentConfig {
        sf::Vector2f worldSize; /*!< World dimensions (width, height) */
        float gravity = EARTH_GRAVITY; /*!< Gravity acceleration (m/s^2) */
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