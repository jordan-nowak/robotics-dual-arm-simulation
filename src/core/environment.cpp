#include "core/environment.hpp"

namespace robot::core {

Environment::Environment(EnvironmentConfig config)
: m_config(std::move(config)) {
}

sf::Vector2f Environment::worldSize() const noexcept {
    return m_config.worldSize;
}

float Environment::gravity() const noexcept {
    return m_config.gravity;
}

float Environment::timeStep() const noexcept {
    return m_config.timeStep;
}

} // namespace robot::core
