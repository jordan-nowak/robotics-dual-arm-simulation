#pragma once

#include <Eigen/Dense>
#include <stdexcept>
#include <optional>

namespace robot::control {

/**
 * @brief Robot command with validation and saturation
 * 
 * Encapsulates joint velocities or positions with optional limits.
 */
class Command {
public:
    /**
     * @brief Command type
     */
    enum class Type {
        VELOCITY,      // Joint velocities
        POSITION,      // Joint positions
        ACCELERATION   // Joint accelerations
    };

    /**
     * @brief Construct command
     * @param _values Command values (velocities, positions, etc.)
     * @param _type Command type
     */
    explicit Command(
        const Eigen::VectorXd& _values,
        Type _type = Type::VELOCITY
    );

    /**
     * @brief Get command values
     */
    const Eigen::VectorXd& getValues() const { return m_values; }

    /**
     * @brief Get command type
     */
    Type getType() const { return m_type; }

    /**
     * @brief Get command dimension
     */
    int getDimension() const { return m_values.size(); }

    /**
     * @brief Set joint limits for saturation
     * @param _lower Lower limits
     * @param _upper Upper limits
     */
    void setLimits(
        const Eigen::VectorXd& _lower,
        const Eigen::VectorXd& _upper
    );

    /**
     * @brief Saturate command to respect limits
     * @return True if saturation was applied
     */
    bool saturate();

    /**
     * @brief Check if command is within limits
     */
    bool isValid() const;

    /**
     * @brief Scale command by a factor
     */
    void scale(double _factor);

private:
    Eigen::VectorXd m_values;
    Type m_type;
    std::optional<Eigen::VectorXd> m_lower_limits;
    std::optional<Eigen::VectorXd> m_upper_limits;
};

} // namespace robot::control
