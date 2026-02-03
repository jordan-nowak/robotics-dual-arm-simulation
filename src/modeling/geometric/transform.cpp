#include "modeling/geometric/transform.hpp"
#include <cmath>

namespace robot::modeling::geometric {

    Transform::Transform() 
    {
        m_T.setIdentity();
    }

    Transform Transform::fromModifiedDH(
        float _a, 
        float _alpha, 
        float _d, 
        float _theta) 
    {
        Transform T;

        const float c_theta = std::cos(_theta);
        const float s_theta = std::sin(_theta);
        const float c_alpha = std::cos(_alpha);
        const float s_alpha = std::sin(_alpha);

        T.m_T <<
            c_theta,            -s_theta,           0.f,        _a,
            c_alpha * s_theta,  c_alpha * c_theta,  -s_alpha,   -s_alpha * _d,
            s_alpha * s_theta,  s_alpha * c_theta,  c_alpha,    c_alpha * _d,
            0.f,                0.f,                0.f,        1.f;

        return T;
    }

    Transform Transform::operator*(
        const Transform& _other) const
    {
        Transform result;
        result.m_T = m_T * _other.m_T;
        return result;
    }

    const Eigen::Matrix4d& Transform::matrix() const noexcept 
    {
        return m_T;
    }

    Eigen::Vector3d Transform::position() const noexcept 
    {
        return m_T.block<3,1>(0,3);
    }

    Eigen::Matrix3d Transform::rotation() const noexcept 
    {
        return m_T.block<3,3>(0,0);
    }

} // namespace robot::modeling::geometric



