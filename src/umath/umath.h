#ifndef UCLOTH_UMATH_H_
#define UCLOTH_UMATH_H_

#include <glm/glm.hpp>
#include <cmath>

namespace ucloth
{
namespace umath
{
using Vec3 = glm::vec3;
using Position = Vec3;
using Quaternion = glm::vec4;
using Real = float;
using Mat3x3 = glm::mat3;

constexpr Real k_div_by_zero_guard = 1e-8;
constexpr Real k_epsilon = 1e-8;

inline Real dot(Vec3 a, Vec3 b)
{
    return glm::dot(a, b);
}
inline Vec3 cross(Vec3 a, Vec3 b)
{
    return glm::cross(a, b);
}
inline Mat3x3 inverse(Mat3x3 const& mat)
{
    return glm::inverse(mat);
}
inline Mat3x3 transpose(Mat3x3 const& mat)
{
    return glm::transpose(mat);
}
inline Vec3 normalize(Vec3 const& v)
{
    return glm::normalize(v);
}
inline Real length(Vec3 const& v)
{
    return glm::length(v);
}

}  // namespace umath
}  // namespace ucloth

#endif  //! UCLOTH_UMATH_H_