#ifndef UCLOTH_UMATH_H_
#define UCLOTH_UMATH_H_

#include <glm/glm.hpp>

namespace ucloth
{
namespace umath
{
using Vec_3 = glm::vec3;
using Position = Vec_3;
using Quaternion = glm::vec4;
using Real = float;
using Mat_3x3 = glm::mat3;

inline Vec_3 cross(Vec_3 a, Vec_3 b)
{
    return glm::cross(a, b);
}
inline Mat_3x3 inverse(Mat_3x3 const& mat)
{
    return glm::inverse(mat);
}
inline Mat_3x3 transpose(Mat_3x3 const& mat)
{
    return glm::transpose(mat);
}

}  // namespace umath
}  // namespace ucloth

#endif  //! UCLOTH_UMATH_H_