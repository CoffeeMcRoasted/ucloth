#include "collision.h"

namespace ucloth
{
namespace simulation
{
std::tuple<bool, umath::Vec3> ray_triangle_intersection(umath::Position const& orig,
                                                        umath::Vec3 const& dir,
                                                        umath::Position const& v0,
                                                        umath::Position const& v1,
                                                        umath::Position const& v2)
{
    // Moller-Trumbore intersection algorithm
    umath::Vec3 const edge1 = v1 - v0;
    umath::Vec3 const edge2 = v2 - v0;
    umath::Vec3 const h = umath::cross(dir, edge2);
    umath::Real const a = umath::dot(edge1, h);
    if (a > -umath::k_epsilon && a < umath::k_epsilon)
    {
        return std::make_tuple(false, umath::Vec3());
    }

    umath::Real const f = 1.0 / a;
    umath::Vec3 const s = orig - v0;
    umath::Real const u = f * umath::dot(s, h);
    if (u < 0.0 || u > 1.0)
    {
        return std::make_tuple(false, umath::Vec3());
    }

    umath::Vec3 const q = umath::cross(s, edge1);
    umath::Real const v = f * umath::dot(dir, q);
    if (v < 0.0 || u + v > 1.0)
    {
        return std::make_tuple(false, umath::Vec3());
    }

    umath::Real const t = f * umath::dot(dir, q);
    if (t > umath::k_epsilon)
    {
        umath::Vec3 const intersection_point = orig + dir * t;
        return std::make_tuple(true, intersection_point);
    }
}
}  // namespace simulation
}  // namespace ucloth