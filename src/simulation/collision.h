#ifndef UCLOTH_COLLISION_H_
#define UCLOTH_COLLISION_H_

#include <umath/umath.h>
#include <tuple>
namespace ucloth
{
namespace simulation
{
std::tuple<bool, umath::Vec3> ray_triangle_intersection(umath::Position const& orig,
                                                        umath::Vec3 const& dir,
                                                        umath::Position const& v1,
                                                        umath::Position const& v2,
                                                        umath::Position const& v3);
}
}  // namespace ucloth

#endif  // !UCLOTH_COLLISION_H_