#ifndef UCLOTH_UCLOTH_COMMON_H_
#define UCLOTH_UCLOTH_COMMON_H_

#include <array>
#include <umath/umath.h>
#include <vector>

namespace ucloth
{
namespace simulation
{
// A particle is an index within the vectors of the world
using Particle = size_t;
using Force = umath::Vec_3;
struct Applied_force
{
    Particle dest;
    Force val;
};

// Clockwise definition of a geometry face referring to particles in world.
using Face = std::array<Particle, 3>;
using Mesh = std::vector<Face>;
}  // namespace simulation
}  // namespace ucloth

#endif  //! UCLOTH_UCLOTH_COMMON_H_