#ifndef UCLOTH_UCLOTH_COMMON_H_
#define UCLOTH_UCLOTH_COMMON_H_

#include <array>
#include <cstdint>
#include <umath/umath.h>
#include <utility>
#include <vector>

namespace ucloth
{
namespace simulation
{
// A particle is an index within the vectors of the world
using Particle = int;
using Force = umath::Vec3;
struct Applied_force
{
    Particle dest;
    Force val;
};

// Clockwise definition of a geometry face referring to particles in world.
using Face = std::array<Particle, 3>;
using Edge = std::pair<Particle, Particle>;
// using Mesh = std::vector<Face>;

enum class Mesh_type : uint32_t
{
    static_mesh,
    rigid_body,
    cloth
};

struct Mesh
{
    std::vector<Face> faces;
    Particle begin;  //< Defines the range in the world's particles.
    Particle end;
    umath::Real cloth_thickness;  // only used if cloth
    Mesh_type type;
};
}  // namespace simulation
}  // namespace ucloth

#endif  //! UCLOTH_UCLOTH_COMMON_H_