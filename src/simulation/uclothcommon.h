#ifndef UCLOTH_UCLOTH_COMMON_H_
#define UCLOTH_UCLOTH_COMMON_H_

#include <array>
#include <cstdint>
#include <umath/umath.h>
#include <utility>
#include <vector>
#include <optional>
#include <variant>

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

struct Attachment
{
    Particle p;
    umath::Real original_inv_mass;
    std::variant<Particle, umath::Position> destination;
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
    umath::Real k_velocity;       //< Defines damping of velocities. [0,1], 1 if regid body.
    umath::Real cloth_thickness;  //< Only used if cloth
    Mesh_type type;
};
}  // namespace simulation
}  // namespace ucloth

#endif  //! UCLOTH_UCLOTH_COMMON_H_