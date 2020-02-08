#include "world.h"

namespace ucloth
{
namespace simulation
{
void World::clear()
{
    positions.clear();
    velocities.clear();
    inverse_particle_masses.clear();
    distance_constraints.clear();
    bending_constraints.clear();
    accelerations.clear();
    meshes.clear();
}
void World::reserve_for_particles(size_t const n_particles)
{
    positions.reserve(n_particles);
    velocities.reserve(n_particles);
    inverse_particle_masses.reserve(n_particles);
}
}  // namespace simulation
}  // namespace ucloth