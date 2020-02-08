#ifndef UCLOTH_WORLD_H_
#define UCLOTH_WORLD_H_

#include <simulation/constraint.h>
#include <simulation/uclothcommon.h>
#include <umath/umath.h>
#include <vector>

namespace ucloth
{
namespace simulation
{
struct World
{
    // Persistent data across simulation cycles.
    // Particles
    std::vector<umath::Position> positions;
    std::vector<umath::Vec_3> velocities;
    std::vector<umath::Real> inverse_particle_masses;
    // Particle relations
    std::vector<Distance_constraint> distance_constraints;
    std::vector<Bending_constraint> bending_constraints;
    // Accelerations global to the simulation
    std::vector<umath::Vec_3> accelerations;
    // Geometrical structures within the simulation
    std::vector<Mesh> meshes;

    void clear();
    void reserve_for_particles(size_t const n_particles);
};
}  // namespace simulation
}  // namespace ucloth
#endif  //! UCLOTH_WORLD_H_