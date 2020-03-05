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
    std::vector<umath::Vec3> velocities;
    std::vector<umath::Real> inverse_particle_masses;
    // Particle relations
    std::vector<Distance_constraint> distance_constraints;
    std::vector<Bending_constraint> bending_constraints;
    // Accelerations global to the simulation
    std::vector<umath::Vec3> accelerations;
    // Geometrical structures within the simulation
    std::vector<Mesh> meshes;

    void clear();
    void reserve_for_particles(size_t const n_particles);
    void add_acceleration(umath::Vec3 acceleration);
    Mesh const& add_cloth(std::vector<umath::Position> const& pos,
                          Mesh const& mesh,
                          umath::Real cloth_mass,
                          umath::Real cloth_elasticity,
                          umath::Real cloth_bending_stifness);

    Mesh const& add_static_mesh(std::vector<umath::Position> const& pos, Mesh const& mesh);

    Mesh const& add_rigid_body(std::vector<umath::Position> const& pos, Mesh const& mesh, umath::Real mass);

private:
    void add_constraints_for_mesh(Mesh const& mesh, umath::Real cloth_elasticity, umath::Real cloth_bending_stifness);

    // std::vector<umath::Position> const& retrieve_cloth(Mesh const& mesh);
};
}  // namespace simulation
}  // namespace ucloth
#endif  //! UCLOTH_WORLD_H_