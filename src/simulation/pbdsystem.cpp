#include "pbdsystem.h"
#include <numeric>
#include <simulation/collision.h>
namespace ucloth
{
namespace simulation
{
void PBD_system::apply_external_accelerations(std::vector<umath::Vec3> const& accelerations,
                                              umath::Real const delta_time,
                                              std::vector<umath::Vec3>& velocities)
{
    for (umath::Vec3 const& acceleration : accelerations)
    {
        // the size for velocties and inverted masses should be the same.
        size_t const size = velocities.size();
        for (size_t i = 0; i < size; ++i)
        {
            velocities[i] += acceleration * delta_time;
        }
    }
}

void PBD_system::damp_velocity(umath::Real const k_damping,
                               std::vector<umath::Real> const& inverse_masses,
                               std::vector<umath::Position> const& positions,
                               std::vector<umath::Vec3>& velocities)
{
    // The positions, velocities and inverse_masses should be the same size.
    size_t const n_particles = positions.size();
    // Mass total of the system
    umath::Real const total_mass = std::accumulate(
        inverse_masses.begin(),
        inverse_masses.end(),
        0.0f,
        [](umath::Real input, ucloth::umath::Real const inverse_mass) { return std::move(input) + 1 / inverse_mass; });
    // Calculate the center of mass
    umath::Position xcm = {0, 0, 0};
    for (Particle p = 0; p < n_particles; ++p)
    {
        xcm += positions[p] / inverse_masses[p];
    }
    xcm /= total_mass;
    // Calculate the velocity of center of mass
    umath::Position vcm = {0, 0, 0};
    for (Particle p = 0; p < n_particles; ++p)
    {
        vcm += velocities[p] / inverse_masses[p];
    }
    vcm /= total_mass;
    // Calculate the angular velocity
    umath::Vec3 L = {0.0f, 0.0f, 0.0f};
    for (Particle p = 0; p < n_particles; ++p)
    {
        umath::Position const ri = positions[p] - xcm;
        L += umath::cross(ri, velocities[p] / inverse_masses[p]);
    }
    // Calculate Inertia
    // ri_prime is the skew-symmetric matrix that has the property ri_prime v = ri x v
    umath::Mat3x3 I = umath::Mat3x3{0.0f};
    for (Particle p = 0; p < n_particles; ++p)
    {
        umath::Position const ri = positions[p] - xcm;
        umath::Mat3x3 const ri_prime = {{0, ri.z, -ri.y}, {-ri.z, 0, ri.x}, {ri.y, -ri.x, 0}};
        I += ri_prime * umath::transpose(ri_prime) / inverse_masses[p];
    }

    umath::Vec3 angular_velocity = umath::inverse(I) * L;
    for (Particle p = 0; p < n_particles; ++p)
    {
        glm::vec3 const ri = positions[p] - xcm;
        glm::vec3 const deltaVel = vcm + umath::cross(angular_velocity, ri) - velocities[p];
        velocities[p] += k_damping * deltaVel;
    }
}

std::vector<simulation::Collision_constraint> PBD_system::generate_collision_constraints(
    std::vector<umath::Position> const& positions,
    std::vector<umath::Position> const& p_estimations,
    std::vector<simulation::Mesh> const& meshes,
    umath::Real const cloth_thickness)
{
    // All the vectors should be the same size.
    // TODO: Modify this to use and optimization structure.
    // We generate the xi -> pi rays
    size_t const n_particles = positions.size();
    std::vector<umath::Ray> rays;
    rays.reserve(n_particles);
    for (Particle p = 0; p < n_particles; ++p)
    {
        rays.emplace_back(umath::Ray{positions[p], p_estimations[p] - positions[p]});
    }
    std::vector<simulation::Collision_constraint> constraints;
    constraints.reserve(n_particles);
    for (Particle p = 0; p < n_particles; ++p)
    {
        for (auto const& mesh : meshes)
        {
            for (auto const& face : mesh)
            {
                // TODO: Revise constraint generation to
                auto [success, result] = simulation::ray_triangle_intersection(
                    rays[p].orig, rays[p].dir, positions[face[0]], positions[face[1]], positions[face[2]]);
                if (success)
                {
                    constraints.emplace_back(Collision_constraint{p, face[0], face[1], face[2], cloth_thickness, 1.0});
                }
            }
        }
    }
    constraints.shrink_to_fit();
}
}  // namespace simulation
}  // namespace ucloth