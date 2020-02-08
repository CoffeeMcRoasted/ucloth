#include "pbdsystem.h"
#include <numeric>
namespace ucloth
{
namespace simulation
{
void PBD_system::apply_external_accelerations(std::vector<umath::Vec_3> const& accelerations,
                                              umath::Real const delta_time,
                                              std::vector<umath::Vec_3>& velocities)
{
    for (umath::Vec_3 const& acceleration : accelerations)
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
                               std::vector<umath::Vec_3>& velocities)
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
    umath::Vec_3 L = {0.0f, 0.0f, 0.0f};
    for (Particle p = 0; p < n_particles; ++p)
    {
        umath::Position const ri = positions[p] - xcm;
        L += umath::cross(ri, velocities[p] / inverse_masses[p]);
    }
    // Calculate Inertia
    // ri_prime is the skew-symmetric matrix that has the property ri_prime v = ri x v
    umath::Mat_3x3 I = umath::Mat_3x3{0.0f};
    for (Particle p = 0; p < n_particles; ++p)
    {
        umath::Position const ri = positions[p] - xcm;
        umath::Mat_3x3 const ri_prime = {{0, ri.z, -ri.y}, {-ri.z, 0, ri.x}, {ri.y, -ri.x, 0}};
        I += ri_prime * umath::transpose(ri_prime) / inverse_masses[p];
    }

    umath::Vec_3 angular_velocity = umath::inverse(I) * L;
    for (Particle p = 0; p < n_particles; ++p)
    {
        glm::vec3 const ri = positions[p] - xcm;
        glm::vec3 const deltaVel = vcm + umath::cross(angular_velocity, ri) - velocities[p];
        velocities[p] += k_damping * deltaVel;
    }
}
}  // namespace simulation
}  // namespace ucloth