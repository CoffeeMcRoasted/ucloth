#include "constraint.h"
#

namespace ucloth
{
namespace simulation
{
void project_distance_constraints(std::vector<Distance_constraint> const& constraints,
                                  std::vector<umath::Real> const& inverse_masses,
                                  unsigned int const solver_iterations,
                                  std::vector<umath::Position>& positions)
{
    for (auto const& constraint : constraints)
    {
        umath::Position& p1 = positions[constraint.p1];
        umath::Position& p2 = positions[constraint.p2];
        umath::Real const w_1 = inverse_masses[constraint.p1];
        umath::Real const w_2 = inverse_masses[constraint.p2];

        umath::Vec3 const shared_value = (umath::length(p1 - p2) - constraint.distance) * (p1 - p2) /
                                         (umath::length(p1 - p2) * (w_1 + w_2) + umath::k_div_by_zero_guard);
        const float k_prime = 1 - (1 / powf(1 - constraint.stiffness, 1 / static_cast<umath::Real>(solver_iterations)));
        p1 += -w_1 * k_prime * shared_value;
        p2 += w_2 * k_prime * shared_value;
    }
}

void project_bending_constraints(std::vector<Bending_constraint> const& constraints,
                                 std::vector<umath::Real> const& inverse_masses,
                                 unsigned int const solver_iterations,
                                 std::vector<umath::Position>& positions)
{
    for (auto const& constraint : constraints)
    {
        // We set the frame of reference at p1
        umath::Position const p2 = positions[constraint.p2] - positions[constraint.p1];
        umath::Position const p3 = positions[constraint.p3] - positions[constraint.p1];
        umath::Position const p4 = positions[constraint.p4] - positions[constraint.p1];
        // Calculate normals and other required modules
        umath::Vec3 const cross_p2xp3 = umath::cross(p2, p3);
        umath::Vec3 const cross_p2xp4 = umath::cross(p2, p4);
        umath::Real const mod_p2xp3 = umath::length(cross_p2xp3);
        umath::Real const mod_p2xp4 = umath::length(cross_p2xp4);
        umath::Vec3 const n1 = umath::normalize(cross_p2xp3);
        umath::Vec3 const n2 = umath::normalize(cross_p2xp4);
        umath::Real const d = umath::dot(n1, n2);
        // Compute the gradients
        umath::Vec3 const q3 = glm::cross(p2, n2) + glm::cross(n1, p2) * d / mod_p2xp3;
        umath::Vec3 const q4 = glm::cross(p2, n1) + glm::cross(n2, p2) * d / mod_p2xp4;
        umath::Vec3 const q2 = -(glm::cross(p3, n2) + glm::cross(n1, p3) * d) / mod_p2xp3 -
                               (glm::cross(p4, n1) + glm::cross(n2, p4) * d) / mod_p2xp4;
        umath::Vec3 const q1 = -q2 - q3 - q4;
        // Compute the projections
        umath::Real const k =
            sqrtf(1 - d * d) * (acos(d) - constraint.dihedral_angle) /
            (inverse_masses[constraint.p1] * umath::length(q1) * umath::length(q1) +
             inverse_masses[constraint.p2] * umath::length(q2) * umath::length(q2) +
             inverse_masses[constraint.p3] * umath::length(q3) * umath::length(q3) +
             inverse_masses[constraint.p4] * umath::length(q4) * umath::length(q4) + umath::k_div_by_zero_guard);

        umath::Position const delta_p1 = inverse_masses[constraint.p1] * k * q1;
        umath::Position const delta_p2 = inverse_masses[constraint.p2] * k * q2;
        umath::Position const delta_p3 = inverse_masses[constraint.p3] * k * q3;
        umath::Position const delta_p4 = inverse_masses[constraint.p4] * k * q4;

        umath::Real const k_prime =
            1 - (1 / powf(1 - constraint.stiffness, 1 / static_cast<umath::Real>(solver_iterations)));

        positions[constraint.p1] += delta_p1 * k_prime;
        positions[constraint.p2] += delta_p2 * k_prime;
        positions[constraint.p3] += delta_p3 * k_prime;
        positions[constraint.p4] += delta_p4 * k_prime;
    }
}

void project_collisionconstraints(std::vector<Collision_constraint> const& constraints,
                                  std::vector<umath::Real> const& inverse_masses,
                                  unsigned int const solver_iterations,
                                  std::vector<umath::Position>& positions)
{
    // https://github.com/InteractiveComputerGraphics/PositionBasedDynamics/blob/master/PositionBasedDynamics/PositionBasedDynamics.cpp
    // https://pybullet.org/Bullet/phpBB3/viewtopic.php?f=4&t=2544&p=10171&hilit=Position+based+dynamics#p10171
    // This constraint is dependant on the side from which the ray is casted.
    // [ -N*u , -N*v , -N*(1-u-v) ]
    // (based on C=(q-(P0 u + P1 v + P2 (1-u-v))).N )
    // We calculate the barycentric coordinates to be able to derive the gradient.
    for (auto const& constraint : constraints)
    {
        float b0, b1, b2 = 1.0f / 3.0f;
        // We define the triangle.
        umath::Position& p1 = positions[constraint.p1];
        umath::Position& p2 = positions[constraint.p2];
        umath::Position& p3 = positions[constraint.p3];
        umath::Position& q = positions[constraint.q];

        umath::Vec3 const u = p2 - p1;
        umath::Vec3 const v = p3 - p1;
        umath::Vec3 const n = umath::cross(u, v);
        umath::Vec3 const w = q - p1;
        // We find the barycentric coordinates of the proyected point.
        float const gamma = umath::dot(umath::cross(u, w), n) / umath::dot(n, n);
        float const beta = umath::dot(umath::cross(w, v), n) / umath::dot(n, n);
        float const alfa = 1 - gamma - beta;
        // TODO: Point lies outside of the triangle!

        // Calculate normal in the direction of the point to retain side
        umath::Vec3 const qproj = alfa * p1 + beta * p2 + gamma * p3;
        umath::Vec3 dir_n = q - qproj;
        float const d = umath::length(dir_n);
        dir_n = umath::normalize(dir_n);
        float const C = d - constraint.thickness;

        umath::Vec3 const grad_q = dir_n;
        umath::Vec3 const grad_p1 = -dir_n * alfa;
        umath::Vec3 const grad_p2 = -dir_n * beta;
        umath::Vec3 const grad_p3 = -dir_n * gamma;
        // Module of the gradients is equal to the scalar (normal vector)
        float const s =
            C / (inverse_masses[constraint.q] + alfa * alfa * inverse_masses[constraint.p1] +
                 beta * beta * inverse_masses[constraint.p2] + gamma * gamma * inverse_masses[constraint.p3]);

        umath::Vec3 const delta_q = -s * inverse_masses[constraint.q] * grad_q;
        umath::Vec3 const delta_p1 = -s * inverse_masses[constraint.p1] * grad_p1;
        umath::Vec3 const delta_p2 = -s * inverse_masses[constraint.p2] * grad_p2;
        umath::Vec3 const delta_p3 = -s * inverse_masses[constraint.p3] * grad_p3;

        umath::Real const k_prime =
            1 - (1 / powf(1 - constraint.stiffness, 1 / static_cast<umath::Real>(solver_iterations)));

        p1 += delta_p1 * k_prime;
        p2 += delta_p2 * k_prime;
        p3 += delta_p3 * k_prime;
        q += delta_q * k_prime;
    }
}

}  // namespace simulation
}  // namespace ucloth