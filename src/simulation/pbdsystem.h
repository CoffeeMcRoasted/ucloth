#ifndef UCLOTH_PBDSYSTEM_H_
#define UCLOTH_PBDSYSTEM_H_

#include <simulation/uclothcommon.h>
#include <simulation/world.h>
#include <umath/umath.h>
#include <vector>

namespace ucloth
{
namespace simulation
{
class PBD_system
{
public:
    void simulate(umath::Real const delta_time, World& world);

    static void apply_external_accelerations(std::vector<umath::Vec3> const& accelerations,
                                             umath::Real const delta_time,
                                             std::vector<umath::Vec3>& velocities);

    static void damp_velocity(umath::Real const k_damping,
                              std::vector<umath::Real> const& inverse_masses,
                              std::vector<umath::Position> const& positions,
                              std::vector<umath::Vec3>& velocities);

    static std::vector<simulation::Collision_constraint> generate_collision_constraints(
        std::vector<umath::Position> const& positions,
        std::vector<umath::Position> const& p_estimations,
        std::vector<simulation::Mesh> const& meshes,
        umath::Real const cloth_thickness);

    static void calculate_position_estimates(std::vector<umath::Position> const& positions,
                                             std::vector<umath::Vec3> const& velocities,
                                             umath::Real const delta_time,
                                             std::vector<umath::Position>& position_estimates);

    static void project_constraints(std::vector<simulation::Collision_constraint> const& collision_constraints,
                                    std::vector<simulation::Distance_constraint> const& distance_constraints,
                                    std::vector<simulation::Bending_constraint> const& bending_constraints,
                                    std::vector<umath::Real> const& inverse_masses,
                                    unsigned int const solver_iterations,
                                    std::vector<umath::Position>& position_estimates);

    // This is unnecesary if there are no static elements in the scene.
    // static void velocity_update();


private:
    umath::Real const velocity_damping = 0.9f;
    umath::Real const cloth_thickness = 0.01f;
    size_t const solver_iterations = 100;
    std::vector<umath::Position> position_estimates;
};
}  // namespace simulation
}  // namespace ucloth
#endif  //! UCLOTH_PBDSYSTEM_H_