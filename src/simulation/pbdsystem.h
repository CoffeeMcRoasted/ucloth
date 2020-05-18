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
    explicit PBD_system(bool simulate_colisions) noexcept;
    ~PBD_system() = default;

    // Unmovable and uncopyable
    PBD_system& operator=(PBD_system const&) = delete;
    PBD_system(PBD_system const&) = delete;
    PBD_system& operator=(PBD_system&&) = delete;
    PBD_system(PBD_system&&) = delete;

    void simulate(umath::Real const delta_time, size_t const solver_iterations, World& world);

    static void apply_external_accelerations(std::vector<umath::Vec3> const& accelerations,
                                             umath::Real const delta_time,
                                             std::vector<umath::Vec3>& velocities);

    static void damp_velocity(std::vector<simulation::Mesh> const& meshes,
                              std::vector<umath::Position> const& positions,
                              std::vector<umath::Real> const& inverse_masses,
                              std::vector<umath::Vec3>& velocities);

    static std::vector<simulation::Collision_constraint> generate_collision_constraints(
        std::vector<umath::Position> const& positions,
        std::vector<umath::Position> const& p_estimations,
        std::vector<simulation::Mesh> const& meshes);

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

    static void solve_attachments(std::vector<Attachment> const& attachments,
                                  std::vector<umath::Position>& position_estimates);

    // static void velocity_update();

private:
    std::vector<umath::Position> position_estimates;
    bool simulate_collisions;
};
}  // namespace simulation
}  // namespace ucloth
#endif  //! UCLOTH_PBDSYSTEM_H_