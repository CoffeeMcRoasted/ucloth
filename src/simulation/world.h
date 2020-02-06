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
    std::vector<umath::Position> positions;
    std::vector<umath::Vec_3> velocities;
    std::vector<umath::Real> inverse_particle_masses;
    std::vector<Distance_constraint> distance_constraints;
    std::vector<Bending_constraint> bending_constraints;
    std::vector<Force> forces;
};
}  // namespace simulation
}  // namespace ucloth
#endif  //! UCLOTH_WORLD_H_