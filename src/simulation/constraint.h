#ifndef UCLOTH_CONSTRAINT_H_
#define UCLOTH_CONSTRAINT_H_

#include <simulation/uclothcommon.h>
#include <tuple>
#include <umath/umath.h>

namespace ucloth
{
namespace simulation
{
struct Distance_constraint
{
    Particle p1;
    Particle p2;
    umath::Real distance;
    umath::Real stiffness;
};

struct Collision_constraint
{
    Particle q;
    Particle p1;
    Particle p2;
    Particle p3;
    umath::Real thickness;
    umath::Real stiffness;
};

struct Bending_constraint
{
    Particle p1;
    Particle p2;
    Particle p3;
    Particle p4;
    umath::Real dihedral_angle;
    umath::Real stiffness;
};

void project_distance_constraints(std::vector<Distance_constraint> const& constraints,
                                  std::vector<umath::Real> const& inverse_masses,
                                  unsigned int const solver_iterations,
                                  std::vector<umath::Position>& positions);
void project_bending_constraints(std::vector<Bending_constraint> const& constraints,
                                 std::vector<umath::Real> const& inverse_masses,
                                 unsigned int const solver_iterations,
                                 std::vector<umath::Position>& positions);
void project_collision_constraints(std::vector<Bending_constraint> const& constraints,
                                   std::vector<umath::Real> const& inverse_masses,
                                   unsigned int const solver_iterations,
                                   std::vector<umath::Position>& positions);

}  // namespace simulation
}  // namespace ucloth
#endif  //! UCLOTH_CONSTRAINT_H_