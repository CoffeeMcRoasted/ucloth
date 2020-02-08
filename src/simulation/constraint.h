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
    Particle p_1;
    Particle p_2;
    umath::Real distance;
    umath::Real stiffness;
};

struct Collision_constraint
{
    Particle q;
    Particle p_1;
    Particle p_2;
    Particle p_3;
    umath::Real thickness;
    umath::Real stiffness;
};

struct Bending_constraint
{
    Particle p_1;
    Particle p_2;
    Particle p_3;
    Particle p_4;
    umath::Real dihedral_angle;
    umath::Real stiffness;
};

}  // namespace simulation
}  // namespace ucloth
#endif  //! UCLOTH_CONSTRAINT_H_