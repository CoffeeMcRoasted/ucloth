#include "ucloth_interface.h"

#include <algorithm>
#include <simulation/world.h>
#include <simulation/pbdsystem.h>
#include <umath/umath.h>

// Caveat about reinterpret_cast:
// https://en.cppreference.com/w/cpp/language/reinterpret_cast
// "Unlike static_cast, but like const_cast, the reinterpret_cast expression does not compile to any CPU instructions
// (except when converting between integers and pointers or on obscure architectures where pointer representation
// depends on its type). It is purely a compile-time directive which instructs the compiler to treat expression as if it
// had the type new_type."

World_hdl ucloth_create_world()
{
    auto* world = new ucloth::simulation::World();
    return reinterpret_cast<World_hdl>(world);
}

void ucloth_clear_world(World_hdl handle)
{
    auto* world = reinterpret_cast<ucloth::simulation::World*>(handle);
    world->clear();
}

void ucloth_delete_world(World_hdl handle)
{
    auto* world = reinterpret_cast<ucloth::simulation::World*>(handle);
    delete world;
}

void ucloth_add_acceleration(World_hdl handle, Ucloth_vector3f acceleration)
{
    auto* world = reinterpret_cast<ucloth::simulation::World*>(handle);
    world->add_acceleration({acceleration.x_, acceleration.y_, acceleration.z_});
}

PBD_system_hdl ucloth_create_simulation(void)
{
    auto* pbd_system = new ucloth::simulation::PBD_system();
    return reinterpret_cast<PBD_system_hdl>(pbd_system);
}

void ucloth_delete_simulation(PBD_system_hdl handle)
{
    auto* pbd_system = reinterpret_cast<ucloth::simulation::PBD_system*>(handle);
    delete pbd_system;
}

void ucloth_simulate(PBD_system_hdl pbd_hdl, World_hdl world_hdl, float delta_time)
{
    auto* pbd_system = reinterpret_cast<ucloth::simulation::PBD_system*>(pbd_hdl);
    auto* world = reinterpret_cast<ucloth::simulation::World*>(world_hdl);
    pbd_system->simulate(delta_time, *world);
}

void ucloth_add_positions(World_hdl handle, Ucloth_vector3f* positions, size_t size)
{
    auto* world = reinterpret_cast<ucloth::simulation::World*>(handle);
    auto* pos = reinterpret_cast<ucloth::umath::Vec3*>(positions);
    std::vector<ucloth::umath::Vec3> pos_vector{pos, pos + size};
    std::copy(
        pos_vector.begin(), pos_vector.end(), std::back_inserter(world->positions));  //< Appends positions at the end
}

void ucloth_add_cloth(World_hdl handle,
                      Ucloth_vector3f* positions,
                      size_t pos_size,
                      int* faces,
                      size_t faces_size,
                      float cloth_elasticity,
                      float cloth_bending_stiffness)
{
    auto* world = reinterpret_cast<ucloth::simulation::World*>(handle);
    auto* pos = reinterpret_cast<ucloth::umath::Vec3*>(positions);
    std::vector<ucloth::umath::Vec3> pos_vector{pos, pos + pos_size};

    ucloth::simulation::Mesh mesh;
    mesh.reserve(faces_size / 3);
    std::vector<int> faces_vector{faces, faces + faces_size};
    for (int f = 0; f < faces_size; f += 3)
    {
        ucloth::simulation::Particle const p1 = faces[f + 0];
        ucloth::simulation::Particle const p2 = faces[f + 1];
        ucloth::simulation::Particle const p3 = faces[f + 2];
        mesh.emplace_back(ucloth::simulation::Face{p1, p2, p3});
    }

    world->add_cloth(pos_vector, mesh, cloth_elasticity, cloth_bending_stiffness);
}