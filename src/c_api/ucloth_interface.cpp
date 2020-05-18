#include "ucloth_interface.h"

#include <algorithm>
#include <simulation/uclothcommon.h>
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

PBD_system_hdl ucloth_create_simulation(bool simulate_collisions)
{
    auto* pbd_system = new ucloth::simulation::PBD_system(simulate_collisions);
    return reinterpret_cast<PBD_system_hdl>(pbd_system);
}

void ucloth_delete_simulation(PBD_system_hdl handle)
{
    auto* pbd_system = reinterpret_cast<ucloth::simulation::PBD_system*>(handle);
    delete pbd_system;
}

void ucloth_simulate(PBD_system_hdl pbd_hdl, World_hdl world_hdl, int solver_iterations, float delta_time)
{
    auto* pbd_system = reinterpret_cast<ucloth::simulation::PBD_system*>(pbd_hdl);
    auto* world = reinterpret_cast<ucloth::simulation::World*>(world_hdl);
    pbd_system->simulate(delta_time, solver_iterations, *world);
}

void ucloth_add_positions(World_hdl handle, Ucloth_vector3f* positions, size_t size)
{
    auto* world = reinterpret_cast<ucloth::simulation::World*>(handle);
    auto* pos = reinterpret_cast<ucloth::umath::Vec3*>(positions);
    std::vector<ucloth::umath::Vec3> pos_vector{pos, pos + size};
    world->positions.insert(world->positions.end(), pos_vector.begin(), pos_vector.end());
}

Cloth_hdl ucloth_add_cloth(World_hdl handle,
                           Ucloth_vector3f* positions,
                           size_t pos_size,
                           int* faces,
                           size_t faces_size,
                           float cloth_mass,
                           float cloth_thickness,
                           float cloth_elasticity,
                           float cloth_bending_stiffness,
                           float cloth_velocity_damping)
{
    auto* world = reinterpret_cast<ucloth::simulation::World*>(handle);
    auto* pos = reinterpret_cast<ucloth::umath::Vec3*>(positions);
    std::vector<ucloth::umath::Vec3> pos_vector{pos, pos + pos_size};

    ucloth::simulation::Mesh mesh;
    mesh.faces.reserve(faces_size / 3);
    mesh.begin = world->positions.size();
    mesh.end = mesh.begin + pos_size;
    mesh.cloth_thickness = cloth_thickness;
    mesh.k_velocity = cloth_velocity_damping;
    mesh.type = ucloth::simulation::Mesh_type::cloth;

    std::vector<int> faces_vector{faces, faces + faces_size};
    for (int f = 0; f < faces_size; f += 3)
    {
        ucloth::simulation::Particle const p1 = faces[f + 0];
        ucloth::simulation::Particle const p2 = faces[f + 1];
        ucloth::simulation::Particle const p3 = faces[f + 2];
        mesh.faces.emplace_back(ucloth::simulation::Face{p1, p2, p3});
    }

    ucloth::simulation::Mesh const* cloth =
        &world->add_cloth(pos_vector, mesh, cloth_mass, cloth_elasticity, cloth_bending_stiffness);
    return reinterpret_cast<Cloth_hdl>(cloth);
}

void ucloth_retrieve_cloth_info(Cloth_hdl cloth_hdl,
                                World_hdl world_hdl,
                                Ucloth_vector3f*& positions,
                                size_t& pos_size,
                                int*& faces,
                                size_t& faces_size)
{
    auto* world = reinterpret_cast<ucloth::simulation::World*>(world_hdl);
    auto const* mesh = reinterpret_cast<const ucloth::simulation::Mesh*>(cloth_hdl);
    positions = reinterpret_cast<Ucloth_vector3f*>(&world->positions[mesh->begin]);
    pos_size = mesh->end - mesh->begin;
    faces = const_cast<int*>(reinterpret_cast<const int*>(mesh->faces.data()));
    faces_size = mesh->faces.size() * 3;
}

Static_mesh_hdl ucloth_add_static_mesh(World_hdl handle,
                                       Ucloth_vector3f* positions,
                                       size_t pos_size,
                                       int* faces,
                                       size_t faces_size)
{
    auto* world = reinterpret_cast<ucloth::simulation::World*>(handle);
    auto* pos = reinterpret_cast<ucloth::umath::Vec3*>(positions);
    std::vector<ucloth::umath::Vec3> pos_vector{pos, pos + pos_size};

    ucloth::simulation::Mesh mesh;
    mesh.faces.reserve(faces_size / 3);
    mesh.begin = world->positions.size();
    mesh.end = mesh.begin + pos_size;
    mesh.k_velocity = 1;
    mesh.type = ucloth::simulation::Mesh_type::static_mesh;

    std::vector<int> faces_vector{faces, faces + faces_size};
    for (int f = 0; f < faces_size; f += 3)
    {
        ucloth::simulation::Particle const p1 = faces[f + 0];
        ucloth::simulation::Particle const p2 = faces[f + 1];
        ucloth::simulation::Particle const p3 = faces[f + 2];
        mesh.faces.emplace_back(ucloth::simulation::Face{p1, p2, p3});
    }

    ucloth::simulation::Mesh const* cloth = &world->add_static_mesh(pos_vector, mesh);
    return reinterpret_cast<Static_mesh_hdl>(cloth);
}

void ucloth_retrieve_static_mesh_info(Static_mesh_hdl mesh_hdl,
                                      World_hdl world_hdl,
                                      Ucloth_vector3f*& positions,
                                      size_t& pos_size,
                                      int*& faces,
                                      size_t& faces_size)
{
    auto* world = reinterpret_cast<ucloth::simulation::World*>(world_hdl);
    auto const* mesh = reinterpret_cast<ucloth::simulation::Mesh const*>(mesh_hdl);
    positions = reinterpret_cast<Ucloth_vector3f*>(&world->positions[mesh->begin]);
    pos_size = mesh->end - mesh->begin;
    faces = const_cast<int*>(reinterpret_cast<const int*>(mesh->faces.data()));
    faces_size = mesh->faces.size() * 3;
}

Rigid_body_hdl ucloth_add_rigid_body(World_hdl handle,
                                     Ucloth_vector3f* positions,
                                     size_t pos_size,
                                     int* faces,
                                     size_t faces_size,
                                     float body_mass)
{
    auto* world = reinterpret_cast<ucloth::simulation::World*>(handle);
    auto* pos = reinterpret_cast<ucloth::umath::Vec3*>(positions);
    std::vector<ucloth::umath::Vec3> pos_vector{pos, pos + pos_size};

    ucloth::simulation::Mesh mesh;
    mesh.faces.reserve(faces_size / 3);
    mesh.begin = world->positions.size();
    mesh.end = mesh.begin + pos_size;
    mesh.k_velocity = 1;
    mesh.type = ucloth::simulation::Mesh_type::rigid_body;

    std::vector<int> faces_vector{faces, faces + faces_size};
    for (int f = 0; f < faces_size; f += 3)
    {
        ucloth::simulation::Particle const p1 = faces[f + 0];
        ucloth::simulation::Particle const p2 = faces[f + 1];
        ucloth::simulation::Particle const p3 = faces[f + 2];
        mesh.faces.emplace_back(ucloth::simulation::Face{p1, p2, p3});
    }

    ucloth::simulation::Mesh const* cloth = &world->add_rigid_body(pos_vector, mesh, body_mass);
    return reinterpret_cast<Rigid_body_hdl>(cloth);
}

void ucloth_retrieve_rigid_body_info(Rigid_body_hdl mesh_hdl,
                                     World_hdl world_hdl,
                                     Ucloth_vector3f*& positions,
                                     size_t& pos_size,
                                     int*& faces,
                                     size_t& faces_size)
{
    auto* world = reinterpret_cast<ucloth::simulation::World*>(world_hdl);
    auto const* mesh = reinterpret_cast<const ucloth::simulation::Mesh*>(mesh_hdl);
    positions = reinterpret_cast<Ucloth_vector3f*>(&world->positions[mesh->begin]);
    pos_size = mesh->end - mesh->begin;
    faces = const_cast<int*>(reinterpret_cast<const int*>(mesh->faces.data()));
    faces_size = mesh->faces.size() * 3;
}

ucloth_export void ucloth_attach_particle_to_position(World_hdl world_hdl,
                                                      Cloth_hdl cloth_hdl,
                                                      unsigned int index,
                                                      Ucloth_vector3f position)
{
    auto* world = reinterpret_cast<ucloth::simulation::World*>(world_hdl);
    auto const* cloth = reinterpret_cast<ucloth::simulation::Mesh const*>(cloth_hdl);
    ucloth::umath::Position pos = {position.x_, position.y_, position.z_};
    world->attach_particle(*cloth, index, pos);
}
