#ifndef UCLOTH_INTERFACE_H_
#define UCLOTH_INTERFACE_H_

#include "ucloth_export.h"
#include "ucloth_handles.h"
#include "ucloth_structures.h"

extern "C"
{
    ucloth_export World_hdl ucloth_create_world(void);
    ucloth_export void ucloth_delete_world(World_hdl world);
    ucloth_export void ucloth_add_acceleration(World_hdl handle, Ucloth_vector3f acceleration);
    ucloth_export void ucloth_clear_world(World_hdl handle);

    ucloth_export PBD_system_hdl ucloth_create_simulation(bool simulate_collisions);
    ucloth_export void ucloth_delete_simulation(PBD_system_hdl pdb_system);
    ucloth_export void ucloth_simulate(PBD_system_hdl pdb, World_hdl world, int solver_iterations, float delta_time);

    // ucloth_export void ucloth_add_positions(World_hdl handle, Ucloth_vector3f* positions, size_t size);
    ucloth_export Cloth_hdl ucloth_add_cloth(World_hdl handle,
                                             Ucloth_vector3f* positions,
                                             size_t pos_size,
                                             int* faces,
                                             size_t faces_size,
                                             float cloth_mass,
                                             float cloth_thickness,
                                             float cloth_elasticity,
                                             float cloth_bending_stiffness,
                                             float cloth_velocity_damping);

    ucloth_export void ucloth_retrieve_cloth_info(Cloth_hdl cloth_hdl,
                                                  World_hdl world_hdl,
                                                  Ucloth_vector3f*& positions,
                                                  size_t& pos_size,
                                                  int*& faces,
                                                  size_t& faces_size);

    ucloth_export Static_mesh_hdl ucloth_add_static_mesh(World_hdl handle,
                                                         Ucloth_vector3f* positions,
                                                         size_t pos_size,
                                                         int* faces,
                                                         size_t faces_size);

    ucloth_export void ucloth_retrieve_static_mesh_info(Static_mesh_hdl mesh_hdl,
                                                        World_hdl world_hdl,
                                                        Ucloth_vector3f*& positions,
                                                        size_t& pos_size,
                                                        int*& faces,
                                                        size_t& faces_size);

    ucloth_export Rigid_body_hdl
    ucloth_add_rigid_body(World_hdl handle, Ucloth_vector3f* positions, size_t pos_size, int* faces, size_t faces_size);

    ucloth_export void ucloth_retrieve_rigid_body_info(Rigid_body_hdl mesh_hdl,
                                                       World_hdl world_hdl,
                                                       Ucloth_vector3f*& positions,
                                                       size_t& pos_size,
                                                       int*& faces,
                                                       size_t& faces_size);

    ucloth_export void ucloth_attach_particle_to_position(World_hdl world_hdl,
                                                          Cloth_hdl cloth_hdl,
                                                          unsigned int index,
                                                          Ucloth_vector3f position);
}

#endif  //! UCLOTH_INTERFACE_H_