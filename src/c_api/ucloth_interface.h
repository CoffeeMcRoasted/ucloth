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

    ucloth_export PBD_system_hdl ucloth_create_simulation(void);
    ucloth_export void ucloth_delete_simulation(PBD_system_hdl pdb_system);
    ucloth_export void ucloth_simulate(PBD_system_hdl pdb, World_hdl world, float delta_time);

    ucloth_export void ucloth_add_positions(World_hdl handle, Ucloth_vector3f* positions, size_t size);
    // TODO: ADD CLOTH MASS TO INTERFACE!
    ucloth_export void ucloth_add_cloth(World_hdl handle,
                                        Ucloth_vector3f* positions,
                                        size_t pos_size,
                                        int* faces,
                                        size_t faces_size,
                                        float cloth_elasticity,
                                        float cloth_bending_stiffness);
}

#endif  //! UCLOTH_INTERFACE_H_