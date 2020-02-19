#ifndef UCLOTH_INTERFACE_H_
#define UCLOTH_INTERFACE_H_

#include "ucloth_export.h"
#include "ucloth_handles.h"
#include "ucloth_structures.h"

extern "C"
{
    ucloth_export World_hdl ucloth_create_world(void);
    ucloth_export void ucloth_delete_world(World_hdl world);

    ucloth_export PBD_system_hdl ucloth_create_simulation(void);
    ucloth_export void ucloth_delete_simulation(PBD_system_hdl pdb_system);

    ucloth_export void ucloth_add_acceleration(World_hdl handle);
    ucloth_export void ucloth_simulate(PBD_system_hdl pdb, World_hdl world);

    ucloth_export void ucloth_clear_world(World_hdl handle);
}

#endif  //! UCLOTH_INTERFACE_H_