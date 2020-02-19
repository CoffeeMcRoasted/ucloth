#include "ucloth_interface.h"

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
    auto* world_ptr = new ucloth::simulation::World();
    return reinterpret_cast<World_hdl>(world_ptr);
}

void ucloth_clear_world(World_hdl handle)
{
    auto* world_ptr = reinterpret_cast<ucloth::simulation::World*>(handle);
    world_ptr->clear();
}

void ucloth_delete_world(World_hdl handle)
{
    auto* world_ptr = reinterpret_cast<ucloth::simulation::World*>(handle);
    delete world_ptr;
}
