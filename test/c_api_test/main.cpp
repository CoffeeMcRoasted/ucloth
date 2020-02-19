#include <c_api/ucloth_interface.h>
#include <iostream>

int main()
{
    World_hdl handle = ucloth_create_world();
    ucloth_clear_world(handle);
    ucloth_delete_world(handle);
}