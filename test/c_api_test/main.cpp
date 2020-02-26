#include <c_api/ucloth_interface.h>
#include <vector>

int main()
{
    World_hdl handle = ucloth_create_world();
    Ucloth_vector3f v1(1, 2, 3);
    Ucloth_vector3f v2(4, 5, 6);
    Ucloth_vector3f v3(7, 8, 9);
    std::vector<Ucloth_vector3f> v = {v1, v2, v3};
    ucloth_add_positions(handle, v.data(), v.size());
    ucloth_clear_world(handle);
    ucloth_delete_world(handle);
}