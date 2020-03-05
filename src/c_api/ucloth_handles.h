#ifndef UCLOTH_HANDLES_H_
#define UCLOTH_HANDLES_H_

#define DECLARE_HANDLE(name) \
    typedef struct name##__  \
    {                        \
        int unused;          \
    } * name

#define DECLARE_CONST_HANDLE(name) \
    typedef struct name##__        \
    {                              \
        int unused;                \
    } const* name

DECLARE_HANDLE(PBD_system_hdl);
DECLARE_HANDLE(World_hdl);
DECLARE_CONST_HANDLE(Rigid_body_hdl);
DECLARE_CONST_HANDLE(Cloth_hdl);
DECLARE_CONST_HANDLE(Static_mesh_hdl);

#endif  //! UCLOTH_HANDLES_H_