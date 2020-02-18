#ifndef UCLOTH_HANDLES_H_
#define UCLOTH_HANDLES_H_

#define DECLARE_HANDLE(name) \
    typedef struct name##__  \
    {                        \
        int unused;          \
    } * name

DECLARE_HANDLE(PBD_system_hdl);
DECLARE_HANDLE(World_hdl);

#endif  //! UCLOTH_HANDLES_H_