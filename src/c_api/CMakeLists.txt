add_library(ucloth_c_api SHARED
    ucloth_export.h
    ucloth_handles.h
    ucloth_interface.h
    ucloth_interface.cpp
    ucloth_structures.h
)

# set_target_properties(ucloth_c_api PROPERTIES
# TODO: Set properties to export dll and lib
# )

target_link_libraries(ucloth_c_api
    PRIVATE ucloth
)

target_include_directories(ucloth_c_api PUBLIC ../)