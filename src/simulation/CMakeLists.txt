add_library(ucloth STATIC
    world.h
    world.cpp
    constraint.h
    constraint.cpp
    uclothcommon.h
)

target_link_libraries(ucloth
    PUBLIC umath
)

target_include_directories(ucloth PUBLIC ../)