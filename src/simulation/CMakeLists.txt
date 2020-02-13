add_library(ucloth STATIC
    world.h
    world.cpp
    constraint.h
    constraint.cpp
    uclothcommon.h
    pbdsystem.h
    pbdsystem.cpp
    collision.h
    collision.cpp
)

target_link_libraries(ucloth
    PUBLIC umath
)

target_include_directories(ucloth PUBLIC ../)