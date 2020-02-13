add_executable(test_simulation 
    collision_test.cpp
    constraint_test.cpp
    main.cpp
    pbdsystem_test.cpp
    world_test.cpp
)

target_link_libraries(test_simulation 
    PRIVATE gtest ucloth
)

add_test(NAME test_simulation COMMAND test_simulation)