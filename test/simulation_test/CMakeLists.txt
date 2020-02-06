add_executable(test_simulation 
    main.cpp
    world_test.cpp
)

target_link_libraries(test_simulation 
    PRIVATE gtest ucloth
)

add_test(NAME test_simulation COMMAND simulation)