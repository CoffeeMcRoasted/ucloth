add_executable(c_api_test
    main.cpp
)

target_link_libraries(c_api_test 
    PRIVATE ucloth_c_api gtest
)

add_test(NAME test_simulation COMMAND test_simulation)