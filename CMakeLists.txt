cmake_minimum_required(VERSION 3.15)

project(ucloth
        LANGUAGES CXX
        VERSION 1.0.0.0
)

enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(thirdparty)
add_subdirectory(src)
add_subdirectory(test)
