cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(scratch)

add_executable(scratch
    src/scratch.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(scratch
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(scratch PRIVATE -Wall -Wextra)
else()
    add_compile_options(scratch PRIVATE -W2)
endif()