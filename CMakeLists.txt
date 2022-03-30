cmake_minimum_required(VERSION 3.0.0)

project(cuboid)

set(EXE_NAME cuboid)

if(WIN32)
    add_compile_options(/wd4819)
    add_compile_options(/D_CRT_SECURE_NO_WARNINGS)
endif()

find_package(PCL 1.12.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${EXE_NAME} main.cpp utility.hpp utility.cpp)

target_link_libraries(${EXE_NAME} ${PCL_LIBRARIES})
