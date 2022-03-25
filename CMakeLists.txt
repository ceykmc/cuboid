cmake_minimum_required(VERSION 3.0.0)

project(cuboid)

if(WIN32)
    add_compile_options(/wd4819)
endif()

find_package(PCL 1.12.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
