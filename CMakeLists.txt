cmake_minimum_required(VERSION 3.20)
project(Ryao)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# engine ilbrary
add_subdirectory(core/Platform)
add_subdirectory(core/Geometry)

# demo project
add_subdirectory(demo)
# add_subdirectory(test)
