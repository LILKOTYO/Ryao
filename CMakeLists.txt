cmake_minimum_required(VERSION 3.20)
project(Ryao)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-DPROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")

# set(CMAKE_TOOLCHAIN_FILE D:/vcpkg/scripts/buildsystems/vcpkg.cmake)

# engine ilbrary
add_subdirectory(core/Platform)
add_subdirectory(core/Geometry)
add_subdirectory(core/Hyperelastic)
add_subdirectory(core/Damping)
add_subdirectory(core/Solver)
add_subdirectory(core/PBDConstraint)

# demo project
add_subdirectory(demo)
# add_subdirectory(test)
