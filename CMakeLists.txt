cmake_minimum_required(VERSION 3.20)
project(Ryao)

# engine ilbrary
add_subdirectory(core/Platform)
add_subdirectory(core/Geometry)

# demo project
add_subdirectory(src)
# add_subdirectory(test)
