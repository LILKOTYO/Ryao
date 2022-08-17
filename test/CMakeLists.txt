cmake_minimum_required(VERSION 3.1)
project(test)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)
set(CMAKE_CXX_FLAGS "-Wall")

# libigl
option(LIBIGL_USE_STATIC_LIBRARY "Use libigl as static library" OFF)
option(LIBIGL_WITH_ANTTWEAKBAR       "Use AntTweakBar"    OFF)
option(LIBIGL_WITH_CGAL              "Use CGAL"           OFF)
option(LIBIGL_WITH_COMISO            "Use CoMiso"         OFF)
option(LIBIGL_WITH_CORK              "Use Cork"           OFF)
option(LIBIGL_WITH_EMBREE            "Use Embree"         OFF)
option(LIBIGL_WITH_LIM               "Use LIM"            OFF)
option(LIBIGL_WITH_MATLAB            "Use Matlab"         OFF)
option(LIBIGL_WITH_MOSEK             "Use MOSEK"          OFF)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          ON)
option(LIBIGL_WITH_PNG               "Use PNG"            OFF)
option(LIBIGL_WITH_PYTHON            "Use Python"         OFF)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"         ON)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"       OFF)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_XML               "Use XML"            OFF)

if (NOT LIBIGL_FOUND)
    find_package(LIBIGL REQUIRED QUIET)
endif()

# Add default project files
file(GLOB LIBFILES ${PROJECT_SOURCE_DIR}/../core/platform/*.*)
source_group("Library Files" FILES ${LIBFILES})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../core/platform)

# Add your project files
file(GLOB SRCFILES *.cpp)
file(GLOB HFILES *.h)

add_definitions(-DIGL_VIEWER_VIEWER_QUIET)
add_executable(${PROJECT_NAME} ${SRCFILES} ${HFILES} ${LIBFILES} )
target_link_libraries(${PROJECT_NAME} igl::core igl::opengl_glfw igl::opengl_glfw_imgui)