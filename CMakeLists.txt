cmake_minimum_required(VERSION 3.24)
project(CA)

# list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# option(LIBIGL_USE_STATIC_LIBRARY        "Use libigl as static library" OFF)
# option(LIBIGL_COPYLEFT_CGAL             "Use CGAL"           OFF)
# option(LIBIGL_COPYLEFT_COMISO           "Use CoMiso"         OFF)
# option(LIBIGL_EMBREE                    "Use Embree"         OFF)
# option(LIBIGL_RESTRICTED_MATLAB         "Use Matlab"         OFF)
# option(LIBIGL_RESTRICTED_MOSEK          "Use MOSEK"          OFF)
# option(LIBIGL_OPENGL                    "Use OpenGL"         ON)
# option(LIBIGL_GLFW                      "Use GLFW"           ON)
# option(LIBIGL_IMGUI                     "Use ImGui"          ON)
# option(LIBIGL_PNG                       "Use PNG"            OFF)
# option(LIBIGL_COPYLEFT_TETGEN           "Use Tetgen"         ON)
# option(LIBIGL_RESTRICTED_TRIANGLE       "Use Triangle"       OFF)
# option(LIBIGL_WITH_VIEWER               "Use OpenGL viewer"  ON)
# option(LIBIGL_WITH_XML                  "Use XML"            OFF)
# include(libigl)

# if(NOT TARGET spdlog)
#     # Stand-alone build
#     find_package(spdlog REQUIRED)
# endif()

add_subdirectory(src)
# add_subdirectory(test)
