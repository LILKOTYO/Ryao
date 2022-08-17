# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/liao/selfcppdev/Ryao/_deps/glfw-src"
  "/home/liao/selfcppdev/Ryao/_deps/glfw-build"
  "/home/liao/selfcppdev/Ryao/_deps/glfw-subbuild/glfw-populate-prefix"
  "/home/liao/selfcppdev/Ryao/_deps/glfw-subbuild/glfw-populate-prefix/tmp"
  "/home/liao/selfcppdev/Ryao/_deps/glfw-subbuild/glfw-populate-prefix/src/glfw-populate-stamp"
  "/home/liao/selfcppdev/Ryao/_deps/glfw-subbuild/glfw-populate-prefix/src"
  "/home/liao/selfcppdev/Ryao/_deps/glfw-subbuild/glfw-populate-prefix/src/glfw-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/liao/selfcppdev/Ryao/_deps/glfw-subbuild/glfw-populate-prefix/src/glfw-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/liao/selfcppdev/Ryao/_deps/glfw-subbuild/glfw-populate-prefix/src/glfw-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
