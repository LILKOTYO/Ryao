# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/liao/selfcppdev/Ryao/_deps/tetgen-src"
  "/home/liao/selfcppdev/Ryao/_deps/tetgen-build"
  "/home/liao/selfcppdev/Ryao/_deps/tetgen-subbuild/tetgen-populate-prefix"
  "/home/liao/selfcppdev/Ryao/_deps/tetgen-subbuild/tetgen-populate-prefix/tmp"
  "/home/liao/selfcppdev/Ryao/_deps/tetgen-subbuild/tetgen-populate-prefix/src/tetgen-populate-stamp"
  "/home/liao/selfcppdev/Ryao/_deps/tetgen-subbuild/tetgen-populate-prefix/src"
  "/home/liao/selfcppdev/Ryao/_deps/tetgen-subbuild/tetgen-populate-prefix/src/tetgen-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/liao/selfcppdev/Ryao/_deps/tetgen-subbuild/tetgen-populate-prefix/src/tetgen-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/liao/selfcppdev/Ryao/_deps/tetgen-subbuild/tetgen-populate-prefix/src/tetgen-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
