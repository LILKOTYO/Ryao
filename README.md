# Ryao
Ryao is a graphics simulator full of bugs XD. Currently it works fine on ubuntu, and will be tested on Windows in the future.

This branch is the `cmake` version of Ryao, and I am also trying to develop the `xmake` version, but it may take some time to learn and debug
## Dependencies
- libigl
- **spdlog**
- Eigen
- imgui
- glad
- glfw

Except for spdlog, all other dependencies can be automatically downloaded by cmake.

To install spdlog:
```
$ git clone https://github.com/gabime/spdlog.git
$ cd spdlog && mkdir build && cd build
$ cmake .. && make -j
```

For Windows users, you can install spdlog sing vcpkg:
```
vcpkg install spdlog
```
Or you can also use head-only version, however it is not recommended as it is much slower while compiling.
## Quick Start
```
git clone git@github.com:LILKOTYO/Ryao.git
git checkout cmake_version
mkdir build
cd build
cmake ..
make -j8
```