# Ryao
Ryao is a graphics simulator full of bugs XD. Currently it works fine on ubuntu, and will be tested on Windows in the future.

***Tips***: This branch is the `xmake` version of Ryao, I haven't found a good plan to handle all dependencies gracefully, you can find a runnable version in [here](https://github.com/LILKOTYO/Ryao/tree/cmake_version) (using cmake).
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