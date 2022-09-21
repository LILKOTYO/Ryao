# Ryao
Ryao is a graphics simulator full of bugs XD. Currently it works fine on ubuntu, and will be tested on Windows in the future.

This branch is the `cmake` version of Ryao, and I am also trying to develop the `xmake` version, but it may take some time to learn and debug
## Dependencies
- libigl
- **spdlog**
- **spectra**(TODO)
- Eigen
- imgui
- glad
- glfw

On Ubuntu, you need to install the following packages:
```shell
sudo apt-get install build-essential libx11-dev mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev libxrandr-dev libxi-dev libxmu-dev libblas-dev libxinerama-dev libxcursor-dev
```

Except for spdlog, all other dependencies can be automatically downloaded by cmake.

To install spdlog:
```shell
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
```shell
git clone git@github.com:LILKOTYO/Ryao.git
git checkout cmake_version
mkdir build
cd build
cmake ..
make -j8
```
## ChangeLog
### 2022.08.17
cmake bug:
```
if given arguments: 
    "OVERRIDE_FIND_PACKAGE" "IN_LIST" "ARGN" "AND" "FIND_PACKAGE_ARGS" "IN_LIST" "ARGN"
Unknown arguments specified
```
The cmake version declared in the `CMakeLists.txt` is too old, change it to your cmake's version.
### 2022.08.18
find a bug(maybe not a bug): Sometimes this error may occur:
```
segmentation fault (core dumped)
```
The reason may be that the Logger is not initialized in the main function, while in **Ryao**, spdlog is used by default (eg in GUI).
So make sure the following code exists in `main.cpp`:
```c++
#include <Logger.h>
...
using namespace Ryao;
int main() {
    ...
    Logger::Init();
    ...
}
```

Try to implement Earth-Moon Simulation and succeed.

Get some model files from libigl XD.

TODO: 
- Combine `RigidObject` class and `BaseObject`.
- Try to find way to implement softbody (particle).