# Ryao
Ryao is a graphics simulator. This version runs smoothly on windows.

## Dependencies
- spdlog
- Eigen3
- glad
- glfw
- glm

I use [`vcpkg`](https://github.com/microsoft/vcpkg) to manage packages.
You can use the following commands to complete the package installation (take 64-bit windows system as an example):
```shell
vcpkg install spdlog:x64-windows
vcpkg install eigen3:x64-windows
vcpkg install glad:x64-windows
vcpkg install glfw3:x64-windows
vcpkg install glm:x64-windows
```
If you haven't installed vcpkg, or don't know how to use vcpkg-managed libraries in Visual Studio's CMake projects, be sure to check the official vcpkg documentation.

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