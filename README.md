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
vcpkg install imgui:x64-windows
```

If IDE can not find file `imgui_impl_glfw.h` and `imgui_impl_opengl3.h`, try:
```shell
vcpkg install imgui[opengl3-bingding]:x64-windows --recurse
vcpkg install imgui[glfw-bingding]:x64-windows --recurse
```
If you haven't installed vcpkg, or don't know how to use vcpkg-managed libraries in Visual Studio's CMake projects, be sure to check the official vcpkg documentation.
