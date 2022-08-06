add_rules("mode.debug", "mode.release")

add_requires("spdlog", "glfw", "glad", "stb", "glm")
add_requires("libigl", {configs = {imgui = true}})

set_languages("cxx17")

set_project("Ryao")

target("libiglxmake")
    set_kind("binary")
    add_files("src/*.cpp")
    add_files("core/platform/*.cpp")
    
    add_includedirs("core/platform", "external/imgui", "external/libigl-imgui")
    -- add_headerfiles("core/platform/*.h", "external/imgui/*.h", "external/libigl-imgui/*.h")
    
    add_packages("spdlog", "glfw", "glad", "stb", "glm", "libigl")

    set_symbols("debug")
    if is_os("linux") then
        add_syslinks("OpenGL")
        add_syslinks("pthread")
    end

    if is_os("windows") then
        add_syslinks(opengl32)
    end

    set_rundir("$(projectdir)")
    set_targetdir("./bin/$(os)_$(arch)_$(mode)")
target_end()