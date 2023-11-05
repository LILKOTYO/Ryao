cmake_minimum_required(VERSION 3.20)
project(mainScene)

set(CMAKE_CXX_FLAGS "-Wall")

set(CMAKE_BUILD_TYPE Debug)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# file(GLOB LIBFILES ${PROJECT_SOURCE_DIR}/../core/Platform/*.*)
# source_group("Library Files" FILES ${LIBFILES})

# Add default project files
 include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../core/Scene)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../core/Hyperelastic)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../core/Geometry)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../external/spectra/include)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../core/Damping)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../core/Timestepper)

# Add your project files
file(GLOB SRCFILES *.cpp)
file(GLOB HFILES *.h ../core/Scene/*.h)

add_executable(${PROJECT_NAME} ${SRCFILES} ${HFILES} )

add_custom_command(
        TARGET ${PROJECT_NAME}  POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${PROJECT_SOURCE_DIR}/../shaders ${PROJECT_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE}/shaders
    )

add_custom_command(
        TARGET ${PROJECT_NAME}  POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${PROJECT_SOURCE_DIR}/../shaders ${PROJECT_BINARY_DIR}/bin/shaders
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    Ryao::Platform
    Ryao::Geometry
    Ryao::Hyperelastic
    Ryao::Damping
    Ryao::Solver
    Ryao::PBDConstraint
)

find_package(spdlog CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog spdlog::spdlog_header_only)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)

find_package(glfw3 CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE glfw)

find_package(glad CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE glad::glad)

find_package(glm CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE glm::glm)

find_package(OpenMP REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
