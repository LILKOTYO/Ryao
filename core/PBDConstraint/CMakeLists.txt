cmake_minimum_required(VERSION 3.20)

project(RyaoPBDConstraint)

# add the sources
file(GLOB SRCFILES src/*.cpp)
file(GLOB HFILES include/*.h)

add_library(${PROJECT_NAME} ${SRCFILES} ${HFILES})
add_library(Ryao::PBDConstraint ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
        PUBLIC
        ${PROJECT_SOURCE_DIR}/include/
        ${PROJECT_SOURCE_DIR}/../
        )

# add package
find_package(spdlog CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog spdlog::spdlog_header_only)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)

find_package(OpenMP REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)