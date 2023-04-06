project(RyaoPlatform)

# add the sources
file(GLOB SRCFILES src/*.cpp)
file(GLOB HFILES include/*.h)

add_library(${PROJECT_NAME} ${SRCFILES})
add_library(Ryao::Platform ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME} 
	PUBLIC ${PROJECT_SOURCE_DIR}/include
)

# add package
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