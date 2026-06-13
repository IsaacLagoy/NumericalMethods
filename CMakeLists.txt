cmake_minimum_required(VERSION 3.20)
project(ik LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# -----------------------------------------------------------------------------
# Sources — add your .cpp files here
# -----------------------------------------------------------------------------
add_executable(${PROJECT_NAME}
    src/render/matrix_stack.cpp
    src/render/camera.cpp
    src/render/renderer.cpp
    src/render/shader_server.cpp
    src/render/GLSL.cpp
    src/render/obj_server.cpp

    src/skeleton/bone.cpp
    src/skeleton/robot.cpp

    src/solver/solver.cpp
    src/solver/gn_solver.cpp
    src/solver/lm_solver.cpp
    src/solver/solver_manager.cpp

    src/main.cpp
)

# -----------------------------------------------------------------------------
# OpenGL — use the system installation
# -----------------------------------------------------------------------------
find_package(OpenGL REQUIRED)

# -----------------------------------------------------------------------------
# GLFW — use the system installation
# -----------------------------------------------------------------------------
find_package(glfw3 REQUIRED)

# -----------------------------------------------------------------------------
# GLAD — expects glad/ in your source tree (generated from glad.dav1d.de)
# Place glad.c and include/glad/glad.h in a glad/ subdirectory
# -----------------------------------------------------------------------------
add_library(glad STATIC
    glad/src/glad.c
)
target_include_directories(glad PUBLIC glad/include)

# -----------------------------------------------------------------------------
# GLM — use the system installation (header-only)
find_package(glm REQUIRED)

# -----------------------------------------------------------------------------
# Eigen — fetched at configure time, header-only so no build step needed
# Pin to 3.4.1 (stable, widely tested) or swap tag to "5.0.0" for the new
# major release. Note: 5.0.0 drops some deprecated APIs from 3.x.
# -----------------------------------------------------------------------------
include(FetchContent)

FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.1
    GIT_SHALLOW    TRUE          # only fetch the tagged commit, not full history
)

# Suppress Eigen's own tests and install targets
set(EIGEN_BUILD_DOC     OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING       OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(eigen)

# -----------------------------------------------------------------------------
# Link everything together
# -----------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME} PRIVATE
    OpenGL::GL
    glfw
    glad
    Eigen3::Eigen
    glm::glm
)


target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    RESOURCE_DIR="${CMAKE_SOURCE_DIR}/resources/"
)

# -----------------------------------------------------------------------------
# Compiler warnings — useful for a learning project
# -----------------------------------------------------------------------------
if (MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()