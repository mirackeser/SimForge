cmake_minimum_required(VERSION 3.20)
project(SimForge VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(SIMFORGE_BUILD_TESTS "Build unit tests" ON)

include(FetchContent)

FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 3.4
  GIT_SHALLOW TRUE
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  glm
  GIT_REPOSITORY https://github.com/g-truc/glm.git
  GIT_TAG 1.0.1
  GIT_SHALLOW TRUE
)

FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.91.5
  GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(glfw glm imgui)

add_library(glad STATIC third_party/glad/src/glad.c)
target_include_directories(glad PUBLIC third_party/glad/include)

add_library(imgui_lib STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_lib PUBLIC
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glad/include
)
target_link_libraries(imgui_lib PUBLIC glfw)
target_compile_definitions(imgui_lib PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLAD)
target_link_libraries(imgui_lib PUBLIC glad)

add_executable(SimForge
  src/main.cpp
  src/app/Application.cpp
  src/render/Renderer.cpp
  src/render/Shader.cpp
  src/physics/PhysicsWorld.cpp
  src/physics/Collision.cpp
  src/physics/SpatialHash.cpp
  src/physics/Constraint.cpp
  src/ecs/World.cpp
  src/scene/DemoScenes.cpp
)
target_include_directories(SimForge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  third_party/glad/include
)
target_link_libraries(SimForge PRIVATE
  glfw
  glad
  glm::glm
  imgui_lib
)
if(WIN32)
  target_link_libraries(SimForge PRIVATE opengl32)
else()
  find_package(OpenGL REQUIRED)
  target_link_libraries(SimForge PRIVATE OpenGL::GL)
endif()

if(SIMFORGE_BUILD_TESTS)
  enable_testing()
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
    GIT_SHALLOW TRUE
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  add_executable(SimForgeTests
    tests/test_collision.cpp
    tests/test_spatial_hash.cpp
    tests/test_advanced_physics.cpp
    src/physics/Collision.cpp
    src/physics/SpatialHash.cpp
    src/physics/Constraint.cpp
    src/ecs/World.cpp
  )
  target_include_directories(SimForgeTests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(SimForgeTests PRIVATE GTest::gtest_main glm::glm)
  include(GoogleTest)
  gtest_discover_tests(SimForgeTests)
endif()
