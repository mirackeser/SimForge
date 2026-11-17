cmake_minimum_required(VERSION 3.20)
project(SimForge VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(SIMFORGE_BUILD_SANDBOX "Build the OpenGL desktop sandbox" ON)
option(SIMFORGE_BUILD_TESTS "Build unit and integration tests" ON)
option(SIMFORGE_BUILD_HEADLESS "Build the headless scene runner" ON)
option(SIMFORGE_ENABLE_WAYLAND "Enable GLFW's Wayland backend on Linux" OFF)

include(FetchContent)
FetchContent_Declare(glm
  GIT_REPOSITORY https://github.com/g-truc/glm.git
  GIT_TAG 1.0.1
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(glm)

# The simulation core must remain usable without a window or graphics libraries.
add_library(SimForgePhysics STATIC
  src/ecs/World.cpp
  src/physics/PhysicsWorld.cpp
  src/physics/Collision.cpp
  src/physics/SpatialHash.cpp
  src/physics/Constraint.cpp
)
target_include_directories(SimForgePhysics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(SimForgePhysics PUBLIC glm::glm)
if(WIN32)
  target_compile_definitions(SimForgePhysics PUBLIC NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

add_library(SimForgeScenes STATIC src/scene/DemoScenes.cpp)
target_link_libraries(SimForgeScenes PUBLIC SimForgePhysics)

if(SIMFORGE_BUILD_HEADLESS)
  add_executable(SimForgeHeadless src/headless/main.cpp)
  target_link_libraries(SimForgeHeadless PRIVATE SimForgeScenes)
endif()

if(SIMFORGE_BUILD_SANDBOX)
  FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE
  )
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_WAYLAND ${SIMFORGE_ENABLE_WAYLAND} CACHE BOOL "" FORCE)
  FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.5
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(glfw imgui)

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
  target_include_directories(imgui_lib PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
  target_link_libraries(imgui_lib PUBLIC glfw glad)
  target_compile_definitions(imgui_lib PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLAD GLFW_INCLUDE_NONE)
  if(WIN32)
    target_compile_definitions(imgui_lib PUBLIC NOMINMAX WIN32_LEAN_AND_MEAN)
  endif()

  add_executable(SimForge
    src/main.cpp
    src/app/Application.cpp
    src/render/Renderer.cpp
    src/render/Shader.cpp
  )
  target_link_libraries(SimForge PRIVATE SimForgeScenes imgui_lib)
  if(WIN32)
    target_link_libraries(SimForge PRIVATE opengl32)
  else()
    find_package(OpenGL REQUIRED)
    target_link_libraries(SimForge PRIVATE OpenGL::GL)
  endif()
endif()

if(SIMFORGE_BUILD_TESTS)
  enable_testing()
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
    GIT_SHALLOW TRUE
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  add_executable(SimForgeTests
    tests/test_collision.cpp
    tests/test_spatial_hash.cpp
    tests/test_advanced_physics.cpp
    tests/test_physics_world.cpp
  )
  target_link_libraries(SimForgeTests PRIVATE SimForgeScenes GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(SimForgeTests)
  if(SIMFORGE_BUILD_HEADLESS)
    add_test(NAME Headless.Smoke COMMAND SimForgeHeadless --scene cradle --steps 2)
    add_test(NAME Headless.RejectsInvalidSteps COMMAND SimForgeHeadless --steps -1)
    set_tests_properties(Headless.RejectsInvalidSteps PROPERTIES WILL_FAIL TRUE)
  endif()
endif()
