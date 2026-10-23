cmake_minimum_required(VERSION 3.20)
project(palletizer_runtime LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Everything except main(): plant simulation, controller, cyclic executive.
add_library(runtime_core STATIC
  src/plant.cpp
  src/drive_motor.cpp
  src/pid.cpp
  src/speed_loop.cpp
  src/controller.cpp
  src/cyclic.cpp)
target_include_directories(runtime_core PUBLIC include)

add_executable(runtime src/main.cpp)
target_link_libraries(runtime PRIVATE runtime_core)

foreach(tgt runtime_core runtime)
  if (MSVC)
    target_compile_options(${tgt} PRIVATE /W4)
  else()
    target_compile_options(${tgt} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endforeach()

include(FetchContent)
FetchContent_Declare(googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)   # needed with MSVC
FetchContent_MakeAvailable(googletest)

enable_testing()
include(GoogleTest)
add_executable(runtime_tests
  tests/test_plant.cpp
  tests/test_controller.cpp
  tests/test_cyclic.cpp
  tests/test_pid.cpp
  tests/test_speed_loop.cpp
  tests/test_machine_pid.cpp)
target_link_libraries(runtime_tests PRIVATE runtime_core GTest::gtest_main)
gtest_discover_tests(runtime_tests)