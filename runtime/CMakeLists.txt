cmake_minimum_required(VERSION 3.20)
project(palletizer_runtime LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(plant STATIC src/plant.cpp)
target_include_directories(plant PUBLIC include)

add_executable(runtime src/main.cpp)
target_link_libraries(runtime PRIVATE plant)

foreach(tgt plant runtime)
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
add_executable(plant_tests tests/test_plant.cpp)
target_link_libraries(plant_tests PRIVATE plant GTest::gtest_main)
gtest_discover_tests(plant_tests)