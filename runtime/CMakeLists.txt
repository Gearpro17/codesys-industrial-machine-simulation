cmake_minimum_required(VERSION 3.20)
project(palletizer_runtime LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(runtime src/main.cpp)
target_include_directories(runtime PRIVATE include)

if (MSVC)
  target_compile_options(runtime PRIVATE /W4)
else()
  target_compile_options(runtime PRIVATE -Wall -Wextra -Wpedantic)
endif()