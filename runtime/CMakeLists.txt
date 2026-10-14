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

