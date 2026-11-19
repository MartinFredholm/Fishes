cmake_minimum_required(VERSION 3.20)
project(Fishes CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(raylib REQUIRED)

add_executable(fishes fishes.cpp)
target_link_libraries(fishes raylib)


