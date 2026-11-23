cmake_minimum_required(VERSION 3.20)
project(Fishes CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(raylib REQUIRED)

# Apple Clang has no bundled OpenMP runtime; point CMake at Homebrew's libomp.
if(APPLE AND NOT OpenMP_ROOT)
    execute_process(COMMAND brew --prefix libomp
                    OUTPUT_VARIABLE OpenMP_ROOT
                    OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
find_package(OpenMP REQUIRED)

add_executable(fishes fishes.cpp)
target_link_libraries(fishes raylib OpenMP::OpenMP_CXX)


