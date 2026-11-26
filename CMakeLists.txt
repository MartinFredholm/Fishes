cmake_minimum_required(VERSION 3.20)
project(Fishes CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(raylib REQUIRED)

# Apple Clang has no bundled OpenMP runtime; point CMake at Homebrew's libomp.
if(APPLE AND NOT OpenMP_ROOT)
    execute_process(COMMAND brew --prefix libomp
                    OUTPUT_VARIABLE OpenMP_ROOT
                    OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
find_package(OpenMP REQUIRED)

add_library(shark_body STATIC shark_body.cpp)
target_include_directories(shark_body PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# Headers only (Vector2, inline raymath): not linking raylib keeps the tests
# free of the window library and fails the link if the body module calls it.
target_include_directories(shark_body SYSTEM PUBLIC
    $<TARGET_PROPERTY:raylib,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_options(shark_body PRIVATE -Wall -Wextra)

add_library(motion STATIC motion.cpp)
target_include_directories(motion PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(motion SYSTEM PUBLIC
    $<TARGET_PROPERTY:raylib,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_options(motion PRIVATE -Wall -Wextra)

add_library(shark_avoidance STATIC shark_avoidance.cpp)
target_include_directories(shark_avoidance PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(shark_avoidance PUBLIC shark_body motion)
target_compile_options(shark_avoidance PRIVATE -Wall -Wextra)

add_executable(fishes fishes.cpp)
target_compile_options(fishes PRIVATE -Wall -Wextra -ffast-math -mcpu=native)
target_link_libraries(fishes shark_avoidance shark_body motion raylib OpenMP::OpenMP_CXX)

enable_testing()
add_executable(shark_body_tests tests/body_update_tests.cpp tests/body_wave_tests.cpp
    tests/body_outline_tests.cpp tests/rigid_parts_tests.cpp
    tests/body_line_tests.cpp tests/motion_tests.cpp
    tests/shark_avoidance_tests.cpp)
target_include_directories(shark_body_tests PRIVATE third_party)
target_compile_options(shark_body_tests PRIVATE -Wall -Wextra)
target_link_libraries(shark_body_tests shark_avoidance shark_body motion)
add_test(NAME shark_body_tests COMMAND shark_body_tests)
