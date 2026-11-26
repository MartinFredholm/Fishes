# Fishes

## Build and test
- Build: `cmake --build build`
- Tests: `./build/shark_body_tests`

## Compile flags
Only `fishes.cpp` is built with `-ffast-math`. The libraries (`shark_body`, `motion`, `shark_avoidance`) are not, and `shark_avoidance` depends on that because it uses `infinity()`. Anything built outside CMake, such as a probe, must use each target's flags from `compile_commands.json`.
