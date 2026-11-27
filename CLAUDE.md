# Fishes

## Build and test
- Build: `cmake --build build`
- Tests: `./build/shark_body_tests`

## Compile flags
Only `fishes.cpp` is built with `-ffast-math`. The libraries (`shark_body`, `motion`, `shark_avoidance`) are not, and `shark_avoidance` depends on that because it uses `infinity()`. Anything built outside CMake, such as a probe, must use each target's flags from `compile_commands.json`.

## Body parameters
`BodyParams` is set in two places, `SHARK_BODY` in `fishes.cpp` and `bodyParams` in `tests/test_support.h`. A new field goes in both.
