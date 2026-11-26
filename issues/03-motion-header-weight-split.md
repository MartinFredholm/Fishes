# 03: Shared motion header and weight split
Depends on: 02
Covers: R13, R14, R20 (header part)

## What to build
A behaviour-neutral prefactor for Rev 1 (see `docs/spec.md`, R14 and R20).

Move `MotionParams`, `Commands`, `integrate`, `CLEARANCE_TIE` and the world size (with the constants it is computed from) out of the main file into a shared header and source, unchanged. Build them into a library that both executables link. The main file then uses them from there.

Split `W_SEPARATION` into `W_FISH_SEPARATION` (fish separation from other fish) and `W_SHARK_SEPARATION` (shark separation). Both stay 0.9 in this issue, so the simulation behaves exactly as before. Issue 05 retunes the shark weight.

## Interfaces
`motion.h` provides:
- `struct MotionParams` and `struct Commands`, unchanged.
- `Kinematics integrate(const Kinematics &kin, Commands commands, const MotionParams &params, float dt)`, unchanged. dt in s.
- `WORLD_WIDTH`, `WORLD_HEIGHT` in m, world frame as in `kinematics.h` (y down).
- `CLEARANCE_TIE` in m.

The library must not need the window library at link time, so the test executable can use it.

## Acceptance criteria
- [ ] `MotionParams`, `Commands`, `integrate`, `CLEARANCE_TIE` and the world size are defined once, in the shared header and source, and the main file uses them from there.
- [ ] `W_SEPARATION` no longer exists. Fish separation uses `W_FISH_SEPARATION` = 0.9 and shark separation uses `W_SHARK_SEPARATION` = 0.9.
- [ ] The definitions moved are unchanged, and no other behaviour changes.
- [ ] Both executables build, the test executable links the new library without the window library, and all tests pass. `N_SHARKS` is still 2.
