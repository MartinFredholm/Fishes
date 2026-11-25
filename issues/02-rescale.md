# 02: Rescale world and shark parameters
Depends on: none
Covers: R1, R2, R3

## What to build
Change constants only, as listed in `docs/spec.md` R1–R3: `METERS_PER_PIXEL`, `N_FISH` and the shark constants. Fish parameters, `WALL_BUFFER_DIST` and `SPAWN_MARGIN` stay unchanged. No logic changes.

## Acceptance criteria
- [ ] `METERS_PER_PIXEL` is 0.02, giving a 30 × 16 m world in the 1500 × 800 px window.
- [ ] `N_FISH` is 4000. Fish length, width and motion parameters are unchanged.
- [ ] The shark constants have the R3 values. `K_phi`, `tau`, hunger rate, eat duration and the 45° rule are unchanged.
- [ ] The project builds, and existing tests (if any) pass.
- [ ] Visual run: fish school and sharks hunt in the larger world.
