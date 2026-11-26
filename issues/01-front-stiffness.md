# 01: Front stiffness
Depends on: none
Covers: R1, R2, R3, R4, R5, R9, R10

## What to build
Front stiffness as specified in `docs/spec.md` (design decisions on front stiffness and its anchor) and defined in `docs/glossary.md`.

In the body update, after the spine nodes are placed on the nose path and before the body wave, rebuild the front (s < `front_stiff_s`) forward from the node at s = `front_stiff_s`. Start from the spine's direction there, and bend each front segment by the nose path's local curvature times smoothstep(s / `front_stiff_s`), keeping segment lengths at L/23. Nodes at s ≥ `front_stiff_s` stay where they are. This applies at any amplitude, including zero, so it must run before the zero-amplitude early return.

The body line is not touched.

Rewrite these existing tests to their intent, as listed in the spec's testing approach:
- the three on-path tests in turns: check only s ≥ 0.25;
- "spine node 0 is the nose every frame": straight swimming only.

Add the new tests for R2 and R3. Do not weaken any other test. If one fails, stop and report.

## Interfaces
- `BodyParams::front_stiff_s`: the stiff length as a fraction of L. Set it to 0.25 in `fishes.cpp` and in the test helper `bodyParams`.
- A signed curvature of the nose path at each spine node, in the body module. Units 1/m. Positive when the path curves screen-left of the swimming direction (the same sign as `kin.r`). It is evaluated at arc s·L behind the nose. Issue 02 uses it for turn asymmetry, so it must be smooth enough on a swum straight path, with uneven dt, that the left and right tail peaks stay equal within 2 % (R7).

## Acceptance criteria
- [ ] R1: with zero amplitude, nodes at s ≥ 0.25 lie on the nose path with the existing tolerances, in the tightest-turn, varying-speed-and-curvature and one-hour tests.
- [ ] R2: with zero amplitude, on steady circles of radius 1.25 m at 1 m/s and 0.625 m at 0.5 m/s, both directions: each front node bends less than the path per segment; the bend does not decrease from the nose toward s = 0.25; the last front node bends at least 90 % of the path's bend per segment.
- [ ] R3: under the same conditions, the direction from node 0 to node 1 lags `kin.heading` toward the outside of the turn by κ·L/8 ±15 % (0.3 rad and 0.6 rad).
- [ ] R4: swimming straight with zero amplitude, node 0 equals `kin.pos` within 1e-5 m every frame, and the existing straight-path and straight body wave tests pass unchanged.
- [ ] R5: the existing segment-length tests pass unchanged.
- [ ] R9, R10: the existing body line, outline and rigid-part tests pass unchanged.
- [ ] The full test suite passes and the app builds.
