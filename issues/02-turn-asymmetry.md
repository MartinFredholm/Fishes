# 02: Turn asymmetry
Depends on: 01
Covers: R5, R6, R7, R8, R9, R10, R11, R12

## What to build
Turn asymmetry as specified in `docs/spec.md` (Rev 1) and defined in `docs/glossary.md`: one body-wide a from the body turn, and each node offset toward the outside of the turn by the smooth form. It acts in the body wave, so only when the amplitude is nonzero. Frequency and phase are unchanged.

The body turn needs the nose path's heading at s = 1. Read it so that it does not step each time the tail passes a path point; R12 catches such steps.

Change the existing "wave adds at most 0.3 rad" test's allowance to (1 + a_max) · 0.3 rad = 0.45 rad. Derive it in the test from a_max, do not tune it. Add the new tests for R6, R7, R11 and R12. Do not weaken any other test. If one fails, stop and report.

## Interfaces
- Adds: `BodyParams::asymmetry_max` (0.5, dimensionless) and `BodyParams::asymmetry_full_curvature` (0.8, in 1/m). Set them in `fishes.cpp` and in the test helper `bodyParams`.
- Does not use issue 01's per-node nose path curvature.

## Acceptance criteria
- [ ] R6: on steady circles of curvature 0, 0.4, 0.8 and 1.6 /m, with length passes 0, both directions, several phases: each node's offset from the zero-amplitude spine is h(s) + a·A·L·amp(s)·(2/π − 4/(3π)·cos 2θ) toward the outside, within 1 mm (a = 0, 0.25, 0.5, 0.5).
- [ ] R7: straight at cruise with uneven dt (all ≤ 1/30 s), the tail tip's left and right peak offsets over a beat are equal within 2 %.
- [ ] R8: in the tightest turn, both directions, the worst bend between neighbouring segments is at most the path's bend per segment plus 0.45 rad, every frame.
- [ ] R11: on the scripted straight → 90° arc (radius 1.25 m, 1 m/s) → straight path, with length passes 0, both directions, several phases, at the four sample points of the spec: offsets as in R6 with a from the body turn computed from the scripted geometry, within 1 mm; a = 0 more than L past the arc's end.
- [ ] R12: on the scripted cruise, tightest and S-turn swims at dt = 1/60 s, both directions: for every node and frame, |p(t + dt) − 2·p(t) + p(t − dt)| exceeds that of the same swim with `asymmetry_max` = 0 by at most 2 mm.
- [ ] R5, R9, R10: the existing segment-length, body line, outline and rigid-part tests pass unchanged.
- [ ] The full test suite passes and the app builds.
