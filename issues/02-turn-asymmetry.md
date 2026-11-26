# 02: Turn asymmetry
Depends on: 01
Covers: R5, R6, R7, R8, R9, R10

## What to build
Turn asymmetry as specified in `docs/spec.md` and defined in `docs/glossary.md`.

In the body wave, scale each node's offset h(s) by (1 + a) when it lies on the outside of the local curve and by (1 − a) when it lies on the inside. Use the signed per-node nose path curvature from issue 01: the outside is the side away from the curve's centre. a = `asymmetry_max` · min(|κ| / `asymmetry_full_curvature`, 1). Frequency and phase are unchanged.

Change the existing "wave adds at most 0.3 rad" test's allowance to (1 + a_max) · 0.3 rad = 0.45 rad. Derive it in the test from a_max, do not tune it. Add the new tests for R6 and R7. Do not weaken any other test. If one fails, stop and report.

## Interfaces
- Uses: the signed per-node nose path curvature from issue 01.
- Adds: `BodyParams::asymmetry_max` (0.5, dimensionless) and `BodyParams::asymmetry_full_curvature` (0.8, in 1/m). Set them in `fishes.cpp` and in the test helper `bodyParams`.

## Acceptance criteria
- [ ] R6: with length passes 0, on steady circles of curvature 0, 0.4, 0.8 and 1.6 /m, both directions, at several phases: each node's offset along the spine normal from the zero-amplitude spine on the same nose path (front stiffness included) is (1 + a)·h(s) on the outside and (1 − a)·h(s) on the inside, within 1 mm (a = 0, 0.25, 0.5, 0.5).
- [ ] R7: straight at cruise with uneven dt (all ≤ 1/30 s), the tail tip's left and right peak offsets over a beat are equal within 2 %.
- [ ] R8: in the tightest turn, both directions, the worst bend between neighbouring segments is at most the path's bend per segment plus 0.45 rad, every frame.
- [ ] R5, R9, R10: the existing segment-length, body line, outline and rigid-part tests pass unchanged.
- [ ] The full test suite passes and the app builds.
