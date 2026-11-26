# 02: Shark separation from the body line
Depends on: 01
Covers: R10, R11, R12 (shark half)

## What to build
In shark separation, measure from the shark's own nose to the nearest point on each other shark's body line, using `nearestPointOnBodyLine` from issue 01, in place of the other shark's nose (R10). Change nothing else in the separation term: `SHARK_COMFORT_RANGE`, the ramp and `W_SEPARATION` stay, and the term stays unnormalised.

Eating, targeting and the safety filter stay unchanged (R11). See `docs/spec.md`.

## Acceptance criteria
- [ ] Shark separation uses the nearest point on the other shark's body line, measured from the shark's own nose. The range, ramp and weight are unchanged, and the term is not normalised (R10).
- [ ] Eating, targeting and the safety filter are unchanged (R11).
- [ ] Both executables build, and all tests pass.
- [ ] Visual check by the user: sharks no longer swim through each other's bodies in normal swimming (R12, shark half).
