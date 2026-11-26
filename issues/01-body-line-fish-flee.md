# 01: Body line and fish fleeing
Depends on: none
Covers: R1, R2, R3, R4, R5, R6, R7, R8, R9, R13, R12 (fish half)

## What to build
The tracer bullet for body-aware steering (see `docs/spec.md`).

- Add the **body line** to the body state. Set it in both the seed and the body update, from the nose path, before the body wave is applied, so it does not depend on the wave (R4). Place its points at uniform s along the nose path, the same way the spine nodes are placed before the wave. Reuse that placement code; do not write a second path walk.
- Add the pure nearest-point function at the body module's interface (R6–R8).
- In fish fleeing, use the nearest point on each shark's body line in place of the shark's nose. Change nothing else in the flee term: range, ramp, normalisation and weight stay as they are (R9).
- Add doctest coverage for R1–R8 at the two seams described in the spec's testing approach.

Shark separation is issue 02. Do not change it here.

## Interfaces
- `constexpr int BODY_LINE_POINTS = 5;` in the body module header.
- `SharkBody::body_line`: `std::vector<Vector2>`, world meters, y down, `BODY_LINE_POINTS` points from the nose (index 0, s = 0) to the tail tip (s = 1).
- `Vector2 nearestPointOnBodyLine(Vector2 p, const std::vector<Vector2> &body_line);` returns the closest point on the line's segments, in world meters.

## Acceptance criteria
- [ ] After seeding and after every update, the body line has 5 points at s = 0, 0.25, 0.5, 0.75 and 1 (R1).
- [ ] Each point lies within 1 mm of the nose path, at an arc length behind the nose within 1 mm of s·L. Point 0 is the nose (R2).
- [ ] At spawn, each point is within 1 mm of nose − heading·s·L (R3).
- [ ] For the same nose path, the body lines with amplitude 0 and amplitude 0.1 differ by at most 1e-6 m per point (R4).
- [ ] R2 holds for straight and circular nose paths (radius down to 0.625 m), with even and uneven dt up to 1/30 s (R5).
- [ ] The nearest point is within 1e-5 m of the analytical answer: the nose for a position ahead of it, the tail tip for a position behind it, and the perpendicular foot for a position beside a segment (R6, R7).
- [ ] For a bent body line, a position inside the bend gets the nearest point over all segments (R8).
- [ ] Fish fleeing uses the nearest body-line point. The flee range, ramp, normalisation and weight are unchanged (R9).
- [ ] Both executables build, and all tests pass, old and new (R13).
- [ ] Visual check by the user: fish keep clear of the whole shark, tail included (R12, fish half).
