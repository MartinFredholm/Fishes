# Body-aware separation and fleeing

## Problem
Fish flee from a shark's nose only, and sharks keep their distance from each other's noses only. The shark is now a 3 m body behind its nose. Fish swim close past its trunk and tail, and two sharks can swim through each other's bodies.

## Goal
Fish flee from the whole shark, and each shark steers its nose away from the whole body of every other shark. Steering uses a body line taken from the nose path, so the animation still never feeds back into the dynamics. Eating and targeting are unchanged.

## Requirements

### Body line (seam 1: body update)
- **R1**: The body state holds a body line with a fixed number of points, set by a named constant with the value 5. Point i lies at s_i = i/(n − 1), so s = 0, 0.25, 0.5, 0.75 and 1.
- **R2**: Every body-line point lies within 1 mm of the nose path, at an arc length behind the nose within 1 mm of s_i·L. Point 0 is the nose.
- **R3**: At spawn, the body line is straight along the initial heading, from the nose to L behind it, with each point within 1 mm of its expected position.
- **R4**: The body line does not depend on the body wave. For the same nose path, the body line with the wave amplitude set to zero and with A = 0.1 differs by at most 1e-6 m per point.
- **R5**: R2 holds for straight and circular nose paths, and with uneven dt values up to 1/30 s.

### Nearest point (seam 2)
- **R6**: Given a position and a body line, the nearest-point function returns the point on the body line's segments that is closest to the position, within 1e-5 m of the analytical answer.
- **R7**: A position ahead of the nose gets the nose, a position behind the tail tip gets the tail tip, and a position beside a segment gets its perpendicular foot on that segment.
- **R8**: Where the body line bends, a position gets the nearest point over all segments, not only the nearest segment by its end points.

### Steering
- **R9**: Fish fleeing adds one term per shark, as today, with the nearest point on that shark's body line in place of its nose. `FISH_FLEE_RANGE` (1.8 m), the ramp 1 − d/`FISH_FLEE_RANGE`, the normalisation and `W_FLEE` are unchanged.
- **R10**: Shark separation adds one term per other shark, as today, with the nearest point on the other shark's body line in place of its nose. The distance is from the shark's own nose. `SHARK_COMFORT_RANGE` (4 m), the ramp and `W_SEPARATION` are unchanged, and the term is not normalised.
- **R11**: Eating and targeting still measure from the nose. The safety filter is unchanged.
- **R12**: Visual check by the user: fish keep clear of the whole shark, tail included, not only ahead of the head. Sharks no longer swim through each other's bodies in normal swimming.

### Build
- **R13**: The main executable and the test executable build. All existing tests pass.

## Design decisions
- **Steering uses the body line, not the spine.** Reason: the body line comes from the nose path, which is dynamics history. The spine includes the body wave, so steering on it would feed the animation into the dynamics. Cost: the waving tail can sit up to about 0.3 m (A·L·amp(1)) off the body line, which is small against both ranges.
- **Five body-line points.** Reason: 4 segments of 0.75 m are enough to cover the whole shark. Each segment cuts the curved nose path by at most about 0.13 m in the tightest turn (radius 0.625 m at `v_min`) and about 0.06 m at cruise. These are my figures, not measured. Rejected: 24 points (the spine's count) and the raw nose path (about 180 points at 60 fps), which cost more for no visible gain.
- **Distance to the body line itself.** Reason: the half-width is at most 0.18 m, and the pectoral tips reach 0.6 m out. That is within the tuning slack of the ranges. Rejected: subtracting the half-width or including the fins and cephalofoil, which would read body geometry, the drawing data.
- **One term per shark, from its nearest body-line point.** Reason: the strength then does not depend on the point count, and a fish ahead of the nose gets the same push as before. Rejected: one term per segment, which makes the body push harder than the nose and turns the point count into a tuning knob.
- **Ranges, ramps, weights and normalisation unchanged.** Reason: only the measured point changes. Tuning, if needed, comes after the visual check.
- **Soft repulsion only.** Reason: the user's decision. Rejected: a hard guarantee in the safety filter.
- **Nose against body.** Reason: the user's decision. A shark steers its own nose away from other sharks' bodies. Its own trailing body is not checked.
- **The body line is computed in the body update and stored in the body state.** Reason: it is computed once per shark per frame, not once per fish. The path and the arc-length placement already live in the body module and are tested. Cost: the body module now also serves steering. Fish read the previous frame's sharks, whose body line was built from the same kinematics, so the nose and the body line agree.
- **The nearest-point function lives in the body module.** Reason: it sits at a testable seam next to the body line.
- **No early skip by distance in the fish loop.** Reason: 4 segments × 2 sharks is 32 point-to-segment tests per fish per frame. An early-out is a guess at an optimisation and can be added if profiling shows a need.
- **The steering change is not unit tested.** Reason: it is one swap per loop in existing formulas. Pulling steering out of the main file into a testable module would be a refactor of its own.
- **Visual acceptance, no instrumented run.** Reason: the ranges are tuning values. A measured threshold would test numbers that were not chosen for a reason.

## Testing approach
- **Seams**
  - *Body update* (existing): takes the previous body state, kinematics and dt, and returns the new body state, now including the body line. Catches point count, placement on the nose path, arc lengths, the spawn state and independence from the body wave. Misses how steering uses the line.
  - *Nearest point* (new, pure): takes a position and a body line, and returns the nearest point on it. Catches projection onto a segment, clamping at the nose and the tail tip, and bends. Misses the steering terms and the ranges.
- **Unit tests (doctest)**
  - R1–R3: seed a shark and check the point count, the s values and the straight line.
  - R2, R5: drive straight and circular nose paths, with even and uneven dt, and check each point's distance to the path and its arc length.
  - R4: drive the same nose path with amplitude 0 and 0.1 and compare the body lines.
  - R6–R8: hand-built straight and bent body lines, with positions ahead of the nose, behind the tail tip, beside each segment and inside a bend.
- **Build check**: R9–R11 and R13. The project builds, and the existing tests pass.
- **Visual check by the user**: R12.

## Out of scope
- A hard no-contact guarantee or any change to the safety filter.
- Checking a shark's own trailing body against other sharks' bodies.
- Fish keeping clear of the fins, the cephalofoil or the body's width.
- Tuning the ranges, weights or ramps.
- Moving steering out of the main file or testing it directly.
- Spatial early-outs or other performance work.
- Any change to eating, targeting, the dynamics, the controller or the drawing.
