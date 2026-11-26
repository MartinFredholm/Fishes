# Next feature: body-aware separation and fleeing

Input for `/discuss` and `/spec`. Not an issue. Written after issue 06, before its review.

## Request
- Sharks try not to collide with any part of another shark's body.
- Fish flee from the whole body of each shark, not only from its nose.

## Decided by the user
- **Soft repulsion only.** No hard guarantee in the safety filter.
- **Nose against body.** A shark steers its own nose (`kin.pos`) away from the nearest point on another shark's body. Its own trailing body is not checked.
- **Same comfort range.** `SHARK_COMFORT_RANGE` (4 m) stays. It is now measured to the nearest point on the other's body, not to its nose.

## Recommendation, not yet decided
- **The dynamics see the nose path, not the drawn spine.** Use the last L of `SharkBody::path`, a straight-line polyline with a fixed half-width from `WIDTH_OVER_LENGTH`. Reason: the path is dynamics history, so the spec's "the animation never feeds back" still holds. The drawn spine includes the body wave, so steering on it would feed the animation into the dynamics. Cost: the waving tail can sit up to ~0.3 m (A·L·amp(1)) off that line.
- **Fish flee from the nearest point on that body line, minus the local half-width,** within the same `FISH_FLEE_RANGE` (1.8 m). Skip a shark early when the fish is further than L + 1.8 m from its nose. Estimated cost: 4000 fish × 2 sharks × ~23 segments per frame. Not measured.

## Open questions
- Should the distance subtract the half-width (distance to the body edge) or not (distance to the centre line)? And should it include the fins and cephalofoil?
- The fish flee term is normalised (`W_FLEE * Vector2Normalize(a_flee)`). With a body-wide source, should the ramp `1 − d/FISH_FLEE_RANGE` and the normalisation stay as they are?
- Where should the point-to-body distance live? `shark_body.cpp` is the tested module and already owns the path. A function there would sit at a testable seam, but its input is dynamics state, not geometry.
- How to test the steering terms, which live in `fishes.cpp`, outside the test seams. Today they can only be checked with an instrumented copy of `fishes.cpp`.

## Conflicts with the current spec
- Out of scope: "Any change to … the steering behaviours …". This feature changes shark separation and fish fleeing.
- Out of scope: "Feedback from the animation into the dynamics." This holds only if the path is used, not the spine.
- R4: "fish fleeing and shark–shark separation measure from [the nose], using the existing code unchanged." This feature replaces R4 for those two uses. Eating and targeting still measure from the nose.

## Where the code is
- Shark separation: `desiredSharkAcceleration` in `fishes.cpp`, the loop over other sharks using `SHARK_COMFORT_RANGE`.
- Fish flee: `desiredFishAcceleration` in `fishes.cpp`, the loop over sharks using `FISH_FLEE_RANGE` and `W_FLEE`.
- Both already receive `const std::vector<Shark> &`, so `Shark::body.path` is available without changing their signatures.
- Path: `SharkBody::path` (`shark_body.h`). It keeps at least L behind the nose. `arc` increases toward the nose.
