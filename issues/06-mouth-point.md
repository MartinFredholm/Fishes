# 06: Mouth point for targeting and eating
Depends on: 05
Covers: R4, R23, R18 (test coverage)

## What to build
The mouth point seam, plus its use in targeting and eating. See `docs/spec.md` R4 and R23, the mouth point design decisions, and the glossary's mouth point.

- In the body module, add `sharkMouthPoint`. It returns the point s_p·L ahead of `kin.pos` along the head arc (curvature r/v, tangent to `kin.heading`). At v = 0 it lies s_p·L straight ahead. It is computed from `kin` and the body parameters alone, never from a `SharkBody`. Share the head-arc computation with 05's head placement, so the two cannot drift apart.
- In `fishes.cpp`, `sharkGetTarget` measures from the mouth point: the grid query centre, `relative_pos` and `distance`. The chase acceleration and the eat check in `killFish` read the target, so they follow without further changes. The 45° heading rule is unchanged.
- Fish fleeing, shark–shark separation, wall repulsion and the safety filter keep measuring from `kin.pos`. Do not change them.
- Restore the test that 05 skipped, "review 03 / design: the drawn nose sways about kin.pos…". Change only its reference point from `kin.pos` to the mouth point (the user approved this), rename it to match, and remove the skip.

## Interfaces
```cpp
Vector2 sharkMouthPoint(const Kinematics &kin, const BodyParams &params);   // world m
```

## Acceptance criteria
- [ ] R23: the mouth point lies on the head arc, s_p·L ahead of `kin.pos` along it, within 1 mm: straight, turning both ways (including ±r_max at v_min) and at v = 0. The test computes the arc independently.
- [ ] R23: with `wave_amplitude = 0`, the mouth point coincides with spine node 0 within 1 mm, straight and turning.
- [ ] R23: it is finite for every v ≥ 0 and |r| ≤ r_max.
- [ ] R4 (code inspection): targeting and eating measure from the mouth point. Fleeing, separation, wall repulsion and the safety filter measure from `kin.pos`. Fish are unchanged.
- [ ] The restored nose-sway test passes about the mouth point.
- [ ] All earlier tests still pass.
- [ ] Visual run: a shark eats fish at its drawn nose, not 1 m behind it.
