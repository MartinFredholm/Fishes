# 04: Turn bend replaces the nose path
Depends on: 01, 03
Covers: R6, R7, R12 (rework), R18 (test coverage), R19, R20

## What to build
Rework of 01, caused by spec rev 1. The spine is shaped by the turn bend, built from the current `kin`, instead of being laid along the recorded nose path. See `docs/spec.md` R6, R7, R19, R20 and the rev 1 design decisions.

- In `updateSharkBody`, replace the path placement with the turn bend's arc (R7). It starts at `kin.pos`, runs back from `kin.heading`, has curvature r/v, and puts node i at arc length s_i·L behind the nose. When v = 0 it is a straight line, and it stays finite for every v ≥ 0 (R20).
- 03's wave, phase and length pass run on this spine unchanged, including 03's rule that zero amplitude skips them.
- Remove the path record: `PathPoint`, `SharkBody::path`, path trimming and placement on the path.
- `seedSharkBody` returns an ordinary frame at phase 0: the same turn bend, wave and length pass as an update (R6). It is not a straight special case.
- `main` keeps seeding at spawn and updating after `integrate`. Drawing is unchanged (polyline).

**Retired tests, delete them** (the user approved this). They check the removed path record, the old straight-seed R6, or nodes on the recorded path. Some drive `kin` around a circle with `kin.r = 0`, which no longer describes a turn.
- `shark_body_tests.cpp`: "R6: seeded spine is straight…", "R5: path keeps at least L…", "R7: zero amplitude, circular path…", "R12: uneven dt (all <= 1/30 s) still meets R7", and any helpers only they use.
- `review_01_tests.cpp`: "review 01 R6: first update after spawn…", "review 01 R5/R7: tight turn…", "review 01 R12: uneven dt including dt = 0 while turning", "review 01 R5: long swim…", and helpers only they use.
- `review_01_round2_tests.cpp`: "review 01 round 2 R7/R12: varying speed and curvature…" and "review 01 round 2 R5: retained path covers L…".

Keep: "R7: zero amplitude, straight path…" (r = 0 makes the arc the straight track), "review 01: default heading…", "review 01 R4: spine node 0 is kin.pos every frame", "review 01 round 2 R12: very small steps keep the spine finite", all of `body_wave_tests.cpp` and all of `review_03_tests.cpp`. If a kept test fails, stop and report it. Do not adapt it.

## Interfaces
Issues 05 and 06 use only `spine`.
```cpp
struct SharkBody {
    float phase = 0.0f;              // rad, [−π, π]
    std::vector<Vector2> spine;      // [0] nose … [node_count−1] tail tip, world m
};
SharkBody seedSharkBody(const Kinematics &kin, const BodyParams &params);   // phase 0, ordinary frame
SharkBody updateSharkBody(const SharkBody &body, const Kinematics &kin,
                          const BodyParams &params, float dt);
```
`BodyParams` is unchanged. `PathPoint` is removed.

## Acceptance criteria
- [ ] R7: with `wave_amplitude = 0`, for r = 0 and for r of both signs (including ±r_max at v_min and at v_max), every node is within 1 mm of the arc and within 1 mm of s_i·L behind the nose along it. The test computes the arc independently, as the steady-turn track of the nose.
- [ ] R6: the first update after spawn at v = 1.0 m/s, r = 0 and dt = 1/60 s moves no node more than 0.05 m. The seeded spine already carries the wave at phase 0.
- [ ] R19: two bodies, one swum straight and one swum through turns, set to the same phase and updated with the same `kin` and dt, give spines that agree within 0.1 mm.
- [ ] R20: v = 0 with `wave_amplitude = 0` gives a straight spine along `kin.heading` at s_i·L within 1 mm. For v from 0 up to v_min and r = ±r_max, every spine coordinate is finite.
- [ ] R12: uneven dt values (all ≤ 1/30 s, including 0) while turning still meet R9 and R10.
- [ ] The listed retired tests are deleted, and all kept tests pass unchanged.
- [ ] No path record remains in the body module.
- [ ] Visual run: when the shark starts turning, the whole body bends at once, with no bend creeping down from the head.
