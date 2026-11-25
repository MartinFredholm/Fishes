# 05: Pivot path and head arc
Depends on: 04
Covers: R6, R7, R11, R12 (rework), R18 (test coverage), R20, R21, R22; R19 removed

## What to build
Rework of 04, caused by spec rev 2. For the shark, `kin.pos` is now the pivot at s_p = 1/3, not the nose. The body behind the pivot follows the pivot path. The head ahead of it lies on the head arc. See `docs/spec.md` R6, R7, R20–R22 and the rev 2 design decisions, and use the `docs/glossary.md` terms (pivot, pivot path, head arc, skid).

- Add `pivot` (s_p) to `BodyParams`. `SHARK_BODY` in `main` sets it to 1/3. It is a named value for tuning in 09.
- The body state records the pivot path again: each update adds `kin.pos`, and trimming keeps just enough track to cover (1 − s_p)·L (R21). An update with v = 0 and `kin.pos` unchanged adds no track (R20).
- Placement, with the wave amplitude at zero:
  - Each node with s_i ≥ s_p goes on the pivot path at arc length (s_i − s_p)·L behind `kin.pos`.
  - Each node with s_i < s_p goes on the head arc at arc length (s_p − s_i)·L ahead of `kin.pos`. The head arc is tangent to `kin.heading` and has curvature r/v. It is straight when v = 0. 04's closed-form arc can be reused, run forward.
  - Every coordinate stays finite for every v ≥ 0 and |r| ≤ r_max (R20).
- Spawn: `seedSharkBody` lays a straight pivot path behind `kin.pos` along `kin.heading`, at least (1 − s_p)·L long. It then builds an ordinary frame at phase 0 with the wave on (R6).
- 03's wave, phase and length pass run on this spine unchanged, including the rule that zero amplitude skips them.
- `main` still seeds at spawn and updates after `integrate`. Drawing stays a polyline. Targeting and eating still measure from `kin.pos` until 06, so for now the drawn nose lies 1 m ahead of where eating measures.

**Retired tests, delete them** (the user approved this). They check the nose at `kin.pos`, nodes at s_i·L behind it, or a spine that ignores history.
- `review_01_tests.cpp`: "review 01 R4: spine node 0 is kin.pos every frame", plus any helpers only it uses.
- `shark_body_tests.cpp`: "R7: zero amplitude, straight path: nodes on the path at s_i L", plus any helpers only it uses.
- `turn_bend_tests.cpp`: "R7: zero amplitude, every node on the turn bend's arc at s_i L", "R19: different swimming histories…", "R20: v = 0 gives a straight spine along kin.heading at s_i L", plus any helpers only they use.

**Adapted test** (the user approved this). In `body_wave_tests.cpp` "R8: wave offset at the blog's six amplitude points", change only the expected along-position from s·L behind `moved.pos` to (s − s_p)·L behind it. Leave the rest of the test unchanged.

Keep unchanged: "review 01: default heading…", "review 01 round 2 R12: very small steps keep the spine finite", the rest of `body_wave_tests.cpp`, the rest of `turn_bend_tests.cpp` (R6 first update, R20 finite, R12 uneven dt), and `review_03_tests.cpp` except "review 03 / design: the drawn nose sways about kin.pos…". That test moves to issue 06. Until then it may fail, so mark it skipped with a pointer to 06 and nothing else. If any other kept test fails, stop and report it. Do not adapt it.

## Interfaces
```cpp
struct BodyParams {
    // …existing fields unchanged…
    float pivot;                     // s_p, fraction of L from the nose
};
struct SharkBody {
    float phase = 0.0f;              // rad, [−π, π]
    std::vector<Vector2> pivot_path; // world m; layout is the implementer's choice, no later issue reads it
    std::vector<Vector2> spine;      // [0] nose … [node_count−1] tail tip, world m
};
SharkBody seedSharkBody(const Kinematics &kin, const BodyParams &params);
SharkBody updateSharkBody(const SharkBody &body, const Kinematics &kin,
                          const BodyParams &params, float dt);
```
If the path needs accumulated arc lengths, store them in a small point struct in place of `Vector2`. Issue 06 uses `BodyParams::pivot`, and issues 07 and 08 use only `spine`.

## Acceptance criteria
- [ ] R6: after seeding, the pivot path is a straight line behind `kin.pos` along `kin.heading`, at least (1 − s_p)·L long. The first update at v = 1.0 m/s, r = 0 and dt = 1/60 s moves no node more than 0.05 m.
- [ ] R7: with `wave_amplitude = 0`, the test swims the pivot along known tracks: straight, circles of both signs (including ±r_max at v_min and at v_max), and varying r and v. Every node behind the pivot is within 1 mm of the track at (s_i − s_p)·L behind `kin.pos`, within 1 mm. Every head node is within 1 mm of the head arc at (s_p − s_i)·L ahead of `kin.pos`, within 1 mm. The test computes the track and the arc independently.
- [ ] R11: the tail-tip peak lateral offset from the straight track of `kin.pos` still holds (existing test, unchanged).
- [ ] R12: uneven dt values (all ≤ 1/30 s, including 0) while turning still meet R7, R9 and R10.
- [ ] R20: v = 0 with `wave_amplitude = 0` gives a straight head along `kin.heading` at (s_p − s_i)·L within 1 mm. An update with v = 0 and `kin.pos` unchanged leaves the nodes behind the pivot within 1 mm of where they were. For v from 0 up to v_min and r = ±r_max, every spine coordinate is finite.
- [ ] R21: on long straight and turning swims, after every update the path covers at least (1 − s_p)·L behind `kin.pos`. Without its oldest point it would cover less, and its size stays bounded.
- [ ] R22: two bodies with the same pivot path and phase but different r on their last update, updated with the same `kin` and dt, give spines that agree within 0.1 mm.
- [ ] The listed retired tests are deleted, the R8 test is adapted as described, and all other kept tests pass unchanged.
- [ ] Visual run: when the shark starts turning, the head swings into the turn at once, and the body behind follows the pivot's track without sliding sideways.
