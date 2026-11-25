# 01: Tracer — spine follows the nose path, drawn as a polyline
Depends on: none
Covers: R4, R5, R6, R7, R12 (path part), R16, R18

## What to build
The thinnest path through the whole system: body module, test target and drawing. See `docs/spec.md` and use `docs/glossary.md` terms.

- Prefactor: move `Kinematics` out of `fishes.cpp` into its own header, unchanged, so the body module can use it.
- New shark body source file with seeding and the body update seam. No body wave yet: the update records the nose path, trims it, advances nothing else, and places the 24 spine nodes on the path (R5–R7). The update must not assume a constant dt (R12).
- Vendor doctest's single header under `third_party/`. **Ask the user to confirm the version before downloading.** Add a CTest test executable that links only what it needs (no window).
- In `main`, each `Shark` owns a `SharkBody`, seeded at spawn (R6) and updated every frame after `integrate`. It is double-buffered like the rest of `Shark`. Draw the shark as a polyline through its spine instead of the triangle. `kin.pos` is the nose (R4). The existing eating, targeting, fleeing and separation code stays unchanged.
- Fish are unchanged.

## Interfaces
All positions in meters, world frame, y down. The left normal of a heading `fwd` is `(fwd.y, −fwd.x)`, as in the existing code. Spine node 0 is the nose (s = 0) and node 23 is the tail tip (s = 1).

- `SharkBody`: the nose path (points with accumulated arc length, m), `phase` (rad), `spine` (24 × `Vector2`).
- `BodyParams`: `length` (L, m), `node_count` (24), `wave_amplitude` (A, dimensionless), `wave_number` (k, dimensionless), `strouhal` (St), `length_passes` (1). `wave_amplitude = 0` turns the wave off.
- `SharkBody seedSharkBody(const Kinematics&, const BodyParams&)`
- `SharkBody updateSharkBody(const SharkBody&, const Kinematics&, const BodyParams&, float dt)`

Issue 03 adds the wave inside `updateSharkBody` without changing these signatures. Issue 04 reads `spine`.

## Acceptance criteria
- [ ] The project builds, and the main executable runs with sharks drawn as polylines starting at `kin.pos`.
- [ ] The test executable builds and passes under CTest.
- [ ] R5: driving the nose along a known path keeps at least L of path, and older points are trimmed.
- [ ] R6: right after seeding, the spine is a straight line of length L behind the nose along the initial heading.
- [ ] R7: with `wave_amplitude = 0`, on straight and circular nose paths, every node is within 1 mm of the path, at an arc length within 1 mm of s_i·L.
- [ ] R12: the same path driven with uneven dt values (all ≤ 1/30 s) still meets R7.
- [ ] R16: the body module works in meters only; pixel conversion happens only in drawing.
- [ ] Fish behaviour and drawing are unchanged.
