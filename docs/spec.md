# Procedural swimming animation for the hammerhead shark

## Problem
The shark is drawn as a rigid red triangle centred on `kin.pos`. It does not look like a shark, and it does not swim: nothing moves along its body, and it does not bend in turns. It is also 0.4 m long, which is unrealistic for a hammerhead and too small on screen to show any body motion.

## Goal
The shark is a 3 m scalloped hammerhead that swims like the dogfish in the Julia blog post (https://julialang.org/blog/2021/08/sharks/): a travelling body wave, small at the head and large at the tail, whose frequency follows speed. In turns, its body follows the curve the nose has traced. The existing dynamics stay the source of truth. The animation reads them and never feeds back.

## Requirements

### World and shark parameters
- **R1**: `METERS_PER_PIXEL` is 0.02, so the 1500 × 800 px window shows a 30 × 16 m world.
- **R2**: `N_FISH` is 4000. Fish length, width and motion parameters are unchanged.
- **R3**: Shark parameters: L = 3.0 m, `v_min` 0.5 m/s, `v_cruise` 1.0 m/s, `v_max` 2.5 m/s, `r_max` 0.8 rad/s, `T_v` 1.0 s, `T_r` 0.4 s, sight range 6 m, comfort range 4 m, eat distance 0.3 m. `K_phi`, `tau`, hunger rate, eat duration and the 45° eating-angle rule are unchanged.
- **R4**: For the shark, `kin.pos` is the nose. Eating, targeting, fish fleeing and shark–shark separation measure from it, using the existing code unchanged. For fish, `kin.pos` is still the body centre.

### Body update (seam 1)
- **R5**: The body state records the path the nose has traced, with accumulated arc length. It keeps at least L of path behind the nose and trims the older part.
- **R6**: At spawn, the nose path is seeded as a straight line of length L behind the nose along the initial heading. The first frame shows a straight shark.
- **R7**: The spine has 24 nodes at uniform s_i = i/23, i = 0…23. With the wave amplitude set to zero, every node lies within 1 mm of the recorded nose path, at an arc length behind the nose within 1 mm of s_i·L.
- **R8**: The body wave offsets each node along the spine's local normal by h(s) = A·L·amp(s)·sin(k·s − φ), with A = 0.1, k = 5.3 and amp(s) interpolated smoothly through [0.2, 0.21, 0.23, 0.4, 0.88, 1.0] at s = 0, 0.2, 0.4, 0.6, 0.8 and 1.0.
- **R9**: Each update advances the phase by ω·dt, with ω = 2π·St·u/(2·A·L) and St = 0.3. With the decided values, that is ω = π·u rad/s. After one update, the phase differs from the expected value by at most 1e-6 rad.
- **R10**: After the wave, one head-to-tail pass resets each segment to its rest length L/23. The number of passes is a named constant. After every update, every segment is within 0.1 % of L/23.
- **R11**: During straight swimming at `v_cruise`, once the body has swum at least one body length, the tail tip's peak lateral offset from the nose path over one beat cycle is within ±20 % of A·L·amp(1) = 0.30 m.
- **R12**: The body update works with the existing variable, clamped timestep (dt ≤ 1/30 s). It does not assume a constant dt.

### Body geometry (seam 2)
- **R13**: The outline is built from the spine nodes. On each side, the outline point at node i lies at ±w(s_i)/2 along the local normal, within 1 mm. w(s) is the full body width, interpolated smoothly through the shape values below.
- **R14**: The caudal fin is the end of the outline taper, from s ≈ 0.75 to the tail tip at s = 1. There is no separate caudal shape.
- **R15**: Rigid parts are attached to the spine at their s positions, within 1 mm, and oriented by the local spine tangent: the cephalofoil, a pair of pectoral fins and a pair of pelvic fins. The dorsal fins are not drawn.
- **R16**: All geometry is in world meters. Conversion to pixels happens only when drawing.

### Shape values (starting values, unverified)
These come from memory of scalloped hammerhead proportions and have **not been checked**. The user checks them against a top-view photo. All are fractions of L.

| Part | Values |
|---|---|
| Width w(s) | s 0.00 → 0.05, 0.10 → 0.09, 0.25 → 0.12, 0.40 → 0.11, 0.60 → 0.07, 0.75 → 0.035, 0.90 → 0.02, 1.00 → 0.005 |
| Cephalofoil | span 0.27 (plausible range 0.24–0.30), leading edge at s = 0, wing tips swept back to s ≈ 0.08, midline chord 0.06 |
| Pectoral fins | root from s 0.26 to 0.32, span 0.14 each, tips swept back to s ≈ 0.38 |
| Pelvic fins | root from s 0.54 to 0.57, span 0.04 each, tips swept back to s ≈ 0.60 |

### Drawing
- **R17**: The shark is drawn as a single grey-bronze fill: the outline as a filled strip, plus the rigid parts. It is not red and has no shading. Fish drawing is unchanged.

### Tests
- **R18**: The shark's body code is in its own source file. A test executable using doctest (vendored under `third_party/`) runs through CTest and covers R5–R16. The main executable and the existing build still work.

## Design decisions
- **Keep the existing dynamics and controller.** Reason: the first-order-lag model is the handoff's torque model in another form (`T_v = m/d_u`), and fish share it. Rejected: a torque/mass rewrite, which adds constants without changing motion.
- **Keep the fixed `r_max` cap.** Reason: with `v_min > 0`, it already prevents pivoting. The radius `v/r_max` (0.21 L at `v_min`, 0.42 L at cruise, 1.0 L at `v_max`) suits a hammerhead's tight slow turns, and the safety filter assumes `R = v/r_max`. Rejected: a speed-dependent `|r| ≤ u/R_min`, which adds a mechanism with no clear gain.
- **`kin.pos` is the shark's nose.** Reason: the spine then follows exactly the trajectory the dynamics produced, and a predator eats with its mouth. Rejected: a derived nose point ahead of the centre, which makes the drawn body drift off `kin.pos` in turns.
- **The drawn nose sways around `kin.pos`.** Reason: amp(0) = 0.2 gives ±0.06 m of head yaw, which is part of the blog's look and irrelevant to eating at 0.3 m.
- **Shark only.** Reason: 4000 fish at 10 px would not show the detail, and one entity keeps the scope small. Fish animation can come later as separate work.
- **Scalloped hammerhead, 3.0 m, with 0.2 m fish as prey.** Reason: scalloped hammerheads hunt mid-water schooling fish of about 0.15–0.4 m, which matches the boids. Rejected: a great hammerhead (eats rays, a solitary bottom hunter) and a bonnethead (fits the old scale but eats crustaceans).
- **`METERS_PER_PIXEL` = 0.02 and 4000 fish.** Reason: 150 px is enough for the wave, the cephalofoil and the fins to read, while fish stay visible at 10 px. Four times the fish keeps the density of the old world. The user has run 16000 fish without problems.
- **New shark speeds and response times.** Reason: about 1 m/s cruise is typical for a large shark (from memory, unverified). A hungry shark outruns fish in a straight line, and fish escape by out-turning it. `T_r` = 0.4 s suits a 3 m body; 0.1 s would look snappy. These are starting values for tuning.
- **Keep the variable, clamped timestep.** Reason: the spine is sampled from the recorded path and the phase is integrated, so nothing needs a fixed step.
- **Integrate the phase, not ω·t.** Reason: the tail-beat frequency then follows speed changes without jumps.
- **Wave parameters from the blog post.** Reason: a sourced, published starting point for a shark with similar kinematics.
- **Length-restoring pass after the wave.** Reason: without it, the body stretches as the wave swings sideways. One pass makes the lengths exact. More passes only if a test shows a need.
- **Caudal fin as the outline taper.** Reason: from above, the caudal fin is edge-on, and it lies within L, since L is total length. Rejected: a separate swept caudal polygon, which is not anatomical and extends past L.
- **No dorsal fins and no pectoral tilt.** Reason: dorsal fins are edge-on from above, and tilting the pectorals is tuning before the basics work.
- **Two pure seams (body update and body geometry).** Reason: tests run without a window or raylib's drawing state. Splitting motion from geometry keeps each test focused.
- **doctest, vendored.** Reason: readable float comparisons and per-test reporting, with no install. Rejected: plain `assert` (crude failures) and Catch2 (needs an install).

## Testing approach
- **Seams**
  - *Body update*: takes the previous body state, `Kinematics` and dt, and returns the new body state (nose path, phase, spine nodes after the wave and the length pass). Catches path placement, trimming, seeding, phase, amplitude and segment lengths. Misses appearance.
  - *Body geometry*: takes the spine nodes and returns the outline and rigid parts in world meters. Catches widths, attachment positions and orientation. Misses whether it looks like a hammerhead.
- **Unit tests (doctest)**
  - R5: drive the nose along a known path and check that the retained path length is at least L and that old points are trimmed.
  - R6: one update after spawn gives a straight spine along the initial heading.
  - R7: with zero amplitude, drive straight and circular nose paths and check the node distances to the path and their arc lengths.
  - R8 and R11: straight swimming at cruise, checking the tail tip's peak lateral offset over a beat cycle.
  - R9: one update with a known u and dt.
  - R10: segment lengths after updates during straight swimming and turns.
  - R12: the same path driven with uneven dt values keeps R7 and R10.
  - R13–R16: spine nodes in a known shape (straight, arc), checking outline offsets and part positions and orientations.
- **Build check**: R1–R4 and R18. The project builds, and the constants have the stated values.
- **Visual check by the user**: R17, how the wave looks, bending in turns, and the shape values against a photo.

## Out of scope
- Fish animation. Fish keep their triangles.
- Turn amplitude boost, pectoral tilt with turning, and dorsal fins.
- Any change to the dynamics model, the controller, the safety filter or the steering behaviours, beyond the shark constants in R3.
- Fixed-timestep simulation.
- Shading, textures or colour variation.
- Feedback from the animation into the dynamics.

## Deferred decisions
- Final shape values. Resolve after: the body is drawn and the user has compared it with a top-view photo. Issue: 06
- Wall buffer for the shark (`WALL_BUFFER_DIST` = 1 m, shared with fish). Resolve after: the shark is seen swimming at the new speeds. Change it only if the shark hits walls. Issue: 06
