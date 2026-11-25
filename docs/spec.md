# Procedural swimming animation for the hammerhead shark

## Problem
The shark is drawn as a rigid red triangle centred on `kin.pos`. It does not look like a shark, and it does not swim: nothing moves along its body, and it does not bend in turns. It is also 0.4 m long, which is unrealistic for a hammerhead and too small on screen to show any body motion.

## Goal
The shark is a 3 m scalloped hammerhead that swims like the dogfish in the Julia blog post (https://julialang.org/blog/2021/08/sharks/): a travelling body wave, small at the head and large at the tail, whose frequency follows speed. In turns, the whole body bends into the turn at once, as the turn bend. The existing dynamics stay the source of truth. The animation reads them and never feeds back.

## Requirements

### World and shark parameters
- **R1**: `METERS_PER_PIXEL` is 0.02, so the 1500 × 800 px window shows a 30 × 16 m world.
- **R2**: `N_FISH` is 4000. Fish length, width and motion parameters are unchanged.
- **R3**: Shark parameters: L = 3.0 m, `v_min` 0.5 m/s, `v_cruise` 1.0 m/s, `v_max` 2.5 m/s, `r_max` 0.8 rad/s, `T_v` 1.0 s, `T_r` 0.4 s, sight range 6 m, comfort range 4 m, eat distance 0.3 m. `K_phi`, `tau`, hunger rate, eat duration and the 45° eating-angle rule are unchanged.
- **R4**: For the shark, `kin.pos` is the nose. Eating, targeting, fish fleeing and shark–shark separation measure from it, using the existing code unchanged. For fish, `kin.pos` is still the body centre.

### Body update (seam 1)
- **R5**: _(removed in rev 1)_ The body state recorded the path the nose had traced.
- **R6**: There is no spawn special case. The first frame is built from `kin` like any other frame, with the phase at 0 and the body wave on. In the first update after spawn, at v = 1.0 m/s, r = 0 and dt = 1/60 s, no node moves more than 0.05 m.
- **R7**: The spine has 24 nodes at uniform s_i = i/23, i = 0…23. With the wave amplitude set to zero, node i lies within 1 mm of the turn bend's arc, at an arc length behind the nose within 1 mm of s_i·L. The turn bend's arc is the track the nose would have swum if the current r and v had been constant: it starts at `kin.pos`, is tangent to `kin.heading` there, and has curvature r/v along its whole length. It is a straight line when r = 0.
- **R8**: The body wave offsets each node along the spine's local normal by h(s) = A·L·amp(s)·sin(k·s − φ), with A = 0.1, k = 5.3 and amp(s) interpolated smoothly through [0.2, 0.21, 0.23, 0.4, 0.88, 1.0] at s = 0, 0.2, 0.4, 0.6, 0.8 and 1.0.
- **R9**: Each update advances the phase by ω·dt, with ω = 2π·St·u/(2·A·L) and St = 0.3. With the decided values, that is ω = π·u rad/s. After one update, the phase differs from the expected value by at most 1e-6 rad.
- **R10**: After the wave, one head-to-tail pass resets each segment to its rest length L/23. The number of passes is a named constant. After every update, every segment is within 0.1 % of L/23.
- **R11**: During straight swimming at `v_cruise`, once the body has swum at least one body length, the tail tip's peak lateral offset from the nose's straight track over one beat cycle is within ±20 % of A·L·amp(1) = 0.30 m.
- **R12**: The body update works with the existing variable, clamped timestep (dt ≤ 1/30 s). It does not assume a constant dt.
- **R19**: The spine depends only on the current `kin` (position, heading, r and v) and the phase, not on earlier positions. Two body states with different swimming histories and the same phase, updated with the same `kin` and dt, give spines that agree within 0.1 mm. A change in r therefore reshapes the whole spine in the same update.
- **R20**: When v = 0, the turn bend is zero: with the wave amplitude set to zero, every node lies within 1 mm of the straight line behind the nose along `kin.heading`, at s_i·L. For every v ≥ 0 and |r| ≤ r_max, every spine coordinate is finite.

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
- **R18**: The shark's body code is in its own source file. A test executable using doctest (vendored under `third_party/`) runs through CTest and covers R6–R16, R19 and R20. The main executable and the existing build still work.

## Design decisions
- **Keep the existing dynamics and controller.** Reason: the first-order-lag model is the handoff's torque model in another form (`T_v = m/d_u`), and fish share it. Rejected: a torque/mass rewrite, which adds constants without changing motion.
- **Keep the fixed `r_max` cap.** Reason: with `v_min > 0`, it already prevents pivoting. The radius `v/r_max` (0.21 L at `v_min`, 0.42 L at cruise, 1.0 L at `v_max`) suits a hammerhead's tight slow turns, and the safety filter assumes `R = v/r_max`. Rejected: a speed-dependent `|r| ≤ u/R_min`, which adds a mechanism with no clear gain.
- **`kin.pos` is the shark's nose, and the spine starts there along `kin.heading`.** Reason: a predator eats with its mouth, and the drawn body stays on the point the dynamics integrate. Rejected: a derived nose point ahead of the centre, and pivoting the body about s ≈ 0.3 with the head yawed into the turn. Both make the drawn nose drift off `kin.pos` in turns.
- **Animation only: the turn bend shows the turn, it does not cause it.** Reason: the dynamics, the controller and everything measured from `kin.pos` stay unchanged, and fish behaviour is unaffected. Rejected: a body that generates the turn, which is a new dynamics model.
- **The turn bend comes from the current r and v, applied to the whole body at once.** Reason: a shark bends its whole body into a turn. Following the recorded nose path delayed the bend by one body length (about 3 s at cruise), so the tail kept beating along the old line and the body looked kinked. In a steady turn, r/v gives the same arc as path following. Rejected: the recorded nose path (the lag above), and a bend that sweeps from head to tail using a short history of r (more realistic, but a further mechanism; deferred until the simpler bend has been seen).
- **One curvature r/v along the whole body.** Reason: steady turns keep the shape they had, and no new shape values are needed. Rejected for now: a bend concentrated behind the pectoral fins, which needs an unchecked shape table.
- **No cap on the turn bend.** Reason: the controller keeps v ≥ v_min and |r| ≤ r_max, so r/v ≤ 1.6 1/m and the body bends at most about 275°. That matches the tight slow turns chosen for a hammerhead.
- **Zero speed gives a straight spine.** Reason: r/v is undefined at v = 0. It cannot happen in the running app, and a straight body is harmless where tests use odd inputs.
- **The body wave stays symmetric on top of the turn bend.** Reason: the tail then beats about a centreline already curved into the turn. A turn amplitude boost adds a mechanism before its need is shown.
- **No spawn special case.** Reason: without a path there is nothing to seed. An ordinary first frame also removes the 0.3 m jump on the second frame that a straight first frame caused.
- **The drawn nose sways around `kin.pos`.** Reason: amp(0) = 0.2 gives ±0.06 m of head yaw, which is part of the blog's look and irrelevant to eating at 0.3 m.
- **Shark only.** Reason: 4000 fish at 10 px would not show the detail, and one entity keeps the scope small. Fish animation can come later as separate work.
- **Scalloped hammerhead, 3.0 m, with 0.2 m fish as prey.** Reason: scalloped hammerheads hunt mid-water schooling fish of about 0.15–0.4 m, which matches the boids. Rejected: a great hammerhead (eats rays, a solitary bottom hunter) and a bonnethead (fits the old scale but eats crustaceans).
- **`METERS_PER_PIXEL` = 0.02 and 4000 fish.** Reason: 150 px is enough for the wave, the cephalofoil and the fins to read, while fish stay visible at 10 px. Four times the fish keeps the density of the old world. The user has run 16000 fish without problems.
- **New shark speeds and response times.** Reason: about 1 m/s cruise is typical for a large shark (from memory, unverified). A hungry shark outruns fish in a straight line, and fish escape by out-turning it. `T_r` = 0.4 s suits a 3 m body; 0.1 s would look snappy. These are starting values for tuning.
- **Keep the variable, clamped timestep.** Reason: the spine is built from the current state and the phase is integrated, so nothing needs a fixed step.
- **Integrate the phase, not ω·t.** Reason: the tail-beat frequency then follows speed changes without jumps.
- **Wave parameters from the blog post.** Reason: a sourced, published starting point for a shark with similar kinematics.
- **Length-restoring pass after the wave.** Reason: without it, the body stretches as the wave swings sideways. One pass makes the lengths exact. More passes only if a test shows a need.
- **Caudal fin as the outline taper.** Reason: from above, the caudal fin is edge-on, and it lies within L, since L is total length. Rejected: a separate swept caudal polygon, which is not anatomical and extends past L.
- **No dorsal fins and no pectoral tilt.** Reason: dorsal fins are edge-on from above, and tilting the pectorals is tuning before the basics work.
- **Two pure seams (body update and body geometry).** Reason: tests run without a window or raylib's drawing state. Splitting motion from geometry keeps each test focused.
- **doctest, vendored.** Reason: readable float comparisons and per-test reporting, with no install. Rejected: plain `assert` (crude failures) and Catch2 (needs an install).

## Testing approach
- **Seams**
  - *Body update*: takes the previous body state, `Kinematics` and dt, and returns the new body state (phase, spine nodes after the turn bend, the wave and the length pass). The previous state matters only for the phase. Catches the turn bend's shape and sign, the nose anchor, the absence of lag along the body, the first frame, zero speed, phase, amplitude and segment lengths. Misses whether a turn looks natural.
  - *Body geometry*: takes the spine nodes and returns the outline and rigid parts in world meters. Catches widths, attachment positions and orientation. Misses whether it looks like a hammerhead.
- **Unit tests (doctest)**
  - R6: the first update after spawn at cruise moves no node more than 0.05 m.
  - R7: with zero amplitude, for r = 0 and for both signs of r, check the node distances to the turn bend's arc and their arc lengths behind the nose. The arc is computed independently, as the steady-turn track of the nose.
  - R19: two bodies, one swum straight and one swum through turns, given the same `kin` and phase, give the same spine.
  - R20: v = 0 gives a straight spine; v from 0 up to v_min with r = ±r_max gives finite coordinates.
  - R8 and R11: straight swimming at cruise, checking the tail tip's peak lateral offset over a beat cycle.
  - R9: one update with a known u and dt.
  - R10: segment lengths after updates during straight swimming and turns.
  - R12: uneven dt values keep R9 and R10.
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
- Shape of the turn bend in motion: a bend cap for tight turns, a bend concentrated along part of the body, a head-to-tail sweep of the bend, and the 14° kink near the tail at v_min and r_max (caused by the length pass on a tightly bent body). Resolve after: the turn bend is seen in motion. Issue: 07
- Final shape values. Resolve after: the body is drawn and the user has compared it with a top-view photo. Issue: 07
- Wall buffer for the shark (`WALL_BUFFER_DIST` = 1 m, shared with fish). Resolve after: the shark is seen swimming at the new speeds. Change it only if the shark hits walls. Issue: 07

## Revisions
- Rev 1: The spine is shaped by the turn bend (curvature r/v from the current `kin`, whole body at once) instead of being laid along the recorded nose path. R5 removed; R6, R7, R11 and R18 changed; R19 and R20 added; design decisions, testing approach and deferred decisions updated. Reason: following the nose path delayed the bend by one body length, so in turns the tail beat about the old line and the body looked kinked. A shark bends its whole body into a turn.
