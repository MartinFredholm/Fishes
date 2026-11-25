# Procedural swimming animation for the hammerhead shark

## Problem
The shark is drawn as a rigid red triangle centred on `kin.pos`. It does not look like a shark, and it does not swim: nothing moves along its body, and it does not bend in turns. It is also 0.4 m long, which is unrealistic for a hammerhead and too small on screen to show any body motion.

## Goal
The shark is a 3 m scalloped hammerhead that swims like the dogfish in the Julia blog post (https://julialang.org/blog/2021/08/sharks/): a travelling body wave, small at the head and large at the tail, whose frequency follows speed. In turns, the shark turns about its pivot: the head swings into the turn at once, and the body behind follows the pivot path without skidding. The existing dynamics stay the source of truth. The animation reads them and never feeds back.

## Requirements

### World and shark parameters
- **R1**: `METERS_PER_PIXEL` is 0.02, so the 1500 × 800 px window shows a 30 × 16 m world.
- **R2**: `N_FISH` is 4000. Fish length, width and motion parameters are unchanged.
- **R3**: Shark parameters: L = 3.0 m, `v_min` 0.5 m/s, `v_cruise` 1.0 m/s, `v_max` 2.5 m/s, `r_max` 0.8 rad/s, `T_v` 1.0 s, `T_r` 0.4 s, sight range 6 m, comfort range 4 m, eat distance 0.3 m. `K_phi`, `tau`, hunger rate, eat duration and the 45° eating-angle rule are unchanged.
- **R4**: For the shark, `kin.pos` is the pivot, at s_p = 1/3. Targeting and eating measure from the mouth point. Fish fleeing, shark–shark separation, wall repulsion and the safety filter measure from `kin.pos`. Apart from the point measured from, that code is unchanged. For fish, `kin.pos` is still the body centre.

### Body update (seam 1)
- **R5**: _(removed in rev 1)_ The body state recorded the path the nose had traced.
- **R6**: At spawn, the pivot path is a straight line behind `kin.pos` along `kin.heading`, at least (1 − s_p)·L long. The first frame is otherwise built like any other frame: the head from the current `kin`, the phase at 0 and the body wave on. In the first update after spawn, at v = 1.0 m/s, r = 0 and dt = 1/60 s, no node moves more than 0.05 m.
- **R7**: The spine has 24 nodes at uniform s_i = i/23, i = 0…23. With the wave amplitude set to zero:
  - each node with s_i ≥ s_p lies within 1 mm of the pivot path, at an arc length behind `kin.pos` within 1 mm of (s_i − s_p)·L;
  - each node with s_i < s_p lies within 1 mm of the head arc, at an arc length ahead of `kin.pos` within 1 mm of (s_p − s_i)·L. The head arc starts at `kin.pos`, is tangent to `kin.heading` there, runs forward, and has curvature r/v along its whole length. It is a straight line when r = 0.
- **R8**: The body wave offsets each node along the spine's local normal by h(s) = A·L·amp(s)·sin(k·s − φ), with A = 0.1, k = 5.3 and amp(s) interpolated smoothly through [0.2, 0.21, 0.23, 0.4, 0.88, 1.0] at s = 0, 0.2, 0.4, 0.6, 0.8 and 1.0.
- **R9**: Each update advances the phase by ω·dt, with ω = 2π·St·u/(2·A·L) and St = 0.3. With the decided values, that is ω = π·u rad/s. After one update, the phase differs from the expected value by at most 1e-6 rad.
- **R10**: After the wave, one head-to-tail pass resets each segment to its rest length L/23. The number of passes is a named constant. After every update, every segment is within 0.1 % of L/23.
- **R11**: During straight swimming at `v_cruise`, once the body has swum at least one body length, the tail tip's peak lateral offset from the straight track of `kin.pos` over one beat cycle is within ±20 % of A·L·amp(1) = 0.30 m.
- **R12**: The body update works with the existing variable, clamped timestep (dt ≤ 1/30 s). It does not assume a constant dt.
- **R19**: _(removed in rev 2)_ The spine depended only on the current `kin` and the phase, not on earlier positions.
- **R20**: When v = 0, the head arc is straight: with the wave amplitude set to zero, every node with s_i < s_p lies within 1 mm of the straight line ahead of `kin.pos` along `kin.heading`, at (s_p − s_i)·L. An update with v = 0 and `kin.pos` unchanged leaves the nodes behind the pivot within 1 mm of where they were. For every v ≥ 0 and |r| ≤ r_max, every spine coordinate is finite.
- **R21**: The body state records the pivot path. After every update, it covers at least (1 − s_p)·L of track behind `kin.pos`, and it keeps no more track than that requires: without its oldest point, it would cover less than (1 − s_p)·L. Its size stays bounded on long swims.
- **R22**: Two body states with the same pivot path and phase but different heads, updated with the same `kin` and dt, give spines that agree within 0.1 mm. A change in r therefore reshapes the head in the same update.

### Mouth point (seam 3)
- **R23**: The mouth point lies on the head arc, s_p·L ahead of `kin.pos` along it, within 1 mm. It is computed from `kin` and the body parameters alone. With the wave amplitude set to zero, it coincides with spine node 0 within 1 mm. At v = 0 it lies s_p·L straight ahead along `kin.heading`, and it is finite for every v ≥ 0 and |r| ≤ r_max.

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
| Pivot | s_p = 1/3 |

### Drawing
- **R17**: The shark is drawn as a single grey-bronze fill: the outline as a filled strip, plus the rigid parts. It is not red and has no shading. Fish drawing is unchanged.

### Tests
- **R18**: The shark's body code is in its own source file. A test executable using doctest (vendored under `third_party/`) runs through CTest and covers R6–R16 and R20–R23. The main executable and the existing build still work.

## Design decisions
- **Keep the existing dynamics and controller.** Reason: the first-order-lag model is the handoff's torque model in another form (`T_v = m/d_u`), and fish share it. Rejected: a torque/mass rewrite, which adds constants without changing motion.
- **Keep the fixed `r_max` cap.** Reason: with `v_min > 0`, it already prevents pivoting in place. The radius `v/r_max` (0.21 L at `v_min`, 0.42 L at cruise, 1.0 L at `v_max`) suits a hammerhead's tight slow turns, and the safety filter assumes `R = v/r_max`. Rejected: a speed-dependent `|r| ≤ u/R_min`, which adds a mechanism with no clear gain.
- **Animation only: the body shows the turn, it does not cause it.** Reason: the dynamics, the controller, the safety filter and the steering stay unchanged, and fish behaviour is unaffected. Rejected: a body bend that drives the turn in the dynamics. A real shark turns by bending, but that is a new dynamics model and would change predator–prey balance. It is a separate project if the animation still looks head-controlled.
- **No skid behind the pivot.** Reason: a shark cuts through the water; its body does not slide sideways. If no node moves across its own spine direction, the body must lie on the track of the point ahead of it, so the body behind the pivot follows the pivot path. Rejected: a bend set from the current r/v for the whole body at once (rev 1), which whipped the tail by up to 2 m when r changed, with nodes sliding sideways at up to 18 m/s. Also rejected: a stored bend angle that builds with r and drains as the body swims, which still slid nodes sideways at about 0.9 m/s at cruise.
- **`kin.pos` is a pivot at s_p = 1/3, not the nose.** Reason: a fish turns about a point near its centre of mass, with the head swinging into the turn ahead of it and the body following behind. s_p = 1/3 puts the pivot just behind the pectoral fin roots. It is a starting value from memory, unverified, and a named constant for tuning. Rejected: the nose as `kin.pos` with the body on the nose path (issues 01–03), which makes the head steer and the body trail; and the nose as `kin.pos` with an added head yaw, which keeps R4 unchanged but looks less real.
- **The head lies on the current turn circle.** Reason: it swings into the turn as soon as r builds, needs no stored state, and in a steady turn lies on the path the pivot is about to swim. At r_max it yaws about 46° at cruise and 92° at `v_min`. Rejected: a head along the path predicted from the commanded r, which leads earlier but needs the controller's commands in the body update; and a straight head, which shows no lead.
- **The mouth point is computed from `kin`, on the head arc.** Reason: it matches the drawn nose up to the body wave's ±0.06 m sway, and the animation does not feed back into the dynamics. Rejected: the drawn spine node 0, which is feedback from the animation; and a point straight ahead along `kin.heading`, which is up to 0.45 m off the drawn nose in tight turns, more than the eat distance.
- **Targeting and eating measure from the mouth point; everything else from the pivot.** Reason: a shark chases and bites with its mouth. Fleeing, separation, walls and the safety filter want the body's position, as for fish.
- **One curvature r/v along the head.** Reason: the head arc is short and the steady-turn shape is right. No new shape values are needed.
- **No cap on the turn bend.** Reason: the controller keeps v ≥ v_min and |r| ≤ r_max, so the curvature stays at most 1.6 1/m. Whether tight turns need a cap is deferred until the pivot version is seen.
- **Zero speed gives a straight head and a still body.** Reason: r/v is undefined at v = 0, and a pivot that does not move adds nothing to its path. It cannot happen in the running app, and it is harmless where tests use odd inputs.
- **The body wave stays symmetric on top of the turn bend.** Reason: the tail then beats about a centreline already curved into the turn. A turn amplitude boost adds a mechanism before its need is shown.
- **Spawn lays a straight pivot path, with the wave on.** Reason: sharks spawn with r = 0, where a straight body is the steady state. With the wave on from the first frame, the second frame does not jump, as the old straight, unwaved first frame did by 0.3 m.
- **The drawn nose sways around the mouth point.** Reason: amp(0) = 0.2 gives ±0.06 m of head yaw, which is part of the blog's look and irrelevant to eating at 0.3 m.
- **Shark only.** Reason: 4000 fish at 10 px would not show the detail, and one entity keeps the scope small. Fish animation can come later as separate work.
- **Scalloped hammerhead, 3.0 m, with 0.2 m fish as prey.** Reason: scalloped hammerheads hunt mid-water schooling fish of about 0.15–0.4 m, which matches the boids. Rejected: a great hammerhead (eats rays, a solitary bottom hunter) and a bonnethead (fits the old scale but eats crustaceans).
- **`METERS_PER_PIXEL` = 0.02 and 4000 fish.** Reason: 150 px is enough for the wave, the cephalofoil and the fins to read, while fish stay visible at 10 px. Four times the fish keeps the density of the old world. The user has run 16000 fish without problems.
- **New shark speeds and response times.** Reason: about 1 m/s cruise is typical for a large shark (from memory, unverified). A hungry shark outruns fish in a straight line, and fish escape by out-turning it. `T_r` = 0.4 s suits a 3 m body; 0.1 s would look snappy. These are starting values for tuning.
- **Keep the variable, clamped timestep.** Reason: the pivot path records positions, not time steps, the head is built from the current state, and the phase is integrated, so nothing needs a fixed step.
- **Integrate the phase, not ω·t.** Reason: the tail-beat frequency then follows speed changes without jumps.
- **Wave parameters from the blog post.** Reason: a sourced, published starting point for a shark with similar kinematics.
- **Length-restoring pass after the wave.** Reason: without it, the body stretches as the wave swings sideways. One pass makes the lengths exact. More passes only if a test shows a need.
- **Caudal fin as the outline taper.** Reason: from above, the caudal fin is edge-on, and it lies within L, since L is total length. Rejected: a separate swept caudal polygon, which is not anatomical and extends past L.
- **No dorsal fins and no pectoral tilt.** Reason: dorsal fins are edge-on from above, and tilting the pectorals is tuning before the basics work.
- **Three pure seams (body update, body geometry, mouth point).** Reason: tests run without a window or raylib's drawing state. Splitting motion from geometry keeps each test focused. The mouth point is its own seam because targeting needs it without a body state.
- **doctest, vendored.** Reason: readable float comparisons and per-test reporting, with no install. Rejected: plain `assert` (crude failures) and Catch2 (needs an install).

## Testing approach
- **Seams**
  - *Body update*: takes the previous body state (phase and pivot path), `Kinematics` and dt, and returns the new body state (phase, pivot path, spine nodes after the turn bend, the wave and the length pass). Catches the body behind the pivot on the pivot path, the head on the head arc, the pivot anchor, spawn, zero speed, the path record's bounds, phase, amplitude and segment lengths. Misses whether a turn looks natural.
  - *Body geometry*: takes the spine nodes and returns the outline and rigid parts in world meters. Catches widths, attachment positions and orientation. Misses whether it looks like a hammerhead.
  - *Mouth point*: takes `Kinematics` and the body parameters and returns the mouth point. Catches its position when straight, turning both ways and at v = 0. Misses whether targeting and eating use it.
- **Unit tests (doctest)**
  - R6: spawn gives a straight pivot path, and the first update at cruise moves no node more than 0.05 m.
  - R7: with zero amplitude, swim the pivot along a known track (straight, circles of both signs including r_max at v_min and v_max, and varying r and v), and check the nodes behind the pivot against that track and the head nodes against the head arc. The track and the arc are computed independently in the test.
  - R8 and R11: straight swimming at cruise, checking the tail tip's peak lateral offset over a beat cycle.
  - R9: one update with a known u and dt.
  - R10: segment lengths after updates during straight swimming and turns.
  - R12: uneven dt values, including 0, while turning keep R7, R9 and R10.
  - R20: v = 0 gives a straight head and a still body; v from 0 up to v_min with r = ±r_max gives finite coordinates.
  - R21: path coverage and trimming over long swims, straight and turning.
  - R22: two bodies with the same pivot path and phase but different r on their last update, given the same `kin`, give the same spine.
  - R23: mouth point against the head arc computed independently, straight, both turn signs and v = 0; coincides with node 0 at zero amplitude.
  - R13–R16: spine nodes in a known shape (straight, arc), checking outline offsets and part positions and orientations.
- **Build check and code inspection**: R1–R4 and R18. The project builds, the constants have the stated values, and targeting and eating use the mouth point while the rest uses `kin.pos`.
- **Visual check by the user**: R17, how the wave looks, the head swinging into turns and the body following without skidding, and the shape values against a photo.

## Out of scope
- Fish animation. Fish keep their triangles.
- Turn amplitude boost, pectoral tilt with turning, and dorsal fins.
- Any change to the dynamics model, the controller, the safety filter or the steering behaviours, beyond the shark constants in R3 and the measuring points in R4.
- A body bend that drives the turn in the dynamics.
- Fixed-timestep simulation.
- Shading, textures or colour variation.
- Feedback from the animation into the dynamics.

## Deferred decisions
- Shape of the turn in motion: the sharp bend at turn onset (r rises over T_r, so the pivot path goes from straight to circle within 0.2–1 m) as it travels down the body; the tight curl at v_min and r_max (about 180° behind the pivot plus 92° of head); the head's yaw size; a bend cap; the 14° kink near the tail caused by the length pass on a tightly bent body; and the value of s_p. Resolve after: the pivot version is seen in motion. Issue: 09
- Final shape values. Resolve after: the body is drawn and the user has compared it with a top-view photo. Issue: 09
- Wall buffer for the shark (`WALL_BUFFER_DIST` = 1 m, shared with fish). The head now reaches s_p·L = 1 m past `kin.pos`, so the nose may cross the buffer. Resolve after: the shark is seen swimming at the new speeds. Change it only if the shark hits walls. Issue: 09

## Revisions
- Rev 1: The spine is shaped by the turn bend (curvature r/v from the current `kin`, whole body at once) instead of being laid along the recorded nose path. R5 removed; R6, R7, R11 and R18 changed; R19 and R20 added; design decisions, testing approach and deferred decisions updated. Reason: following the nose path delayed the bend by one body length, so in turns the tail beat about the old line and the body looked kinked. A shark bends its whole body into a turn.
- Rev 2: `kin.pos` is a pivot at s_p = 1/3. The body behind it follows the pivot path, and the head ahead of it lies on the current turn circle. Targeting and eating measure from a mouth point on the head arc. R4, R6, R7, R11, R18 and R20 changed; R19 removed; R21 (pivot path record), R22 (head reacts at once) and R23 (mouth point) added; mouth point seam added; goal, design decisions, testing approach and deferred decisions updated. Reason: rev 1's whole-body bend whipped the tail when r changed, which looks like skidding. A shark does not skid, and only a body that follows a path avoids it. Turning about a pivot with the head leading looks more real than the nose path, where the head steers and the body trails.
