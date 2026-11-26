# Front stiffness and turn asymmetry

## Problem
In a turn the whole spine lies on the nose path, so the head and front body bend as much as the curve the nose swam. A real shark's front is stiff. When the nose has finished a turn and the tail is still passing through the curve, the body wave beats the same amount to both sides, and the tail looks limp instead of driving through the curve.

## Goal
In a turn the drawn front, s = 0 to 0.35, bends progressively less toward the nose, and the head stays still. Where the body passes through a curve, the body wave beats harder on the outside of the curve and less on the inside. The dynamics, the nose path and the body line are not affected. Whether it looks right is intent, judged by the user by eye in the running app.

## Requirements
- **R1**: With zero amplitude, spine nodes at s ≥ 0.35 lie on the nose path at s·L behind the nose, with the existing tolerances, under the existing conditions: tightest turn with trimming, varying speed and curvature with uneven dt, and a one-hour swim.
- **R2**: With zero amplitude, in a steady turn of radius 1.25 m at cruise (1 m/s) and of radius 0.625 m at `v_min` (0.5 m/s), in both turn directions:
  - each front node (0 < s < 0.35) bends less than the nose path does over one segment;
  - the bend per node does not decrease from the nose toward s = 0.35;
  - the last front node bends at least 90 % of the nose path's bend per segment.
- **R3**: Under the conditions of R2, the direction from node 0 to node 1 lags `kin.heading` toward the outside of the turn by κ·L·0.35/2, within ±15 %: 0.42 rad at radius 1.25 m and 0.84 rad at radius 0.625 m. κ is the turn's curvature in 1/m.
- **R4**: When swimming straight with zero amplitude, node 0 is `kin.pos` every frame (within 1e-5 m), and all nodes lie on the straight path at s·L. The existing straight-swimming body wave tests pass unchanged.
- **R5**: Every spine segment is within 0.1 % of L/23 after every update, in turns and when swimming straight, with the normal amplitude. With zero amplitude there is no length pass, and R1 places the nodes at s ≥ 0.35 by arc length, so their chords are up to 0.2 % short in the tightest turn, as today.
- **R6**: Turn asymmetry: with length passes set to 0, on steady circles of curvature 0, 0.4, 0.8 and 1.6 /m (a = 0, 0.25, 0.5 and 0.5), in both turn directions, at several phases, each node's offset along the spine normal from the spine built from the same nose path with zero amplitude (which includes front stiffness) equals (1 + a)·h(s) when it lies on the outside of the curve and (1 − a)·h(s) when it lies on the inside, within 1 mm.
- **R7**: Swimming straight at cruise with uneven dt (all ≤ 1/30 s), the tail tip's peak offset over a beat to the left and to the right are equal within 2 %.
- **R8**: In the tightest turn (radius 0.625 m at `v_min`), in both directions, the worst bend between neighbouring spine segments is at most the nose path's bend per segment plus (1 + a_max)·0.3 rad = 0.45 rad, every frame.
- **R9**: The body line is unchanged: it stays on the nose path at uniform s, with no front stiffness and no turn asymmetry. The existing body line tests pass unchanged.
- **R10**: The existing body geometry tests (outline and rigid parts) pass unchanged, on spines from the new body update.

## Design decisions
- Front stiffness acts on the bend from the turn, not on the body wave. Reason: the head bend you see comes from path-following; the wave adds only ±0.06 m at the head. Rejected: damping the wave at the front, because the envelope comes from the Julia shark data and sharks do sway their heads.
- Front stiffness and turn asymmetry act only on the drawn spine. Reason: the animation never feeds back into the dynamics; the body line, steering, shark avoidance and contact keep using the nose path.
- Stiffness ramps along the front: fully rigid at the nose, fully flexible at s = 0.35, as a smoothstep (3t² − 2t³) with t = s/0.35. The front bends by the nose path's local curvature times this ramp. Reason: the user wants the head still and the body more flexible further back; smoothstep is the simplest curve flat at both ends, so there is no kink at s = 0.35. Rejected: a rigid front (visible kink at s = 0.35), and a linear ramp (a corner in the curvature at s = 0.35).
- The front is anchored at s = 0.35 and built forward from the spine's direction there. Reason: the body from s = 0.35 back stays on the nose path and close to the body line; the drawn nose sits outside `kin.pos` in a turn by 13 cm at cruise and 25 cm in the tightest turn (measured, zero amplitude). Rejected: anchoring at the nose, which swings the whole rear body about 0.75 m outside the nose path at cruise.
- Front stiffness applies at any amplitude, including zero. Reason: it is part of path placement, not of the body wave.
- Turn asymmetry follows the nose path's local curvature at each node's arc, not `kin.r`. Reason: the user sees the problem when the nose has finished turning and the tail is still in the curve; `kin.r` is then near zero. Rejected: `kin.r`, which acts at the start of a turn, before the tail reaches the curve.
- Asymmetry scales each half-stroke: (1 + a) on the outside of the curve, (1 − a) on the inside. a rises linearly with curvature to a_max = 0.5 at 0.8 /m, and is held there for tighter curves. Reason: simplest form; frequency and phase are unchanged. The tail's mean position moves outward by 2a/π of the local amplitude (about 10 cm at the tail tip at a = 0.5), which the user accepted, as it straightens the tail in the curve. Capping a below 1 keeps the inside stroke from reversing. 0.8 /m is a full-rate turn at cruise (r_max / v_cruise).
- The stiff length (0.35, raised from 0.25 at the user's request for a stiffer front), a_max (0.5) and the full-asymmetry curvature (0.8 /m) are body parameters, alongside the wave parameters. Reason: they are starting values to tune by eye. Rejected: off-switches to keep the old tests unchanged; the tests are rewritten to their intent instead.

## Testing approach
- Seams: **body update** catches everything in R1–R8: the spine from a scripted nose path. It misses how the result looks. **Body geometry** is a regression check only (R10): the existing outline and rigid-part tests already run on swum, turning spines.
- R1: the existing on-path tests in turns (tightest turn, circle, varying speed and curvature, one hour), restricted to s ≥ 0.35.
- R2, R3: new tests on steady scripted circles, zero amplitude, both directions.
- R4: the existing "node 0 is the nose every frame" test, changed to straight swimming; the existing straight-path test unchanged.
- R5: the existing segment-length tests, unchanged.
- R6: new test with length passes 0, like the existing "six amplitude points" test.
- R7: new test, straight with uneven dt.
- R8: the existing "wave adds at most 0.3 rad" test with the allowance derived from a_max.
- R9, R10: existing tests, unchanged.
- The look (head stiffness, tail in a curve) is judged by the user in the running app.

## Out of scope
- Damping the body wave at the front.
- Any change to the dynamics, steering, shark avoidance, the nose path or the body line.
- Asymmetry driven by `kin.r`.
- Changes to the width profile, outline or rigid-part shapes.
- Fish.
