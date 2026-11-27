# Front stiffness and turn asymmetry

## Problem
In a turn the whole spine lies on the nose path, so the head and front body bend as much as the curve the nose swam. A real shark's front is stiff. When the nose has finished a turn and the tail is still passing through the curve, the body wave beats the same amount to both sides, and the tail looks limp instead of driving through the curve.

## Goal
In a turn the drawn front, s = 0 to 0.35, bends progressively less toward the nose, and the head stays still. While the body lies in a turn, the body wave beats harder on the outside of the turn and less on the inside, as one effect along the whole body. It builds up and fades smoothly, with no jumps from frame to frame, and it is gone once the tail tip has left the turn. The dynamics, the nose path and the body line are not affected. Whether it looks right is intent, judged by the user by eye in the running app.

## Requirements
- **R1**: With zero amplitude, spine nodes at s ≥ 0.35 lie on the nose path at s·L behind the nose, with the existing tolerances, under the existing conditions: tightest turn with trimming, varying speed and curvature with uneven dt, and a one-hour swim.
- **R2**: With zero amplitude, in a steady turn of radius 1.25 m at cruise (1 m/s) and of radius 0.625 m at `v_min` (0.5 m/s), in both turn directions:
  - each front node (0 < s < 0.35) bends less than the nose path does over one segment;
  - the bend per node does not decrease from the nose toward s = 0.35;
  - the last front node bends at least 90 % of the nose path's bend per segment.
- **R3**: Under the conditions of R2, the direction from node 0 to node 1 lags `kin.heading` toward the outside of the turn by κ·L·0.35/2, within ±15 %: 0.42 rad at radius 1.25 m and 0.84 rad at radius 0.625 m. κ is the turn's curvature in 1/m.
- **R4**: When swimming straight with zero amplitude, node 0 is `kin.pos` every frame (within 1e-5 m), and all nodes lie on the straight path at s·L. The existing straight-swimming body wave tests pass unchanged.
- **R5**: Every spine segment is within 0.1 % of L/23 after every update, in turns and when swimming straight, with the normal amplitude. With zero amplitude there is no length pass, and R1 places the nodes at s ≥ 0.35 by arc length, so their chords are up to 0.2 % short in the tightest turn, as today.
- **R6**: Turn asymmetry on steady circles: with length passes set to 0, with the whole nose path behind the nose on a circle of curvature 0, 0.4, 0.8 and 1.6 /m (body turn equal to that curvature, so a = 0, 0.25, 0.5 and 0.5), in both turn directions, at several phases, each node's offset along the spine normal from the spine built from the same nose path with zero amplitude (which includes front stiffness) equals h(s) plus a·A·L·amp(s)·(2/π − 4/(3π)·cos 2θ) toward the outside of the turn, with θ = k·s − phase, within 1 mm.
- **R7**: Swimming straight at cruise with uneven dt (all ≤ 1/30 s), the tail tip's peak offset over a beat to the left and to the right are equal within 2 %.
- **R8**: In the tightest turn (radius 0.625 m at `v_min`), in both directions, the worst bend between neighbouring spine segments is at most the nose path's bend per segment plus (1 + a_max)·0.3 rad = 0.45 rad, every frame.
- **R9**: The body line is unchanged: it stays on the nose path at uniform s, with no front stiffness and no turn asymmetry. The existing body line tests pass unchanged.
- **R10**: The existing body geometry tests (outline and rigid parts) pass unchanged, on spines from the new body update.
- **R11**: The body turn builds up and fades with the body's passage through a turn: with length passes set to 0, on a scripted nose path of a straight, a 90° arc of radius 1.25 m at 1 m/s, and a straight, in both turn directions, at several phases, sampled with the nose 1.0 m into the arc, 1.0 m past its end, 2.5 m past its end, and more than L past its end: each node's offset is as in R6, with a from the body turn computed from the scripted path's geometry (its heading change over the last L of path, divided by L), within 1 mm. More than L past the end, a = 0.
- **R12**: No jumps from frame to frame: at dt = 1/60 s with the normal amplitude, on scripted swims of 1 L straight, a full-rate turn of 180° and 2 L straight, at cruise (1 m/s) and in the tightest turn (`v_min`), and an S-turn of 90° one way and 90° the other at cruise, each in both directions: for every node and frame, the size of the node's second difference in time, |p(t + dt) − 2·p(t) + p(t − dt)|, exceeds that of the same swim with `asymmetry_max` = 0 by at most 2 mm.

## Design decisions
- Front stiffness acts on the bend from the turn, not on the body wave. Reason: the head bend you see comes from path-following; the wave adds only ±0.06 m at the head. Rejected: damping the wave at the front, because the envelope comes from the Julia shark data and sharks do sway their heads.
- Front stiffness and turn asymmetry act only on the drawn spine. Reason: the animation never feeds back into the dynamics; the body line, steering, shark avoidance and contact keep using the nose path.
- Stiffness ramps along the front: fully rigid at the nose, fully flexible at s = 0.35, as a smoothstep (3t² − 2t³) with t = s/0.35. The front bends by the nose path's local curvature times this ramp. Reason: the user wants the head still and the body more flexible further back; smoothstep is the simplest curve flat at both ends, so there is no kink at s = 0.35. Rejected: a rigid front (visible kink at s = 0.35), and a linear ramp (a corner in the curvature at s = 0.35).
- The front is anchored at s = 0.35 and built forward from the spine's direction there. Reason: the body from s = 0.35 back stays on the nose path and close to the body line; the drawn nose sits outside `kin.pos` in a turn by 13 cm at cruise and 25 cm in the tightest turn (measured, zero amplitude). Rejected: anchoring at the nose, which swings the whole rear body about 0.75 m outside the nose path at cruise.
- Front stiffness applies at any amplitude, including zero. Reason: it is part of path placement, not of the body wave.
- Turn asymmetry uses one value of a for the whole body, from the body turn: the nose path's heading change over the last body length swum, divided by L. Reason: a routine shark turn is one whole-body bend that builds up during the turn and relaxes after it (Hoffmann & Porter 2019, bonnethead sharks), and the user wants the change to carry through the whole tail. The body turn ends when the tail tip leaves the curve, which is when the user expects the tail to stop driving. It reads the existing nose path, so it needs no new state and does not depend on dt. Rejected: each node's local path curvature, because neighbouring nodes disagree where a turn starts and the tail kinks (0.85 rad against R8's 0.659 rad, measured); `kin.r`, which acts at the start of a turn, before the tail reaches the curve; an exponential lag of `kin.r` with time constant L/v, because the tail would keep driving for 3–6 s after the body is straight, and it needs new state.
- Each node is offset toward the outside of the turn by a·A·L·amp(s)·(2/π − 4/(3π)·cos 2θ), on top of h(s). This is the two-term Fourier form of a·|h(s)| over a beat. Reason: the user wants uneven strokes, larger on the outside and smaller on the inside, without jumps. The outside peak grows by about 1.06a of the local amplitude and the inside peak shrinks by the same. The mean position moves outward by 2a/π of the local amplitude (about 10 cm at the tail tip at a = 0.5), which the user accepted, as it straightens the tail in the curve. The form is smooth in time and along the body; frequency and phase are unchanged. Rejected: scaling each half-stroke by (1 ± a), which switches at the centre line, so a node's sideways speed jumps by 2a times its stroke speed (tail spikes of 22 mm/frame against 4 mm/frame without asymmetry, measured in a headless copy of the app); a tanh-smoothed switch, which adds a free width and still has a steep zone; a shifted centre with a symmetric wave, which the user rejected as reading like a bent tail.
- a = a_max · min(|body turn| / 0.8 /m, 1), with a_max = 0.5. Reason: simplest mapping; capping a below 1 keeps the inside stroke from reversing. 0.8 /m is a full-rate turn at cruise (r_max / v_cruise). The cap is reached only when the whole body lies in a full-rate cruise turn, at least 137°; a 90° turn at cruise peaks at a body turn of 0.52 /m, a = 0.33. Starting values, tuned by eye.
- In an S-bend the body turn roughly cancels and the tail beats evenly. Accepted by the user.
- The stiff length (0.35, raised from 0.25 at the user's request for a stiffer front), a_max (0.5) and the full-asymmetry curvature (0.8 /m) are body parameters, alongside the wave parameters. Reason: they are starting values to tune by eye. Rejected: off-switches to keep the old tests unchanged; the tests are rewritten to their intent instead.

## Testing approach
- Seams: **body update** catches everything in R1–R8: the spine from a scripted nose path. It misses how the result looks. **Body geometry** is a regression check only (R10): the existing outline and rigid-part tests already run on swum, turning spines.
- R1: the existing on-path tests in turns (tightest turn, circle, varying speed and curvature, one hour), restricted to s ≥ 0.35.
- R2, R3: new tests on steady scripted circles, zero amplitude, both directions.
- R4: the existing "node 0 is the nose every frame" test, changed to straight swimming; the existing straight-path test unchanged.
- R5: the existing segment-length tests, unchanged.
- R6: new test with length passes 0, like the existing "six amplitude points" test.
- R11: new test on the scripted straight–arc–straight path, with length passes 0; the expected body turn comes from the scripted geometry.
- R12: new test on the scripted swims, each run twice, with `asymmetry_max` 0.5 and 0, compared node by node and frame by frame.
- R7: new test, straight with uneven dt.
- R8: the existing "wave adds at most 0.3 rad" test with the allowance derived from a_max.
- R9, R10: existing tests, unchanged.
- The look (head stiffness, tail in a curve) is judged by the user in the running app.

## Out of scope
- Damping the body wave at the front.
- Any change to the dynamics, steering, shark avoidance, the nose path or the body line.
- Asymmetry driven by `kin.r` or a time lag of it.
- A headless copy of the app as a build target. The jitter is a property of the body update and is checked there (R12).
- Changes to the width profile, outline or rigid-part shapes.
- Fish.

## Revisions
- Rev 1: Turn asymmetry redesigned. a is now one value for the whole body, from the body turn, instead of each node's local path curvature. The (1 ± a) scaling of each half-stroke is replaced by a smooth offset toward the outside, a·A·L·amp(s)·(2/π − 4/(3π)·cos 2θ). R6 rewritten; R11 and R12 added; R7, R8 unchanged. Reason: in the running app the (1 ± a) rule made the head, body and tail jump, and a headless copy of the app traced the jumps to the switch at the centre line (tail spikes of 22 mm/frame against 4 mm/frame). Per-node curvature also kinked the tail where a turn starts. Front stiffness is unchanged.
