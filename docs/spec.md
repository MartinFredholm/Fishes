# Body-aware separation, fleeing and shark avoidance

## Problem
Fish flee from a shark's nose only, and sharks keep their distance from each other's noses only. The shark is now a 3 m body behind its nose. Fish swim close past its trunk and tail, and two sharks can swim through each other's bodies.

With body-line separation alone (rev 0), 10 sharks still have about 21 shark contacts per minute. The push is too weak, and once a shark turns at `r_max` a stronger push cannot turn it faster. Retuning alone leaves about 4 per minute (measured with the contact probe, 2 seeds × 120 s).

## Goal
Fish flee from the whole shark, and each shark steers its nose away from the whole body of every other shark. Steering uses a body line taken from the nose path, so the animation still never feeds back into the dynamics. With 10 sharks, shark contact is rare: a retuned soft separation does most of the work, and a hard shark avoidance step catches what it misses. Eating and targeting are unchanged.

## Requirements

### Body line (seam 1: body update)
- **R1**: The body state holds a body line with a fixed number of points, set by a named constant with the value 5. Point i lies at s_i = i/(n − 1), so s = 0, 0.25, 0.5, 0.75 and 1.
- **R2**: Every body-line point lies within 1 mm of the nose path, at an arc length behind the nose within 1 mm of s_i·L. Point 0 is the nose.
- **R3**: At spawn, the body line is straight along the initial heading, from the nose to L behind it, with each point within 1 mm of its expected position.
- **R4**: The body line does not depend on the body wave. For the same nose path, the body line with the wave amplitude set to zero and with A = 0.1 differs by at most 1e-6 m per point.
- **R5**: R2 holds for straight and circular nose paths, and with uneven dt values up to 1/30 s.

### Nearest point (seam 2)
- **R6**: Given a position and a body line, the nearest-point function returns the point on the body line's segments that is closest to the position, within 1e-5 m of the analytical answer.
- **R7**: A position ahead of the nose gets the nose, a position behind the tail tip gets the tail tip, and a position beside a segment gets its perpendicular foot on that segment.
- **R8**: Where the body line bends, a position gets the nearest point over all segments, not only the nearest segment by its end points.

### Steering
- **R9**: Fish fleeing adds one term per shark, as today, with the nearest point on that shark's body line in place of its nose. `FISH_FLEE_RANGE` (1.8 m), the ramp 1 − d/`FISH_FLEE_RANGE`, the normalisation and `W_FLEE` are unchanged.
- **R10**: Shark separation adds one term per other shark, with the nearest point on the other shark's body line in place of its nose. The distance is from the shark's own nose. The ramp is 1 − d/`SHARK_COMFORT_RANGE`, and the term is not normalised. `SHARK_COMFORT_RANGE` starts at 6 m and the term is weighted by `W_SHARK_SEPARATION`, which starts at 4 (both may be retuned under R21).
- **R11**: Eating and targeting still measure from the nose. The wall safety filter is unchanged.
- **R12**: Visual check by the user: fish keep clear of the whole shark, tail included, not only ahead of the head. With 2 sharks and with 10 sharks, sharks no longer swim through each other's bodies in normal swimming.
- **R14**: The shared separation weight is split. Fish separation from other fish uses `W_FISH_SEPARATION` = 0.9, so fish behave exactly as before. Shark separation uses `W_SHARK_SEPARATION`.

### Shark avoidance (seam 3)
- **R15**: Each frame, each shark's commands pass through shark avoidance after the steering produces them and before the wall safety filter. The wall safety filter runs last and keeps the final say.
- **R16**: Shark avoidance predicts the shark's own nose over the horizon `SHARK_AVOID_HORIZON` (starts at 2.5 s). It steps the real dynamics (`integrate`) at a fixed dt of 1/30 s, holding the commands constant, from the shark's current kinematics. It predicts four candidates: the commanded commands, and three options at `v_min` with turn rate +`r_max` (left), 0 (straight) and −`r_max` (right). Positive turn rate turns screen-left of the swimming direction.
- **R17**: In the prediction, every other shark's body line moves rigidly along that shark's heading at its current speed. At time t ahead it is the current body line shifted by heading·v·t.
- **R18**: A candidate's shark clearance is the smallest distance, over all prediction steps and other sharks, from the predicted nose to the shifted body line, minus the contact distance (0.36 m). Its wall clearance is the smallest distance over all steps from the predicted nose to the nearest wall, minus the shark's wall clearance. Its score is the smaller of the two.
- **R19**: If the commanded candidate's shark clearance is at least `SHARK_AVOID_MARGIN` (starts at 0.2 m), shark avoidance returns the commanded commands unchanged. Otherwise it returns the option (left, straight or right) with the highest score, even if every score is below the margin. When the left and right scores differ by less than `CLEARANCE_TIE`, it picks the side the shark is already turning toward, as the wall safety filter does.
- **R20**: Shark avoidance is a pure function in its own module, built into both executables. `MotionParams`, `Commands`, `integrate` and the world size move from the main file to a shared header unchanged, so the module and the main file use the same definitions.

### Contact target (seam 4: contact probe)
- **R21**: With 10 sharks in the current 30 × 16 m world and fish as today, the contact probe measures at most 1 shark contact per minute, averaged over seeds 1–4. With 2 sharks, the average over seeds 1–4 is no higher than with the code at the start of this revision (issue 02 implemented) on the same seeds. The implementer may adjust `SHARK_COMFORT_RANGE`, `W_SHARK_SEPARATION`, `SHARK_AVOID_HORIZON` and `SHARK_AVOID_MARGIN` to meet this, and reports the final values and the measured rates in the handoff. The contact distance is not a tuning value. The target holds only for this density; it says nothing about other world sizes or shark counts.

### Build
- **R13**: The main executable and the test executable build. All existing tests pass. `N_SHARKS` stays 2.

## Design decisions
- **Steering uses the body line, not the spine.** Reason: the body line comes from the nose path, which is dynamics history. The spine includes the body wave, so steering on it would feed the animation into the dynamics. Cost: the waving tail can sit up to about 0.3 m (A·L·amp(1)) off the body line, which is small against both ranges.
- **Five body-line points.** Reason: 4 segments of 0.75 m are enough to cover the whole shark. Each segment cuts the curved nose path by at most about 0.13 m in the tightest turn (radius 0.625 m at `v_min`) and about 0.06 m at cruise. These are my figures, not measured. Rejected: 24 points (the spine's count) and the raw nose path (about 180 points at 60 fps), which cost more for no visible gain.
- **Distance to the body line itself.** Reason: the half-width is at most 0.18 m, and the pectoral tips reach 0.6 m out. That is within the tuning slack of the ranges. Rejected: subtracting the half-width or including the fins and cephalofoil, which would read body geometry, the drawing data.
- **One term per shark, from its nearest body-line point.** Reason: the strength then does not depend on the point count, and a fish ahead of the nose gets the same push as before. Rejected: one term per segment, which makes the body push harder than the nose and turns the point count into a tuning knob.
- **Fish ranges, ramps, weights and normalisation unchanged.** Reason: only the measured point changes for fish, and fish fleeing was accepted by eye.
- **Soft separation plus a hard shark avoidance step.** Reason: measured at 10 sharks, a stronger soft push stops helping once the turn rate saturates at `r_max`, and retuning alone leaves about 4 contacts per minute. Retuning keeps the hard step rare, so sharks still look like they steer; the hard step catches the rest. Rejected: soft only (rev 0), which misses the target; hard only, which would make sharks swerve late and often.
- **Retune shark separation to range 6 m, weight 4 as the start.** Reason: the best soft-only setting in the probe sweep. Range helped more than weight, because reacting earlier matters more than pushing harder. The values are a start, not a requirement, since the target is what matters.
- **Split `W_SEPARATION` into `W_FISH_SEPARATION` and `W_SHARK_SEPARATION`.** Reason: the shared weight would change fish schooling when sharks are retuned.
- **Prediction over three options, not a check against where the other shark is now.** Reason: another shark covers about 4 m in the time a shark needs for a half turn, so a static check comes too late. Holding three options and checking them against moving body lines is the simplest model that accounts for motion.
- **Other sharks move straight at their current speed.** Reason: the simplest motion model. Cost: a turning shark's body line is predicted wrong by the end of the horizon. Both sharks run the step, which makes up for part of that.
- **Prediction through the real `integrate`, at a fixed dt of 1/30 s.** Reason: the turn rate lags by `T_r` = 0.4 s and the speed by `T_v` = 1.0 s. An ideal model would overestimate how fast the shark turns away and take over too late. A fixed dt makes the prediction independent of the frame rate. Cost: `integrate` and `Commands` also move to the shared header.
- **Options at `v_min` and ±`r_max`.** Reason: slowing to `v_min` halves the turn radius, from 1.25 m to 0.625 m, the biggest gain the dynamics allow.
- **Horizon 2.5 s and margin 0.2 m as the start.** Reason: a 90° turn away at `v_min` and `r_max`, with the turn lag, takes about 2.4 s (π/2 ÷ 0.8 + 0.4). The margin is the same size as `SAFETY_MARGIN` but is its own constant.
- **Best option even when all fail.** Reason: turning away from the threat beats holding course into it.
- **Score is the smaller of shark clearance and wall clearance.** Reason: the step then avoids options that steer into a wall, which covers most of the conflicts measured at contact onset (the wall safety filter was active at 20–60% of them). The takeover trigger uses shark clearance only, because walls are the wall safety filter's job.
- **Wall safety filter last, unchanged.** Reason: it keeps the guarantee that the drawn body stays inside the window. Cost: in a real trap, with a wall on one side and a shark on the other, the wall wins and a contact can happen. The user accepts this, because the other shark is avoiding too.
- **Tie rule as in the wall safety filter.** Reason: a head-on encounter is symmetric, and the tie needs a rule that is deterministic and consistent with the existing filter.
- **Nose against body.** Reason: the user's decision. A shark steers its own nose away from other sharks' bodies, in both the soft term and the hard step. Its own trailing body is not checked.
- **Shark avoidance in its own module, with a pure function.** Reason: it predicts, scores and chooses, which can be checked with hand-built cases. The probe can show that contacts happen, but not why. Moving `MotionParams`, `Commands`, `integrate` and the world size is the smallest extraction that makes the module possible.
- **The body line is computed in the body update and stored in the body state.** Reason: it is computed once per shark per frame, not once per fish. The path and the arc-length placement already live in the body module and are tested. Cost: the body module now also serves steering. Fish and sharks read the previous frame's sharks, whose body line was built from the same kinematics, so the nose and the body line agree.
- **The nearest-point function lives in the body module.** Reason: it sits at a testable seam next to the body line.
- **No early skip by distance in the fish loop.** Reason: 4 segments × 2 sharks is 32 point-to-segment tests per fish per frame. An early-out is a guess at an optimisation and can be added if profiling shows a need.
- **The soft steering terms are not unit tested.** Reason: they are one swap per loop in existing formulas. The contact probe covers their effect on sharks.
- **A measured contact target for 10 sharks at today's density only.** Reason: now that shark contact has a definition, a count means something. The count depends on the density and on the number of pairs, so a target for one fixed setup is honest, and a scaled target would claim a generality nobody tested. The value 1 per minute is the user's judgement: rare enough to be seldom seen.
- **The contact probe stays outside the repo, described here.** Reason: committing it would mean adding a headless mode to the main file, a feature of its own that fits the later refactor. A precise procedure makes the number reproducible in any session.

## Testing approach
- **Seams**
  - *Body update* (existing): takes the previous body state, kinematics and dt, and returns the new body state, including the body line. Catches point count, placement on the nose path, arc lengths, the spawn state and independence from the body wave. Misses how steering uses the line.
  - *Nearest point* (existing, pure): takes a position and a body line, and returns the nearest point on it. Catches projection onto a segment, clamping at the nose and the tail tip, and bends. Misses the steering terms and the ranges.
  - *Shark avoidance* (new, pure): takes the shark's kinematics, its commanded commands, every other shark's kinematics and body line, and the motion parameters, and returns the commands. Catches the takeover trigger, the option chosen, the wall scoring and the commanded speed. Misses the interplay with soft separation and the overall contact rate.
  - *Contact probe* (outside the repo, whole system): runs the full loop headless and counts shark contacts. Catches the 10-shark target and the 2-shark comparison. Misses why a contact happens.
- **Unit tests (doctest)**
  - R1–R3: seed a shark and check the point count, the s values and the straight line.
  - R2, R5: drive straight and circular nose paths, with even and uneven dt, and check each point's distance to the path and its arc length.
  - R4: drive the same nose path with amplitude 0 and 0.1 and compare the body lines.
  - R6–R8: hand-built straight and bent body lines, with positions ahead of the nose, behind the tail tip, beside each segment and inside a bend.
  - R16–R19, hand-built shark avoidance scenarios, each in the middle of the world unless stated:
    - No other shark, and another shark far beyond reach within the horizon: the commands come back unchanged.
    - Head-on: another shark ahead on the same line, swimming toward the nose: takeover at `v_min` with turn rate ±`r_max`.
    - Body line ahead and to the right: takeover turning left (positive turn rate). The mirror case turns right.
    - Crossing: another shark that is not near now but swims across the predicted path within the horizon: takeover. With the same shark held still (speed 0), no takeover. This catches R17.
    - Side by side: another shark parallel, same heading and speed, outside contact distance plus margin: no takeover.
    - Wall trap: a threat ahead and a wall close on the left: the step does not pick left.
    - Symmetric head-on with the shark already turning right: it picks right (R19 tie rule).
- **Contact probe** (R21): a headless copy of the main loop outside the repo, with the same simulation code and drawing removed. `N_SHARKS` set to 10 (or 2 for the comparison), fixed dt = 1/60 s, a random seed per run, 120 s per run, seeds 1–4. The first 5 s are skipped. Each frame, after the update, the distance between every pair's body lines is computed by sampling each segment of one body line at 11 evenly spaced points and taking the nearest point on the other body line. A contact is counted when a pair goes from at least 0.36 m to below 0.36 m. The rate is contacts per minute over the 115 s counted, averaged over the seeds.
- **Build check**: R9–R11, R13–R15 and R20. The project builds, and the existing tests pass.
- **Visual check by the user**: R12.

## Out of scope
- A guarantee of zero shark contact. Sharks cannot stop and walls can trap them.
- Any change to the wall safety filter.
- Checking a shark's own trailing body against other sharks' bodies.
- Fish keeping clear of the fins, the cephalofoil or the body's width.
- Tuning fish ranges, weights or ramps.
- A contact target for other world sizes or shark counts.
- Splitting the main file into loop, steering, dynamics and drawing. A later refactor cycle.
- Committing the contact probe or a headless mode.
- Spatial early-outs or other performance work.
- Any change to eating, targeting, the controller or the drawing.

## Revisions
- Rev 1: Added a hard shark avoidance step (R15–R20) and a measured contact target at 10 sharks (R21). Shark separation now starts at range 6 m with its own weight `W_SHARK_SEPARATION` = 4 (R10, R14). R12 now also covers 10 sharks. The rev 0 decisions "soft repulsion only", "ranges and weights unchanged" and "visual acceptance, no instrumented run" are replaced. Reason: with 10 sharks the user saw sharks swim through each other; the probe measured about 21 contacts per minute, and retuning alone left about 4.
