# 05: Retune shark separation and meet the contact target
Depends on: 02, 04
Covers: R10, R12 (shark half), R21

## What to build
Rework of issue 02, caused by spec Rev 1 (see `docs/spec.md`, R10 and R21).

Set `SHARK_COMFORT_RANGE` to 6 m and `W_SHARK_SEPARATION` to 4. The rest of shark separation stays as issue 02 built it.

Build the contact probe outside the repo, following the procedure in the spec's testing approach, and measure. If R21 is not met, adjust `SHARK_COMFORT_RANGE`, `W_SHARK_SEPARATION`, `SHARK_AVOID_HORIZON` and `SHARK_AVOID_MARGIN` until it is. Do not change the contact distance.

For the 2-shark comparison, measure the baseline on the commit that contains issue 02, with the same probe and seeds.

Report in the handoff: the final values of the four tuning constants, the contact rate per seed and averaged at 10 sharks, the same at 2 sharks for the baseline and for the final code, and where the probe lives.

## Acceptance criteria
- [ ] Shark separation measures from the shark's nose to the nearest body-line point, uses the ramp 1 − d/`SHARK_COMFORT_RANGE`, is weighted by `W_SHARK_SEPARATION` and is not normalised (R10).
- [ ] With 10 sharks, the contact probe averages at most 1 contact per minute over seeds 1–4 (R21).
- [ ] With 2 sharks, the average over seeds 1–4 is no higher than the issue 02 baseline (R21).
- [ ] The handoff lists the final tuning values and the measured rates.
- [ ] `N_SHARKS` is 2 in the committed code. Both executables build and all tests pass.
- [ ] Visual check by the user: with 2 sharks and with 10 sharks, sharks no longer swim through each other's bodies in normal swimming (R12, shark half).
