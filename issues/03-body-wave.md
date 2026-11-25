# 03: Body wave, phase and length pass
Depends on: 01
Covers: R8, R9, R10, R11, R12 (wave part)

## What to build
Inside `updateSharkBody`, after placing the nodes on the path: advance the phase, offset each node along the spine's local normal by the body wave, then run the length-restoring pass. See `docs/spec.md` R8–R12 for formulas, parameters and the amplitude envelope. The signatures from 01 stay the same. The number of passes is `BodyParams::length_passes`.

## Acceptance criteria
- [ ] R8: the offset follows h(s) = A·L·amp(s)·sin(k·s − φ), with A = 0.1, k = 5.3 and amp(s) through the blog's six points.
- [ ] R9: after one update with known u and dt, the phase advanced by π·u·dt within 1e-6 rad.
- [ ] R10: after every update, while swimming straight and while turning, every segment is within 0.1 % of L/23.
- [ ] R11: straight swimming at 1.0 m/s, after at least one body length, gives a peak tail-tip lateral offset over one beat cycle within ±20 % of 0.30 m.
- [ ] R12: uneven dt values (all ≤ 1/30 s) still meet R9 and R10.
- [ ] The 01 tests still pass with `wave_amplitude = 0`.
- [ ] Visual run: the polyline undulates, with a small swing at the head and a large one at the tail, and the beat frequency rises with speed.
