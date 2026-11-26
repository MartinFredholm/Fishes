# 04: Shark avoidance
Depends on: 01, 03
Covers: R15, R16, R17, R18, R19, R20

## What to build
The hard shark avoidance step from Rev 1 (see `docs/spec.md`, R15–R20, and the glossary entries **Shark avoidance** and **Shark contact**).

A new module with a pure function that predicts the shark's own nose under the commanded commands and the three options, against the other sharks' moving body lines and the walls. It returns the commanded commands or the best option, as R16–R19 state. It uses `integrate` from issue 03 and `nearestPointOnBodyLine` from issue 01.

Wire it into the shark loop in the main file: after the steering produces the commands, before the wall safety filter, which stays unchanged and last (R15). Build the `others` list from the previous frame's sharks, excluding the shark itself.

Write the unit tests from the spec's shark avoidance scenario list.

Constants in the module: `SHARK_AVOID_HORIZON` = 2.5 s, `SHARK_AVOID_MARGIN` = 0.2 m, `SHARK_CONTACT_DISTANCE` = 0.36 m. Issue 05 may retune the horizon and the margin.

## Interfaces
```cpp
struct OtherShark {
    Kinematics kin;
    std::vector<Vector2> body_line;
};

Commands avoidSharks(const Kinematics &self, Commands commanded,
                     const std::vector<OtherShark> &others,
                     const MotionParams &params);
```
World frame, meters, y down. Positive turn rate turns screen-left of the swimming direction. `others` does not contain the shark itself.

## Acceptance criteria
- [ ] With no other shark, and with another shark beyond reach within the horizon, the commands come back unchanged (R19).
- [ ] Head-on, another shark ahead on the same line swimming toward the nose: takeover at `v_min` with turn rate ±`r_max` (R16, R19).
- [ ] A body line ahead and to the right gives a left turn (positive turn rate); the mirror case gives a right turn (R18, R19).
- [ ] Crossing: another shark not near now but crossing the predicted path within the horizon causes a takeover; the same shark with speed 0 does not (R17).
- [ ] Side by side, parallel, same heading and speed, outside contact distance plus margin: no takeover (R19).
- [ ] Wall trap, a threat ahead and a wall close on the left: the step does not pick left (R18).
- [ ] Symmetric head-on with the shark already turning right: it picks right (R19 tie rule).
- [ ] In the main loop, shark commands pass through shark avoidance after the steering and before the unchanged wall safety filter (R15).
- [ ] The module is a pure function in its own module, built into both executables (R20).
- [ ] Both executables build and all tests pass.
