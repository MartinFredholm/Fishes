## Round 1

**Verdict:** pass

An earlier draft of this round failed on a finding that came from a mistake in my probe. The probe compiled `shark_avoidance.cpp` with `-ffast-math`, but CMake builds that library without it, and the module relies on `infinity()`. All numbers below come from a probe built with CMake's per-target flags.

### Requirements
| Requirement | Result | Evidence |
|---|---|---|
| R1 | met | `tests/body_line_tests.cpp` "body line has 5 points…"; `BODY_LINE_POINTS = 5` in `shark_body.h` |
| R2 | met | `tests/body_line_tests.cpp` straight, circle and "varying tightest-radius turn" cases |
| R3 | met | `tests/body_line_tests.cpp` "at spawn the body line is straight…" |
| R4 | met | `tests/body_line_tests.cpp` "body line is the same with and without the body wave" |
| R5 | met | `tests/body_line_tests.cpp` uneven-dt straight and circle cases |
| R6 | met | `tests/body_line_tests.cpp` "nearest point on body line" suite |
| R7 | met | same suite: ahead of nose, behind tail tip, beside a segment |
| R8 | met | same suite: right-angle bend, hairpin, zigzag |
| R9 | met | read `fishes.cpp:431-439`: one term per shark from the nearest body-line point, range 1.8 m, ramp, `Vector2Normalize`, `W_FLEE` unchanged |
| R10 | met | read `fishes.cpp:363-376`: distance from own nose to the nearest point, ramp 1 − d/6, not normalised, weight `W_SHARK_SEPARATION` = 4 |
| R11 | met | `git diff 9099813~1 HEAD -- fishes.cpp`: eating, targeting and `safetyFilter` are unchanged. With 2 sharks, the probe counts 14–15 fish eaten per 120 s, against 16–17 at `9099813`. |
| R12 | met | Visual check by the user: confirmed it looks good. |
| R13 | met | `fishes` and `shark_body_tests` build; 69/69 tests pass; `N_SHARKS = 2` |
| R14 | met | `W_FISH_SEPARATION = 0.9` on fish separation, `W_SHARK_SEPARATION` on sharks (`fishes.cpp:71-72`, `388`, `443`) |
| R15 | met | `fishes.cpp:222-226`: steering → `avoidSharks` → `safetyFilter` |
| R16 | met | `shark_avoidance.cpp`: `integrate` at 1/30 s, four candidates as specified. Horizon retuned to 8 s under R21. |
| R17 | met | `shark_avoidance.cpp:20-21` (nose shifted back by heading·v·t, which is the same as shifting the line forward); "crossing / held still" test |
| R18 | met | `shark_avoidance.cpp:16-37`; wall-trap tests |
| R19 | met | `shark_avoidance.cpp:62-79`; tie-rule tests. Margin retuned to 1.6 m under R21. |
| R20 | met | `motion.h` / `motion.cpp`; definitions unchanged in the diff; library linked into both executables |
| R21 | met | Contact probe as in the spec: seeds 1–4, 120 s, first 5 s skipped, CMake's compile flags. 10 sharks: 3, 0, 0, 1 contacts, mean 0.52 per minute (target ≤ 1). 2 sharks: mean 0.00 per minute, against 0.52 per minute (0, 2, 0, 2 contacts) at `9099813`. Sanity check: 10 sharks at `9099813` gives 17.2 per minute, close to the spec's figure of about 21. |

### Observations (not requirement failures)
- The share of shark frames where `avoidSharks` changes the commands is 2–11% with 2 sharks and 44–55% with 10 sharks. The Goal says soft separation does most of the work. At 10 sharks the hard step is active about half the time. No requirement sets a number for this; R12 is where it shows.
- Fish eaten per 120 s with 10 sharks: 25–41, against 90–96 at `9099813`. Not a requirement, since only the 2-shark setup ships.

### Unnecessary comments
None found. The remaining comments state units, frame and sign conventions, sources, or a non-obvious why.

### Tests added
- `tests/system_tests.cpp` (registered in `CMakeLists.txt`), suite "system, acceptance". All cases chain `integrate` → `updateSharkBody` → `avoidSharks` with real body lines at 60 fps:
  - A positive turn rate through `integrate` bends the body line to screen-left. This checks that the motion module, the body update and avoidance's left option use the same sign convention.
  - Head-on, both sharks running avoidance: their body lines never come closer than the contact distance.
  - Crossing at right angles, both running avoidance: no contact.
  - One shark aimed at the other's tail half: no contact.

### Earlier findings
None. This is the first round.
