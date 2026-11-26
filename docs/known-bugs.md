# Known bugs

## The spine folds in slow, tight turns
At 0.5 m/s with r = ±0.8 rad/s, the body update sometimes folds the spine at nodes 13–14. The bend between neighbouring segments reaches 1.2–1.6 rad, so slow tight turns probably show a kink. Segment lengths stay correct, so no test catches it. One test skips those frames: "rigid parts on a swum, waved, turning spine" in `tests/rigid_parts_tests.cpp`.
