# 07: Visual tuning
Depends on: 02, 03, 04, 06
Covers: the spec's deferred decisions (turn bend in motion; final shape values; shark wall buffer)

## What to build
Resolve the three deferred decisions in `docs/spec.md` with the user, who judges by eye.
- **Turn bend in motion.** Watch the shark turn at slow, cruise and fast speeds, in slight and tight turns. The user decides whether to add a bend cap for tight turns, a bend concentrated along part of the body, or a head-to-tail sweep of the bend, and whether the kink near the tail at v_min and r_max needs fixing. Make only the changes the user approves, each with tests at the body update seam, and record each decision in the spec.
- **Shape values.** The user compares the drawn shark with a top-view photo of a scalloped hammerhead. Adjust the shape values in the code and in the spec's shape values table to match, and drop the "unverified" mark for values the user confirms.
- **Wall buffer.** Watch the shark at the new speeds. Change the wall handling for the shark only if it hits walls, and only in a way the user approves. Otherwise record in the spec that `WALL_BUFFER_DIST` stays as it is.

## Acceptance criteria
- [ ] The user approves the shape against the photo, and the spec's shape values table matches the code.
- [ ] The wall buffer decision is recorded in the spec, with its reason.
- [ ] The turn-bend decisions are recorded in the spec, with their reasons, and any change the user approved has tests.
- [ ] All three deferred decisions are removed from the spec's "Deferred decisions" section and recorded under "Design decisions".
- [ ] All tests still pass.
