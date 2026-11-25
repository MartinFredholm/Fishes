# 06: Visual tuning
Depends on: 02, 03, 05
Covers: the spec's deferred decisions (final shape values; shark wall buffer)

## What to build
Resolve the two deferred decisions in `docs/spec.md` with the user, who judges by eye.
- **Shape values.** The user compares the drawn shark with a top-view photo of a scalloped hammerhead. Adjust the shape values in the code and in the spec's shape values table to match, and drop the "unverified" mark for values the user confirms.
- **Wall buffer.** Watch the shark at the new speeds. Change the wall handling for the shark only if it hits walls, and only in a way the user approves. Otherwise record in the spec that `WALL_BUFFER_DIST` stays as it is.

## Acceptance criteria
- [ ] The user approves the shape against the photo, and the spec's shape values table matches the code.
- [ ] The wall buffer decision is recorded in the spec, with its reason.
- [ ] Both deferred decisions are removed from the spec's "Deferred decisions" section and recorded under "Design decisions".
- [ ] All tests still pass.
