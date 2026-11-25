# 08: Cephalofoil, pectoral and pelvic fins
Depends on: 07
Covers: R15, R17

## What to build
Add the rigid parts to `buildBodyShape`: the cephalofoil, a pair of pectoral fins and a pair of pelvic fins. Each is attached to the spine at its s position and oriented by the local spine tangent. Shapes and positions come from the shape values table in `docs/spec.md` (unverified starting values). There are no dorsal fins and no pectoral tilt. Draw the parts in the same grey-bronze fill as the outline.

## Acceptance criteria
- [ ] R15: on straight and arc-shaped spines, each part's attachment points lie at their s positions within 1 mm and are oriented by the local tangent.
- [ ] R15: no dorsal fins are produced.
- [ ] R17: the whole shark is one grey-bronze fill with no shading.
- [ ] All earlier tests still pass.
- [ ] Visual run: the shark reads as a hammerhead from above.
