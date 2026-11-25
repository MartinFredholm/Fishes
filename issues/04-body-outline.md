# 04: Body outline with caudal taper and grey-bronze fill
Depends on: 01
Covers: R13, R14, R16, R17 (outline part)

## What to build
The body geometry seam: from the spine nodes, build the outline using the width profile w(s) in `docs/spec.md` (shape values table, unverified starting values). The caudal fin is the end of the taper, not a separate shape (R14). Draw the outline as a filled strip in a single grey-bronze colour, replacing the polyline. Pixel conversion happens only in drawing.

## Interfaces
- `BodyShape`: the outline as left and right point arrays, one point per spine node, plus a list of rigid-part polygons (empty in this issue). World meters.
- `BodyShape buildBodyShape(const std::vector<Vector2>& spine, float length)`, with one point per spine node from 01.

Issue 05 fills the rigid-part list without changing these signatures.

## Acceptance criteria
- [ ] R13: on straight and arc-shaped spines, each outline point at node i lies at ±w(s_i)/2 along the local normal, within 1 mm.
- [ ] R14: there is no separate caudal shape; the outline tapers to w(1) at the tail tip.
- [ ] R16: the outputs are in world meters.
- [ ] R17 (outline): the shark is drawn as a single grey-bronze fill, not red, with no shading. Fish drawing is unchanged.
- [ ] All earlier tests still pass.
