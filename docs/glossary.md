# Glossary

## Entities and dynamics

**Fish**: The 0.2 m schooling prey (boids). `kin.pos` is its body centre. Drawn as a triangle.
_Not to be confused with_: shark.

**Shark**: A 3 m scalloped hammerhead (*Sphyrna lewini*), the predator. It is the only entity with a body (spine, body wave, outline and rigid parts).
_Not to be confused with_: fish.

**Kinematics**: An entity's dynamic state (`kinematics.h`): position `pos`, heading angle `phi`, speed `v`, yaw rate `r` and the unit vector `heading`. World frame, meters, y down. The dynamics are the source of truth. The shark's body reads them and never feeds back.

**Nose**: For a shark, `kin.pos`. It is the point the dynamics integrate and where the spine starts (s = 0). Eating, targeting, fleeing and shark–shark separation measure from it. The drawn nose sways around it by the body wave.
_Not to be confused with_: body centre, which is what `kin.pos` means for fish.

**Wall clearance**: `MotionParams::wall_clearance`, in meters. It moves the walls inward for one entity, in the wall repulsion and in the safety filter. Shark: `SHARK_HALF_SPAN_AT_PECTORAL_TIPS` (0.6 m), so the drawn body stays inside the window. Fish: 0. It is applied around `kin.pos`.
_Not to be confused with_: `WALL_BUFFER_DIST`, the distance at which wall repulsion starts.

## Shark body

**Body length (L)**: The shark's nose-to-tail length along the spine, 3.0 m. Positions along the body and shape values are given as fractions of L.

**s**: Position along the body as a fraction of L. 0 at the nose, 1 at the tail tip. Negative s lies ahead of the nose. Dimensionless.

**Nose path**: `SharkBody::path`, the positions the nose has traced, with accumulated arc length (`arc` increases toward the nose). It keeps at least L behind the nose and trims the older part. At spawn it is seeded as a straight line of length L along the initial heading.

**Spine**: The shark's centreline, 24 nodes at uniform s from the nose (node 0) to the tail tip (node 23). The nodes are first placed on the nose path at arc length s·L behind the nose, so in a turn the body follows the curve the nose has traced. Then the body wave and the length pass are applied.
_Not to be confused with_: nose path, which is dynamics history and has no body wave.

**Body wave**: The travelling lateral offset added to each spine node along the local normal: h(s) = A·L·amp(s)·sin(k·s − phase). Small at the head and large at the tail. Parameters from the Julia shark blog post (A = 0.1, k = 5.3, St = 0.3).

**Amplitude envelope (amp(s))**: The smooth curve that scales the body wave along the body, through 0.2, 0.21, 0.23, 0.4, 0.88 and 1.0 at s = 0, 0.2, …, 1.

**Phase**: `SharkBody::phase`, in radians. Each update advances it by ω·dt, with ω = 2π·St·v/(2·A·L) = π·v rad/s. Integrated, not computed as ω·t, so the tail-beat frequency follows speed without jumps.

**Length pass**: A head-to-tail pass after the body wave that resets each spine segment to its rest length L/23. The count is `BodyParams::length_passes`.

**Body update**: The first test seam. `updateSharkBody` takes the previous `SharkBody`, the `Kinematics` and dt, and returns the new nose path, phase and spine. It works with any dt ≤ 1/30 s.

**Body geometry**: The second test seam. `buildBodyShape` takes the spine and returns a `BodyShape` (outline and rigid parts) in world meters. Conversion to pixels happens only when drawing.

## Shark shape

**Outline**: The body's left and right edges, one point per spine node at ±w(s)/2 along the local normal. Drawn as a filled triangle strip. "Left" is screen-left of the swimming direction (y down).

**Width profile (w(s))**: The full body width along the body, a smooth curve through the shape values. Widest (0.12 L) at s = 0.25.

**Caudal taper**: The end of the outline, from s ≈ 0.75 to the tail tip. It is the caudal fin seen edge-on from above. There is no separate caudal shape.

**Rigid part**: A shape that does not bend: attached to the spine at its s position and oriented by the local spine tangent. Each is a triangle fan around its attachment point, wound like the outline strip. The shark has five: the cephalofoil, two pectoral fins and two pelvic fins. Dorsal fins are not drawn.

**Cephalofoil**: The hammerhead's wide, flat head. A rigid part attached at the nose, span 0.27 L, leading edge 0.01 L ahead of the nose, wing tips swept back to s ≈ 0.07.

**Pectoral fins**: The large paired fins behind the head. Root from s 0.26 to 0.32, span 0.14 L each, tips swept back to s ≈ 0.38.

**Pelvic fins**: The small paired fins near mid-body. Root from s 0.54 to 0.57, span 0.04 L each, tips swept back to s ≈ 0.60.

**Shape values**: The width profile and the rigid-part dimensions, as fractions of L. They were approved by eye against a top-view photo of a scalloped hammerhead.
