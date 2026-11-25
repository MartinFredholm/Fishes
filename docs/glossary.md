# Glossary

**Shark**: A scalloped hammerhead (*Sphyrna lewini*), the predator. It is the only entity with a spine and procedural animation.
_Not to be confused with_: fish, the 0.2 m schooling prey.

**Body length (L)**: The shark's nose-to-tail length along the spine, 3.0 m. Spine positions and shape proportions are given as fractions of L.

**Spine**: The shark's centreline, a chain of nodes from nose to tail, shaped by the turn bend. It has no undulation of its own.
_Not to be confused with_: body wave, which is added on top of the spine.

**Pivot**: The point on the shark's spine that the dynamics integrate: for a shark, `kin.pos` is the pivot, at s_p = 1/3 of a body length behind the nose (starting value, unverified). The shark turns about it: the body behind follows the pivot path and the head swings into the turn ahead of it. Meters, world frame.
_Not to be confused with_: nose, the front end of the spine; body centre, which `kin.pos` still means for fish.

**Pivot path**: The track the pivot has swum. The spine behind the pivot lies along it, so that part of the body does not skid. Meters, world frame.

**Turn bend**: The curvature of the spine that shows the shark turning. Behind the pivot it is the curvature of the pivot path, so a change in turning travels toward the tail as the shark swims. Ahead of the pivot it is the current turn's curvature r/v, so the head swings into a turn as soon as r builds. Units 1/m. It has the same sign as r.
_Not to be confused with_: body wave, which is the periodic swimming undulation.

**Skid**: Sideways motion of a point on the body, across the local spine direction. The body wave's oscillation is not skid. A natural-looking shark does not skid behind the pivot.

**s**: Position along the spine as a fraction of L. 0 at the nose, 1 at the tail tip. Dimensionless.

**Body wave**: The travelling lateral offset added to the spine along its local normal, small at the head and large at the tail. It is a function of s and a phase that advances with speed.

**Cephalofoil**: The hammerhead's wide, flattened head, drawn as a rigid shape attached to the front of the spine and oriented by the local tangent.

**Mouth point**: The point that targeting and eating measure from. It lies one pivot-to-nose distance ahead of the pivot, along the current turn's curvature r/v, and is computed from `kin` alone. It is the drawn nose without the body wave's sway. Meters, world frame.
_Not to be confused with_: pivot (`kin.pos`), from which fleeing, separation, walls and the safety filter measure.

**Nose**: The front end of the shark's spine (s = 0). It swings into turns ahead of the pivot, and the body wave sways it. Meters, world frame.
_Not to be confused with_: pivot, which is `kin.pos`.
