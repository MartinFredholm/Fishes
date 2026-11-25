# Glossary

**Shark**: A scalloped hammerhead (*Sphyrna lewini*), the predator. It is the only entity with a spine and procedural animation.
_Not to be confused with_: fish, the 0.2 m schooling prey.

**Body length (L)**: The shark's nose-to-tail length along the spine, 3.0 m. Spine positions and shape proportions are given as fractions of L.

**Spine**: The shark's centreline, a chain of nodes from nose to tail placed along the path the nose has traced. It has no undulation of its own.
_Not to be confused with_: body wave, which is added on top of the spine.

**s**: Position along the spine as a fraction of L. 0 at the nose, 1 at the tail tip. Dimensionless.

**Body wave**: The travelling lateral offset added to the spine along its local normal, small at the head and large at the tail. It is a function of s and a phase that advances with speed.

**Cephalofoil**: The hammerhead's wide, flattened head, drawn as a rigid shape attached to the front of the spine and oriented by the local tangent.

**Nose**: The front end of the shark's spine (s = 0). For a shark, `kin.pos` is the nose: the point the dynamics integrate and the spine follows. All distances to the shark (eating, fleeing, separation) are measured from it. The drawn nose sways around it by the body wave. Meters, world frame.
_Not to be confused with_: body centre, which is what `kin.pos` meant before this change and still means for fish.
