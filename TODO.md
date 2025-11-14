# Implementation TODO

Project choices
- Language: Java (confirmed)
- Framework: libGDX + Box2D for 2D physics
- Motion: continuous particle-based physics
- Time: real-time simulation
- Randomness: stochastic (seedable RNG)
- Scale target: thousands of nodes (performance-focused)

High-level tasks

- [x] Scaffold libGDX project
  - Create a desktop-focused libGDX Gradle project with `core` and `desktop` modules and Box2D dependency.
  - Acceptance: builds and runs a blank window printing "App started".

- [x] Create physics layer (Particle system)
  - Implement `Particle` Java class wrapping Box2D dynamic body.
  - Include: type enum, mass, collision radius (smaller than visual radius), energy, pooling.
  - **Use collision filtering**: Configure Box2D collision groups to minimize unnecessary particle-particle collisions.
  - Acceptance: spawnable particles simulated and rendered as circles.

- [x] Create bond (Bar) implementation
  - Implement Bond class as custom spring constraint (NOT Box2D joint). Apply verlet-style forces to bodies each frame. Properties: restLength, stiffness, damping, breakForceThreshold, bondType. Bonds render as lines, apply spring forces, break when threshold exceeded.
- [x] Implement Reaction system
  - Create ReactionManager with seedable RNG. Implement bond formation rules (A + B within distance → create bond with probability p). Implement bond break rules beyond mechanical stress. Acceptance: simple A+B -> bond rule works.
- [x] Graph bookkeeping (component tracking)
  - Implement incremental union-find to track connected components. Update only when bonds form/break, not every frame. Store component ID in Particle. Acceptance: can query which particles belong to same structure efficiently.
- [x] Camera controls
  - Implement pan (drag), zoom (scroll), screen-to-world coordinate conversion. Acceptance: can navigate large simulation space smoothly.
- [x] Implement Spatial hashing / neighbor search
  - Create SpatialHashGrid with cell-based grid for proximity queries. Integrate with PhysicsWorld to update grid each tick. Use for efficient neighbor finding (O(k) vs O(n²)).
- [x] Fix particle motion & create molecule demo
  - Add random initial velocities to particles. Replace grid spawn with varied molecules (singles, pairs, triplets, triangles, chains). Color gradient based on mol size (white=1 node, dark blue=10+ nodes).
- [x] Implement Reaction system
- [ ] Optimize rendering
  - Replace ShapeRenderer in BondRenderer with mesh batching. Replace SpriteBatch in ParticleRenderer with instanced rendering or custom shader. Acceptance: Smooth 60fps with thousands of elements.

- [ ] Spatial hashing / neighbor search
  - Implement grid-based spatial hash for proximity queries updated each tick.
  - Acceptance: neighbor queries are efficient for thousands of nodes.

- [ ] Reaction system and rules engine
  - Reaction manager inspects nearby particles/bonds and applies stochastic graph rules.
  - Support: seedable RNG, configurable probabilities, formation/break distances.
  - Acceptance: simple A+B -> bond rule works.

- [ ] Graph overlay & bookkeeping
  - Maintain adjacency lists for connected components using **incremental union-find** (with path compression).
  - Component IDs are updated **only when bonds form/break**, not every frame.
  - Acceptance: query connected components and sizes in O(α(n)) time; updates triggered by bond events only.

- [ ] Camera controls & viewport
  - Implement OrthographicCamera with pan (mouse drag), zoom (scroll wheel), and optional follow mode (track a particle or cluster).
  - Add viewport management for window resize handling.
  - Acceptance: smooth camera navigation; can zoom out to see thousands of particles and zoom in for detail.

- [ ] Rendering & batching
  - **Use instanced rendering or mesh batching** for thousands of nodes and bonds (single draw call per type).
  - Implement custom shaders and batched geometry. **Do not use ShapeRenderer** (too slow).
  - Acceptance: smooth 60fps rendering with thousands of elements on a typical desktop GPU.

- [ ] UI for parameters & controls
  - Runtime controls (Scene2D/ImGui) for temperature, probabilities, bond strength, pause/run/step, RNG seed.
  - Acceptance: live updates affect simulation.

- [ ] Performance optimizations
  - **Hybrid physics model**: Box2D bodies + custom verlet spring constraints (already implemented in Bond class).
  - **Collision filtering**: Box2D collision groups to disable unnecessary collisions; collision radius < visual radius.
  - Pooling for Particle and Bond objects. Tune Box2D velocity/position iterations for performance.
  - Acceptance: documented and measurable improvements; smooth performance with thousands of nodes.
  - [ ] Optimize `applyHuntingBehavior()` in `PhysicsWorld` (lines ~870-960). It currently recomputes molecule groupings each tick and, for every molecule, scans *all* bonds to measure average strength, resulting in O(M·B) work per frame. Cache per-component bond aggregates or maintain running stats.
  - [x] Accelerate `applyFoodSuction()` in `PhysicsWorld` (lines ~960-1040). Switched to spatial-hash radius queries per molecule center so we only consider nearby food/corpses instead of scanning every particle.
  - [x] Reduce cost of `detectAndBreakCrossingBonds()` (lines ~2050-2095). Bonds are now bucketed via the spatial hash cell size and compared only within/adjacent buckets, slashing the previous O(B²) scan.
  - [x] Avoid full `rebuildUnionFind()` calls inside `processMitosis()` (line ~1355). Component rebuilding now runs only when bond/particle changes mark the structure dirty, eliminating the unconditional sweep every few seconds.
  - [x] Rework mitosis splitting dynamics. Bonds intersecting the molecule's central plane now break together, their stored break-force energy is split between the daughter cells' remaining bonds, and equal/opposite pushes separate the two halves. Mitosis checks now run on a 1s cadence in small batches so the system stays responsive without tanking performance.

- [ ] Example node/bond types and reactions
  - Implement at least 4 node types and 3 bond types with different behaviors (push, break, catalysis, stretchy).
  - Acceptance: visible distinct behaviors.

- [ ] Save/load presets and replay
  - Save experiment presets and replay via RNG seed and simple replay log.
  - Acceptance: reproducible experiments.

- [ ] Automated tests and benchmarks
  - Unit tests for neighbor search, bond logic, and connectivity. Benchmark harness to spawn N nodes and report FPS.
  - Acceptance: tests run locally and a baseline benchmark exists.

- [ ] Documentation and README
  - Build/run instructions for Linux (desktop), design notes, and parameter explanations.
  - Acceptance: README at project root.

- [ ] Polish: inspector, visuals, presets
  - Node inspector, hover selection, trails, color maps, 3 example presets.
  - Acceptance: shipped presets.

## New Features (Added)

- [x] Add world configuration sliders
  - Add UI sliders for: a) initial spawn count (number of nodes), b) world size (dynamic boundary adjustment)
  - Boundaries should expand/contract based on world size slider
  - Acceptance: can adjust spawn count and world size, restart updates boundaries

- [x] Monochrome rendering mode toggle
  - Add UI checkbox plus color picker to force all nodes and bonds to render in a single (default blue) color when desired.
  - Acceptance: Toggle and color picker appear together; changing either updates the simulation visuals immediately.
- [x] Monochrome status + dark mode toggle
  - Display a read-only checkbox showing whether monochrome mode is currently active and add a dark-mode toggle that swaps the background between black and white.
  - Acceptance: Indicator mirrors the toggle, and disabling dark mode visibly flips the playfield to a white background.

- [x] Multi-type energy fields with environmental dynamics
  - Multiple energy field types (color-coded: red, green, blue, etc.)
  - Each field type attracts/repels specific particle types differently
  - Fields should move across screen like waves (spawn from edge, move in random direction/speed)
  - Visual representation for each field type
  - Acceptance: multiple colored fields visible, moving, affecting different particles uniquely

- [x] Corpse system (dead particles remain consumable)
  - Dead particles (energy=0) remain in world as "corpses" 
  - Corpses cannot form new bonds but remain consumable by living particles
  - Visual indicator for dead particles (grayscale, reduced opacity, or distinct marker)
  - Acceptance: dead particles visible but inactive, can be consumed for energy

- [ ] Cellular organelle roadmap
  - [ ] Implement "carrier" organelle type that can ferry energy between neighboring cells without leaving their host membrane.
  - [ ] Add visualization/state debugging for different organelle classes so it's clear which provide photosynthesis vs. transport.

- [x] Organelle immortality & replication controls
  - Organelles trapped inside a cell no longer lose energy or die, ensuring consistent power sources for the host membrane.
  - Added UI slider for "Organelle Replication" to control the time (seconds) it takes for organelles to self-replicate inside a cell, with "Off" option at 0 seconds.

Notes and assumptions
- We'll start with a desktop-only libGDX project for fast iteration; mobile/Android can be added later.
- **Hybrid physics**: Box2D bodies for particle motion/collisions, custom spring constraints for bonds (NOT Box2D joints) to avoid joint solver bottleneck.
- **Rendering**: Instanced rendering or mesh batching from the start; no ShapeRenderer.
- **Graph updates**: Incremental union-find; update connected components only on bond events.
- **Collision optimization**: Collision filtering + smaller collision radius than visual radius.
- Use seedable java.util.Random (or SplittableRandom) for reproducible stochastic behavior.

