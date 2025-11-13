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

Notes and assumptions
- We'll start with a desktop-only libGDX project for fast iteration; mobile/Android can be added later.
- **Hybrid physics**: Box2D bodies for particle motion/collisions, custom spring constraints for bonds (NOT Box2D joints) to avoid joint solver bottleneck.
- **Rendering**: Instanced rendering or mesh batching from the start; no ShapeRenderer.
- **Graph updates**: Incremental union-find; update connected components only on bond events.
- **Collision optimization**: Collision filtering + smaller collision radius than visual radius.
- Use seedable java.util.Random (or SplittableRandom) for reproducible stochastic behavior.

Next steps
- Implement the custom bond (spring constraint) system and integrate it with the physics world.
