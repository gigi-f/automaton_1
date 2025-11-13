# Implementation TODO

Project choices
- Language: Java (confirmed)
- Framework: libGDX + Box2D for 2D physics
- Motion: continuous particle-based physics
- Time: real-time simulation
- Randomness: stochastic (seedable RNG)
- Scale target: thousands of nodes (performance-focused)

High-level tasks

- [ ] Scaffold libGDX project
  - Create a desktop-focused libGDX Gradle project with `core` and `desktop` modules and Box2D dependency.
  - Acceptance: builds and runs a blank window printing "App started".

- [ ] Create physics layer (Particle system)
  - Implement `Particle` Java class wrapping Box2D dynamic body.
  - Include: type enum, mass, radius, energy, pooling.
  - Acceptance: spawnable particles simulated and rendered as circles.

- [ ] Create bond (Bar) implementation
  - Implement `Bond` class wrapping Box2D DistanceJoint / custom spring.
  - Properties: restLength, stiffness, damping, maxForce (break threshold), bondType.
  - Acceptance: bonds render as lines and break when thresholds reached.

- [ ] Spatial hashing / neighbor search
  - Implement grid-based spatial hash for proximity queries updated each tick.
  - Acceptance: neighbor queries are efficient for thousands of nodes.

- [ ] Reaction system and rules engine
  - Reaction manager inspects nearby particles/bonds and applies stochastic graph rules.
  - Support: seedable RNG, configurable probabilities, formation/break distances.
  - Acceptance: simple A+B -> bond rule works.

- [ ] Graph overlay & bookkeeping
  - Maintain adjacency lists for connected components, cluster detection, and organism identity.
  - Acceptance: query connected components and sizes in real time.

- [ ] Rendering & batching
  - Efficient rendering (SpriteBatch / Mesh / or optimized ShapeRenderer) for thousands of nodes and bonds.
  - Acceptance: smooth rendering on a typical desktop GPU.

- [ ] UI for parameters & controls
  - Runtime controls (Scene2D/ImGui) for temperature, probabilities, bond strength, pause/run/step, RNG seed.
  - Acceptance: live updates affect simulation.

- [ ] Performance optimizations
  - Pooling, joint reduction strategies, tuning Box2D iterations, optional simplified physics mode (verlet) for large counts.
  - Acceptance: documented and measurable improvements.

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
- Box2D may need tuning for large joint counts; profiling and fallbacks planned.
- Use seedable java.util.Random (or SplittableRandom) for reproducible stochastic behavior.

Next steps
- If you want, I can scaffold the libGDX project now (Java, desktop-only) and add a minimal `DesktopLauncher` + `Core` application that opens a window and prints "App started". Say "go scaffold" and I'll proceed.
