# Style and Conventions
- Language: Java (follow standard Java 17 conventions, packages `com.automaton.*`).
- Project structure modules: `core` for game logic, `desktop` for launcher.
- Physics: custom `Particle` and `Bond` classes, avoid Box2D joints for bonds; use collision filtering and smaller collision radius than visuals.
- Rendering: plan to use instanced rendering or mesh batching (avoid ShapeRenderer) with custom shaders.
- Graph management: maintain incremental union-find updated only when bonds form/break.
- Documentation: keep TODO.md updated as tasks progress; use succinct code comments only when necessary for complex logic.
