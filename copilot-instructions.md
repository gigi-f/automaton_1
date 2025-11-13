# Copilot Instructions — Broad Overview

Purpose

This file contains a short, high-level overview for contributors and for the Copilot/assistant to follow while implementing the cellular-automaton-inspired particle system using libGDX + Box2D in Java.

Project goals
- Real-time, continuous 2D simulation of nodes (particles) connected by bars (bonds).
- Physics-inspired behavior: bonds act like springs/joints and can form, change, or break due to stochastic reactions and mechanical stress.
- Scales to thousands of nodes with performance optimizations.

Architecture overview

1. Core modules
   - `physics` (Box2D wrapper)
     - `Particle` (wraps a Box2D dynamic body): stores type, mass, radius, energy state, and pooling logic.
     - `Bond` (wraps a DistanceJoint or custom spring): stores bond type, rest length, stiffness, damping, breakThreshold.
   - `spatial`
     - SpatialHashGrid for proximity queries (cell size tuned to typical bond formation range).
   - `rules` (reaction engine)
     - Rule definitions (pairwise, catalytic, cluster-based) executed each tick or at tuned intervals.
     - Stochastic decision-making with seedable RNG.
   - `graph`
     - Adjacency bookkeeping for quick connected-component queries and cluster metadata.
   - `render`
     - Efficient rendering for nodes and bonds. Keep rendering decoupled from physics update.
   - `ui`
     - Scene2D or Dear ImGui binding for runtime parameter tuning and debugging.

2. Control flow and timing
   - Fixed physics step (e.g., 1/60s) for Box2D and a separate reaction step frequency (can be every physics step or every N frames).
   - Reaction engine reads particle/bond data, decides bond formation/changes, and posts structural changes to be applied safely between physics steps.

3. Performance strategies
   - Pool Particle and Bond objects to reduce GC.
   - Use cell-based spatial hash for neighbor queries (O(k) per particle instead of O(n^2)).
   - Minimize Box2D joint count where possible — consider grouping or occasional soft constraints.
   - Provide a simplified physics fallback (verlet or mass-spring integration without Box2D) for extreme counts.

Design conventions and style
- Language: Java. Use standard Java 17+ conventions where available.
- Package layout (suggested):
  - `io.projectname.core` (or `org.example.automaton.core`)
    - `physics` — Particle, Bond, PhysicsWorld
    - `spatial` — SpatialHashGrid
    - `rules` — ReactionManager, Rule definitions
    - `graph` — GraphManager, ConnectedComponentFinder
    - `render` — Renderer / RenderUtils
    - `ui` — UI screens and controls
- Keep physics and logic decoupled: ReactionManager should operate on a light-weight snapshot (ids + positions + types) to avoid holding Box2D references too long.

Developer workflow and quick tasks
- Start with a minimal libGDX desktop scaffold and a running Box2D world that renders a few test particles.
- Add SpatialHashGrid and unit tests for neighbor queries.
- Implement a simple reaction: when `A` and `B` are within `formationDistance`, create a `Bond` with probability `p`.
- Visualize bonds and break them when `Bond.getReactionForce()` > `breakThreshold`.

Files to create first (priority)
- `TODO.md` (this project's todo list) — done
- Basic libGDX scaffold (core + desktop) — next
- `physics/Particle.java`, `physics/Bond.java`, `physics/PhysicsWorld.java`
- `spatial/SpatialHashGrid.java`
- `rules/ReactionManager.java` (start with pairwise A+B rule)
- `render/Renderer.java` (draw particles and bonds)

Testing and validation
- Add unit tests for SpatialHashGrid and Graph connected-component finder.
- Add a small integration benchmark that spawns configurable N particles and prints FPS and memory usage.

Notes for Copilot/Assistant
- Always propose small, incremental changes and verify they compile and run when creating runnable code.
- When making changes that affect many files, provide a short plan and apply edits in small patches.
- Preserve coding style and avoid large refactors early — focus on a working MVP.

