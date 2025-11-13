# Copilot Instructions — Broad Overview

Purpose

This file contains a short, high-level overview for contributors and for the Copilot/assistant to follow while implementing the cellular-automaton-inspired particle system using libGDX + Box2D in Java.

Project goals
- Real-time, continuous 2D simulation of nodes (particles) connected by bars (bonds).
- Physics-inspired behavior: bonds act like springs/joints and can form, change, or break due to stochastic reactions and mechanical stress.
- Scales to thousands of nodes with performance optimizations.

Be sure to update TODO.md as progress is made.

Architecture overview

1. Core modules
   - `physics` (Hybrid Box2D + custom constraints)
     - `Particle` (wraps a Box2D dynamic body): stores type, mass, radius (collision radius < visual radius), energy state, and pooling logic.
     - `Bond` (custom spring constraint, NOT Box2D joints): stores bond type, rest length, stiffness, damping, breakThreshold. Applies constraint forces directly to Box2D bodies each frame using verlet-style soft constraints.
     - **Collision filtering**: Use Box2D collision groups/filters to minimize particle-particle collisions (e.g., particles in same cluster don't collide).
   - `spatial`
     - SpatialHashGrid for proximity queries (cell size tuned to typical bond formation range).
   - `rules` (reaction engine)
     - Rule definitions (pairwise, catalytic, cluster-based) executed each tick or at tuned intervals.
     - Stochastic decision-making with seedable RNG.
   - `graph`
     - Adjacency bookkeeping with **incremental union-find** for connected components. Component IDs maintained and updated only when bonds form/break, not every frame.
   - `render`
     - **Instanced rendering or mesh batching** for thousands of nodes and bonds. Use custom shaders and batched geometry (single draw call per type). Avoid ShapeRenderer.
   - `ui`
     - Scene2D or Dear ImGui binding for runtime parameter tuning and debugging.

2. Control flow and timing
   - Fixed physics step (e.g., 1/60s) for Box2D and a separate reaction step frequency (can be every physics step or every N frames).
   - Reaction engine reads particle/bond data, decides bond formation/changes, and posts structural changes to be applied safely between physics steps.

3. Performance strategies
   - Pool Particle and Bond objects to reduce GC.
   - Use cell-based spatial hash for neighbor queries (O(k) per particle instead of O(n^2)).
   - **Hybrid physics model**: Use Box2D bodies for particle motion/collisions, but apply bond constraints as custom verlet-style springs (forces applied directly to bodies each frame) instead of Box2D joints. This avoids Box2D's joint solver bottleneck at scale.
   - **Collision filtering**: Use Box2D collision groups to disable unnecessary collisions (e.g., bonded particles in same cluster). Use smaller collision radius than visual radius to reduce contact overhead.
   - **Incremental graph updates**: Update connected components only on bond formation/break events, not every frame. Use union-find with path compression for O(α(n)) component queries.

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
- Basic libGDX scaffold (core + desktop) — done
- `physics/Particle.java`, `physics/Bond.java` (custom spring constraint), `physics/PhysicsWorld.java`
- `render/ParticleRenderer.java` (instanced rendering or mesh batching), `render/BondRenderer.java`
- `spatial/SpatialHashGrid.java`
- `graph/UnionFind.java` (incremental connected components)
- `rules/ReactionManager.java` (start with pairwise A+B rule)

Testing and validation
- Add unit tests for SpatialHashGrid and Graph connected-component finder.
- Add a small integration benchmark that spawns configurable N particles and prints FPS and memory usage.

Notes for Copilot/Assistant
- Always propose small, incremental changes and verify they compile and run when creating runnable code.
- When making changes that affect many files, provide a short plan and apply edits in small patches.
- Preserve coding style and avoid large refactors early — focus on a working MVP.

