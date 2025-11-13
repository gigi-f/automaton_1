# Project Overview
- Goal: build a real-time 2D cellular-automaton-inspired simulation using libGDX + Box2D where particles (nodes) form and break bonds with physics-like reactions.
- Tech stack: Java 17+, Gradle multi-module (core + desktop), libGDX 1.11.0, Box2D (but bonds will use custom verlet constraints), LWJGL3 backend for desktop.
- Key design themes: continuous motion, stochastic reactions, scale to thousands of nodes, hybrid physics (Box2D bodies + custom springs), incremental graph bookkeeping.
- Current state: scaffolded libGDX project with minimal `AutomatonGame`; TODO.md and copilot instructions define future roadmap.
- Repository layout: `core/` contains shared game logic, `desktop/` holds desktop launcher, Gradle wrapper scripts in root (lightweight shims).
