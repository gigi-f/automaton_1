# Violent Bond Breaking Reactions

## Overview
When bonds break in the simulation, they now produce violent reactions—explosive repulsive forces that push the two bonded particles apart. The strength of the reaction is directly proportional to the bond's break force threshold.

## Mechanics

### Reaction Force Calculation
```
Reaction Multiplier = breakForceThreshold / 50
Repulsion Strength = 300 × Reaction Multiplier
```

### Force Scaling Examples
- **Fragile bond** (break force = 60): 1.2x multiplier → 360 repulsion force
- **Medium bond** (break force = 200): 4x multiplier → 1200 repulsion force  
- **Strong bond** (break force = 600): 12x multiplier → 3600 repulsion force

### Visual Feedback
Bond colors now indicate their strength, providing visual warning before violent reactions:
- **Red bonds**: Fragile (break force < 200) - small reactions
- **Yellow bonds**: Medium strength (break force ≈ 200-400) - moderate reactions
- **Green bonds**: Strong (break force > 400) - **explosive reactions**

## When Reactions Occur

### 1. Stress-Based Breaking
Bonds break when the spring force exceeds the break force threshold during normal physics simulation. This triggers violent reactions automatically.

### 2. Crossing Detection
When bonds within the same molecule cross each other (spatial intersection), both crossing bonds break simultaneously with violent reactions, creating dramatic explosions.

### 3. Stochastic Breaking
The ReactionManager randomly breaks bonds based on logic gate rules and probability. These also trigger violent reactions.

## No Reactions
Violent reactions are **disabled** for:
- Manual cleanup operations (`clearAllBonds()`)
- Simulation restart (`restartSimulation()`)

## Physics Impact
The repulsive forces are applied to the Box2D bodies using `applyForceToCenter()`, which:
- Pushes particles in opposite directions along the bond axis
- Creates dramatic separation effects
- Can trigger chain reactions when molecules break apart
- Affects neighboring particles through subsequent collisions

## Bond Type Variety
With 45+ unique bond combinations having break forces ranging from ~60 to ~600:
- **XOR-XNOR bonds** (very fragile): Small pops when breaking
- **NOR-FALSE bonds** (very strong): Massive explosions when breaking
- Logic gate combinations create varied reaction intensities

## Implementation Details
See `PhysicsWorld.destroyBond()` and `applyBondBreakReaction()` methods for the core implementation.
