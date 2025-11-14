# Energy Field System

## Overview
The simulation now uses **4 moving energy fields** (RED, GREEN, BLUE, YELLOW) that replace the static energy field. Each field type has unique affinities for different particle types, creating complex environmental dynamics.

## Field Types and Particle Affinities

### RED Field (Red colored, 20% opacity)
- **Strongly Attracts**: AND gates (1.0x)
- **Attracts**: NAND gates (0.8x)
- **Weakly Attracts**: Other gates (0.2x)
- **Repels**: TRUE states (-0.6x), FALSE states (-0.4x)
- **Effect**: Energy gain only for attracted particles

### GREEN Field (Green colored, 20% opacity)
- **Strongly Attracts**: TRUE states (1.0x)
- **Attracts**: OR gates (0.8x)
- **Weakly Attracts**: Most gates (0.1x)
- **Repels**: FALSE states (-0.8x), NOR gates (-0.5x)
- **Effect**: Provides most energy to TRUE particles

### BLUE Field (Blue colored, 20% opacity)
- **Strongly Attracts**: NOT gates (1.0x)
- **Attracts**: NOR gates (0.7x)
- **Neutral**: Logic states (0.0x)
- **Repels**: XOR gates (-0.7x), XNOR gates (-0.5x)
- **Effect**: Selective energy for inverter-type gates

### YELLOW Field (Yellow colored, 20% opacity)
- **Strongly Attracts**: XOR gates (1.0x), XNOR gates (0.9x)
- **Weakly Attracts**: FALSE states (0.3x)
- **Neutral**: Other types (0.0x)
- **Effect**: Specialized for exclusive gates

## Mechanics

### Energy Absorption
- Particles gain energy only from fields they have **positive affinity** for
- Energy gain = `fieldEnergy × affinity × environmentalEnergyRate × deltaTime`
- Multiple fields can provide energy simultaneously
- Repelled particles (negative affinity) gain NO energy from that field

### Chemotaxis (Movement)
- Particles are pushed/pulled by field gradients based on affinity
- Positive affinity = attracted toward field center
- Negative affinity = repelled from field center
- Force = `gradient × chemotaxisStrength × affinity × fieldEnergy`
- All active fields contribute to particle movement

### Field Behavior
- Fields spawn from random edges (left, right, top, bottom)
- Random speeds: 5-15 units/second
- Random directions with some cross-screen component
- **Variable radius**: 15-50 units (random per field for visual variety)
- Strength: 0.5-1.0 (random per field)
- **Quadratic falloff**: Energy is strongest at field center, fades to weak at edges
- Fields respawn from new edges when they exit the world

## Strategic Implications

1. **Type-Specific Niches**: Different particle types thrive in different field conditions
2. **Dynamic Environment**: Moving fields create constantly changing energy landscapes
3. **Emergent Clustering**: Particles with similar affinities may cluster when fields pass
4. **Energy Competition**: Particles compete for favorable field positions
5. **Evolutionary Pressure**: Mutations that change type affect field affinity and survival

## Controls

### UI Sliders
- **Field Energy Slider**: Controls energy absorption rate (0-20/s)
- **Chemotaxis Slider**: Controls movement force strength (0-200)
- **Spawn Count**: Number of particles to spawn (10-200)
- **World Size**: Arena size multiplier (0.5x-2x)

### Camera Controls
- **Mouse Drag**: Click and drag to pan camera
- **Mouse Scroll**: Zoom in/out
- **Arrow Keys**: Pan camera (←↑↓→)
- **= Key**: Zoom in
- **- Key**: Zoom out

### Other Keys
- **R**: Restart simulation with current settings
- **H**: Toggle UI visibility

## Visual Details
- Fields render at **20% opacity** for subtle atmospheric effect
- Field sizes vary from small (15 units) to large (50 units)
- Energy strength has **quadratic falloff** from center to edge
