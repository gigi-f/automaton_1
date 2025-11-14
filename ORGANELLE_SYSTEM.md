# Organelle System Implementation

## Overview
Implemented a cellular biology-inspired organelle system where molecules can absorb GREEN (TRUE) particles as internal "organelles" that strengthen the molecular structure and generate energy - similar to chloroplasts in cells.

## System Components

### 1. Particle Organelle Tracking
**File:** `Particle.java`
- Added `isOrganelle` boolean field
- Organelles are marked particles that:
  - Don't form bonds
  - Are spatially trapped within molecules
  - Generate energy for surrounding particles
  - Have maximum energy (100)

### 2. Organelle Absorption
**File:** `PhysicsWorld.java` - `processOrganelleAbsorption()`
- Detects GREEN particles entering molecular interiors (3+ bonded particles)
- Uses convex hull and ray-casting algorithms to determine containment
- Converts captured GREEN particles into organelles
- Slows organelle movement dramatically (90% velocity reduction)
- Called every physics step after corpse consumption

### 3. Bond Strengthening
**File:** `Bond.java` - `getEffectiveBreakForce(Array<Particle>)`
- Each organelle within 4.0 units multiplies bond strength by 1.5x
- Exponential scaling: strength × (1.5^organelleCount)
- Examples:
  - 1 organelle: 1.5× strength
  - 2 organelles: 2.25× strength
  - 3 organelles: 3.375× strength
  - 4 organelles: 5.06× strength
- Applied during bond constraint evaluation in physics step

### 4. Energy Generation
**File:** `PhysicsWorld.java` - `applyOrganelleEnergyGeneration()`
- Base generation: 8.0 energy per organelle per second
- Scaling with diminishing returns: count × (1 - 0.05 × count)
  - 1 organelle: 1.0× (8.0 energy/sec)
  - 2 organelles: 1.9× (15.2 energy/sec)
  - 3 organelles: 2.7× (21.6 energy/sec)
  - 5 organelles: 3.75× (30.0 energy/sec)
- Energy distributed equally to all particles in molecule
- Organelle membership determined by spatial proximity (5.0 unit range)

### 5. Visual Distinction
**File:** `ParticleRenderer.java`
- Organelles render with pulsing glow effect
- Pulse frequency: 3 Hz sine wave
- Brightness oscillates between 1.0× and 1.5×
- Outer glow ring at 1.4× particle radius
- Semi-transparent aura (30% opacity × pulse)

## Biological Parallels

| Organelle Feature | Biological Analog |
|-------------------|-------------------|
| Absorption into cell | Endosymbiosis (mitochondria/chloroplast origin) |
| Energy generation | Photosynthesis (chloroplasts) |
| Bond strengthening | Structural proteins / cytoskeleton support |
| Trapped inside | Membrane-bound compartments |
| GREEN particle preference | Chlorophyll pigmentation |

## Strategic Implications

1. **Evolutionary Advantage**: Molecules with organelles are stronger and self-sustaining
2. **Predation Strategy**: Hunting behavior now has dual purpose - food AND organelle capture
3. **Size Management**: Organelles counter metabolic scaling penalties for large molecules
4. **Stability**: Exponential bond strengthening prevents fragmentation under stress
5. **Green Particle Dynamics**: TRUE particles now have dual role as logic gates AND potential organelles

## Performance Considerations

- Organelle detection: O(n×m) where n=molecules, m=GREEN particles
- Bond strengthening: O(bonds×organelles) per physics step
- Energy generation: O(molecules×organelles) per physics step
- All algorithms use spatial proximity checks to limit computation

## Parameters

| Parameter | Value | Location | Purpose |
|-----------|-------|----------|---------|
| `ORGANELLE_RANGE` (absorption) | 4.0 units | PhysicsWorld.processOrganelleAbsorption | Containment detection radius |
| `ORGANELLE_RANGE` (energy) | 5.0 units | PhysicsWorld.applyOrganelleEnergyGeneration | Energy distribution radius |
| `STRENGTH_PER_ORGANELLE` | 1.5× | Bond.getEffectiveBreakForce | Bond strength multiplier |
| `ENERGY_PER_ORGANELLE` | 8.0/sec | PhysicsWorld.applyOrganelleEnergyGeneration | Base energy generation rate |
| Velocity reduction | 90% | PhysicsWorld.processOrganelleAbsorption | Traps organelle in place |

## Integration with Existing Systems

- **Hunting Behavior**: Weak molecules now seek GREEN particles for dual benefit (food + organelles)
- **Metabolic Scaling**: Organelles offset increased energy costs of large molecules
- **Mitosis**: Strong bonds from organelles reduce fragmentation, allowing larger stable structures
- **Energy Fields**: Organelles provide alternative energy source independent of field proximity
- **Cell Interior Detection**: Reuses convex hull algorithms from corpse consumption system

## Future Enhancements

Possible extensions:
1. Different particle types as specialized organelles (e.g., NOT gate for energy storage)
2. Organelle replication when molecule divides
3. Maximum organelle capacity per molecule
4. Organelle decay/aging mechanics
5. Organelle ejection under extreme stress
