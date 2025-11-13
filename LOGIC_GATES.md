# Logic Gate Particle System

## Overview
The simulation now implements all classical logic gates as particle types. Particles represent either logic states (TRUE/FALSE) or logic gates (AND, OR, NOT, NAND, NOR, XOR, XNOR).

## Particle Types

### Logic States
- **TRUE** (Green) - Represents logic HIGH (1)
- **FALSE** (Gray) - Represents logic LOW (0)

### Logic Gates
- **AND** (Red) - Output TRUE only when both inputs are TRUE
- **OR** (Orange) - Output TRUE when at least one input is TRUE
- **NOT** (Purple) - Inverts the input (TRUE → FALSE, FALSE → TRUE)
- **NAND** (Pink) - Output FALSE only when both inputs are TRUE (NOT AND)
- **NOR** (Cyan) - Output TRUE only when both inputs are FALSE (NOT OR)
- **XOR** (Yellow) - Output TRUE when inputs differ (exclusive OR)
- **XNOR** (Magenta) - Output TRUE when inputs match (exclusive NOR)

## Truth Tables

### AND Gate
| Input A | Input B | Output |
|---------|---------|--------|
| FALSE   | FALSE   | FALSE  |
| FALSE   | TRUE    | FALSE  |
| TRUE    | FALSE   | FALSE  |
| TRUE    | TRUE    | TRUE   |

### OR Gate
| Input A | Input B | Output |
|---------|---------|--------|
| FALSE   | FALSE   | FALSE  |
| FALSE   | TRUE    | TRUE   |
| TRUE    | FALSE   | TRUE   |
| TRUE    | TRUE    | TRUE   |

### NOT Gate
| Input   | Output |
|---------|--------|
| FALSE   | TRUE   |
| TRUE    | FALSE  |

### NAND Gate
| Input A | Input B | Output |
|---------|---------|--------|
| FALSE   | FALSE   | TRUE   |
| FALSE   | TRUE    | TRUE   |
| TRUE    | FALSE   | TRUE   |
| TRUE    | TRUE    | FALSE  |

### NOR Gate
| Input A | Input B | Output |
|---------|---------|--------|
| FALSE   | FALSE   | TRUE   |
| FALSE   | TRUE    | FALSE  |
| TRUE    | FALSE   | FALSE  |
| TRUE    | TRUE    | FALSE  |

### XOR Gate
| Input A | Input B | Output |
|---------|---------|--------|
| FALSE   | FALSE   | FALSE  |
| FALSE   | TRUE    | TRUE   |
| TRUE    | FALSE   | TRUE   |
| TRUE    | TRUE    | FALSE  |

### XNOR Gate
| Input A | Input B | Output |
|---------|---------|--------|
| FALSE   | FALSE   | TRUE   |
| FALSE   | TRUE    | FALSE  |
| TRUE    | FALSE   | FALSE  |
| TRUE    | TRUE    | TRUE   |

## Behavior in Simulation

### Bonding Rules
- Logic states (TRUE/FALSE) can bond with each other
- Logic gates can bond with logic states
- Logic gates can bond with other logic gates
- Gate outputs are evaluated based on their connected inputs

### Violent Reactions
- **NOR** gate violently repels TRUE inputs (creates explosive force)
- **NAND** gate violently repels TRUE inputs
- These reactions simulate gate rejection of invalid logic conditions

### Special Properties
- **XOR** gates have controllable friction via the UI slider
- All gates evaluate their logic based on nearby particles
- Bond formation follows classical logic gate truth tables

## Color Coding
Each particle type has a unique color for easy identification:
- **Green** = TRUE state
- **Gray** = FALSE state
- **Red** = AND gate
- **Orange** = OR gate
- **Purple** = NOT gate
- **Pink** = NAND gate
- **Cyan** = NOR gate
- **Yellow** = XOR gate
- **Magenta** = XNOR gate

## Controls
- **R** - Restart simulation with new random particles
- **H** - Hide/show UI
- **Left-click drag** - Pan camera
- **Scroll wheel** - Zoom camera
- **Slider** - Adjust XOR gate friction

## Technical Implementation
The logic gate system is implemented in `ParticleType.java`:
- Each type has a color and logic state flag
- `canBondWith()` determines valid particle interactions
- `evaluateGate()` implements truth table logic
- `isViolentReaction()` creates repulsive forces for invalid combinations
