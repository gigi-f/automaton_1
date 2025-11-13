# Bond Properties System - UI Integration Guide

## Overview
The bond properties system allows dynamic control over bond behavior between different particle type combinations. All properties are exposed as public fields in the `BondProperties` class for easy UI slider integration.

## Accessing Bond Properties

```java
// In AutomatonGame or any class with access to PhysicsWorld
BondProperties props = physicsWorld.getBondProperties();

// Or through AutomatonGame directly
BondProperties props = game.getBondProperties();
```

## Property Structure

Each bond combination has three multipliers:
- **Length Multiplier** - Affects rest length of the bond
- **Stiffness Multiplier** - Affects rigidity (higher = more rigid, lower = more stretchy)
- **Break Force Multiplier** - Affects strength (higher = stronger, lower = more fragile)

Base values (before multipliers):
- Rest Length: 1.5 units (actual distance between particles at creation)
- Stiffness: 80
- Damping: 3 (not affected by multipliers currently)
- Break Force: 200

## Available Properties

### Logic State Connections

```java
// TRUE ↔ TRUE (Power Connection)
props.trueTrueLengthMult = 1.0f;      // Default: 1.0
props.trueTrueStiffnessMult = 2.0f;   // Default: 2.0
props.trueTrueBreakMult = 2.0f;       // Default: 2.0

// FALSE ↔ FALSE (Ground Connection)
props.falseFalseLengthMult = 0.8f;    // Default: 0.8
props.falseFalseStiffnessMult = 1.5f; // Default: 1.5
props.falseFalseBreakMult = 1.5f;     // Default: 1.5

// TRUE ↔ FALSE (Voltage Differential)
props.trueFalseLengthMult = 2.0f;     // Default: 2.0
props.trueFalseStiffnessMult = 0.5f;  // Default: 0.5
props.trueFalseBreakMult = 0.5f;      // Default: 0.5
```

### Gate to Logic State Connections

```java
// AND Gate
props.andTrueLengthMult = 1.2f;       // Default: 1.2
props.andTrueStiffnessMult = 1.5f;    // Default: 1.5
props.andTrueBreakMult = 1.8f;        // Default: 1.8

props.andFalseLengthMult = 1.0f;      // Default: 1.0
props.andFalseStiffnessMult = 0.8f;   // Default: 0.8
props.andFalseBreakMult = 0.8f;       // Default: 0.8

// OR Gate
props.orTrueLengthMult = 1.5f;        // Default: 1.5
props.orTrueStiffnessMult = 1.2f;     // Default: 1.2
props.orTrueBreakMult = 1.5f;         // Default: 1.5

props.orFalseLengthMult = 1.3f;       // Default: 1.3
props.orFalseStiffnessMult = 1.0f;    // Default: 1.0
props.orFalseBreakMult = 1.2f;        // Default: 1.2

// NOT Gate
props.notTrueLengthMult = 2.5f;       // Default: 2.5
props.notTrueStiffnessMult = 0.3f;    // Default: 0.3
props.notTrueBreakMult = 0.6f;        // Default: 0.6

props.notFalseLengthMult = 1.0f;      // Default: 1.0
props.notFalseStiffnessMult = 2.0f;   // Default: 2.0
props.notFalseBreakMult = 2.0f;       // Default: 2.0

// NAND Gate
props.nandTrueLengthMult = 1.8f;      // Default: 1.8
props.nandTrueStiffnessMult = 0.6f;   // Default: 0.6
props.nandTrueBreakMult = 0.7f;       // Default: 0.7

props.nandFalseLengthMult = 1.1f;     // Default: 1.1
props.nandFalseStiffnessMult = 1.3f;  // Default: 1.3
props.nandFalseBreakMult = 1.4f;      // Default: 1.4

// NOR Gate
props.norTrueLengthMult = 3.0f;       // Default: 3.0
props.norTrueStiffnessMult = 0.2f;    // Default: 0.2
props.norTrueBreakMult = 0.3f;        // Default: 0.3

props.norFalseLengthMult = 0.9f;      // Default: 0.9
props.norFalseStiffnessMult = 2.5f;   // Default: 2.5
props.norFalseBreakMult = 2.5f;       // Default: 2.5

// XOR Gate
props.xorTrueLengthMult = 1.4f;       // Default: 1.4
props.xorTrueStiffnessMult = 0.9f;    // Default: 0.9
props.xorTrueBreakMult = 1.0f;        // Default: 1.0

props.xorFalseLengthMult = 1.4f;      // Default: 1.4
props.xorFalseStiffnessMult = 0.9f;   // Default: 0.9
props.xorFalseBreakMult = 1.0f;       // Default: 1.0

// XNOR Gate
props.xnorTrueLengthMult = 1.0f;      // Default: 1.0
props.xnorTrueStiffnessMult = 1.5f;   // Default: 1.5
props.xnorTrueBreakMult = 1.6f;       // Default: 1.6

props.xnorFalseLengthMult = 1.0f;     // Default: 1.0
props.xnorFalseStiffnessMult = 1.5f;  // Default: 1.5
props.xnorFalseBreakMult = 1.6f;      // Default: 1.6
```

### Gate-to-Gate Connections

```java
// Complementary Gates
props.andOrLengthMult = 2.0f;         // Default: 2.0
props.andOrStiffnessMult = 0.8f;      // Default: 0.8
props.andOrBreakMult = 1.2f;          // Default: 1.2

// Inverse Pairs (Very Unstable)
props.andNandLengthMult = 4.0f;       // Default: 4.0
props.andNandStiffnessMult = 0.4f;    // Default: 0.4
props.andNandBreakMult = 0.5f;        // Default: 0.5

props.orNorLengthMult = 4.0f;         // Default: 4.0
props.orNorStiffnessMult = 0.4f;      // Default: 0.4
props.orNorBreakMult = 0.5f;          // Default: 0.5

props.xorXnorLengthMult = 5.0f;       // Default: 5.0 (Most extreme!)
props.xorXnorStiffnessMult = 0.3f;    // Default: 0.3
props.xorXnorBreakMult = 0.4f;        // Default: 0.4

// Special Combinations
props.notNotLengthMult = 2.5f;        // Default: 2.5
props.notNotStiffnessMult = 0.5f;     // Default: 0.5
props.notNotBreakMult = 0.8f;         // Default: 0.8

props.xorOrLengthMult = 1.6f;         // Default: 1.6
props.xorOrStiffnessMult = 1.0f;      // Default: 1.0
props.xorOrBreakMult = 1.1f;          // Default: 1.1

props.nandNorLengthMult = 1.8f;       // Default: 1.8
props.nandNorStiffnessMult = 0.9f;    // Default: 0.9
props.nandNorBreakMult = 1.0f;        // Default: 1.0

// Same Gate Type
props.sameGateLengthMult = 1.2f;      // Default: 1.2
props.sameGateStiffnessMult = 1.3f;   // Default: 1.3
props.sameGateBreakMult = 1.5f;       // Default: 1.5

// Default Fallback
props.defaultLengthMult = 1.0f;       // Default: 1.0
props.defaultStiffnessMult = 1.0f;    // Default: 1.0
props.defaultBreakMult = 1.0f;        // Default: 1.0
```

## Example UI Slider Integration

```java
// Create a slider for TRUE-TRUE bond length
Label label = new Label("TRUE-TRUE Length: 1.0x", skin);
Slider slider = new Slider(0.5f, 5.0f, 0.1f, false, skin);
slider.setValue(bondProps.trueTrueLengthMult);

slider.addListener(new ChangeListener() {
    @Override
    public void changed(ChangeEvent event, Actor actor) {
        bondProps.trueTrueLengthMult = slider.getValue();
        label.setText(String.format("TRUE-TRUE Length: %.1fx", slider.getValue()));
    }
});
```

## Recommended Slider Ranges

- **Length Multipliers**: 0.5x to 5.0x (step: 0.1)
- **Stiffness Multipliers**: 0.1x to 5.0x (step: 0.1)
- **Break Force Multipliers**: 0.1x to 5.0x (step: 0.1)

## Live Tuning

Properties can be changed at runtime:
- **Existing bonds** retain their original properties
- **New bonds** will use the updated multipliers
- To see changes immediately, restart the simulation with 'R' key

## Notable Combinations

**Most Stable:**
- NOR ↔ FALSE: The ground connection (0.9x length, 2.5x stiffness, 2.5x break)
- TRUE ↔ TRUE: Power line (1.0x length, 2.0x stiffness, 2.0x break)
- NOT ↔ FALSE: Stable inversion (1.0x length, 2.0x stiffness, 2.0x break)

**Most Unstable:**
- XOR ↔ XNOR: The oscillator (5.0x length, 0.3x stiffness, 0.4x break)
- NOR ↔ TRUE: The repeller (3.0x length, 0.2x stiffness, 0.3x break)
- AND ↔ NAND: Inverse gates (4.0x length, 0.4x stiffness, 0.5x break)

## Effects of Multipliers

**Length Multiplier:**
- < 1.0: Bonds try to compress to shorter rest length
- > 1.0: Bonds try to stretch to longer rest length
- Creates visual diversity and affects interaction distance

**Stiffness Multiplier:**
- < 1.0: Stretchy, bouncy bonds (softer springs)
- > 1.0: Rigid, tight bonds (harder springs)
- Affects how much bonds compress/extend under force

**Break Force Multiplier:**
- < 1.0: Fragile bonds that break easily
- > 1.0: Strong bonds that resist breaking
- Affects survival under stress and collisions
