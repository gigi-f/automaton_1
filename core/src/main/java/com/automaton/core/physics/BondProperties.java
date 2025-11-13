package com.automaton.core.physics;

/**
 * Manages bond property multipliers for different particle type combinations.
 * All multipliers are exposed as public fields for UI slider control.
 * Base values: restLength=1.5, stiffness=80, damping=3, breakForce=200
 */
public class BondProperties {
    
    // === Logic State Connections ===
    
    // TRUE ↔ TRUE (Power Connection)
    public float trueTrueLengthMult = 1.0f;
    public float trueTrueStiffnessMult = 2.0f;
    public float trueTrueBreakMult = 2.0f;
    
    // FALSE ↔ FALSE (Ground Connection)
    public float falseFalseLengthMult = 0.8f;
    public float falseFalseStiffnessMult = 1.5f;
    public float falseFalseBreakMult = 1.5f;
    
    // TRUE ↔ FALSE (Voltage Differential)
    public float trueFalseLengthMult = 2.0f;
    public float trueFalseStiffnessMult = 0.5f;
    public float trueFalseBreakMult = 0.5f;
    
    // === AND Gate Connections ===
    
    public float andTrueLengthMult = 1.2f;
    public float andTrueStiffnessMult = 1.5f;
    public float andTrueBreakMult = 1.8f;
    
    public float andFalseLengthMult = 1.0f;
    public float andFalseStiffnessMult = 0.8f;
    public float andFalseBreakMult = 0.8f;
    
    // === OR Gate Connections ===
    
    public float orTrueLengthMult = 1.5f;
    public float orTrueStiffnessMult = 1.2f;
    public float orTrueBreakMult = 1.5f;
    
    public float orFalseLengthMult = 1.3f;
    public float orFalseStiffnessMult = 1.0f;
    public float orFalseBreakMult = 1.2f;
    
    // === NOT Gate Connections ===
    
    public float notTrueLengthMult = 2.5f;
    public float notTrueStiffnessMult = 0.3f;
    public float notTrueBreakMult = 0.6f;
    
    public float notFalseLengthMult = 1.0f;
    public float notFalseStiffnessMult = 2.0f;
    public float notFalseBreakMult = 2.0f;
    
    // === NAND Gate Connections ===
    
    public float nandTrueLengthMult = 1.8f;
    public float nandTrueStiffnessMult = 0.6f;
    public float nandTrueBreakMult = 0.7f;
    
    public float nandFalseLengthMult = 1.1f;
    public float nandFalseStiffnessMult = 1.3f;
    public float nandFalseBreakMult = 1.4f;
    
    // === NOR Gate Connections ===
    
    public float norTrueLengthMult = 3.0f;
    public float norTrueStiffnessMult = 0.2f;
    public float norTrueBreakMult = 0.3f;
    
    public float norFalseLengthMult = 0.9f;
    public float norFalseStiffnessMult = 2.5f;
    public float norFalseBreakMult = 2.5f;
    
    // === XOR Gate Connections ===
    
    public float xorTrueLengthMult = 1.4f;
    public float xorTrueStiffnessMult = 0.9f;
    public float xorTrueBreakMult = 1.0f;
    
    public float xorFalseLengthMult = 1.4f;
    public float xorFalseStiffnessMult = 0.9f;
    public float xorFalseBreakMult = 1.0f;
    
    // === XNOR Gate Connections ===
    
    public float xnorTrueLengthMult = 1.0f;
    public float xnorTrueStiffnessMult = 1.5f;
    public float xnorTrueBreakMult = 1.6f;
    
    public float xnorFalseLengthMult = 1.0f;
    public float xnorFalseStiffnessMult = 1.5f;
    public float xnorFalseBreakMult = 1.6f;
    
    // === Gate-to-Gate Connections ===
    
    // AND ↔ OR
    public float andOrLengthMult = 2.0f;
    public float andOrStiffnessMult = 0.8f;
    public float andOrBreakMult = 1.2f;
    
    // AND ↔ NAND (Inverse pair)
    public float andNandLengthMult = 4.0f;
    public float andNandStiffnessMult = 0.4f;
    public float andNandBreakMult = 0.5f;
    
    // OR ↔ NOR (Inverse pair)
    public float orNorLengthMult = 4.0f;
    public float orNorStiffnessMult = 0.4f;
    public float orNorBreakMult = 0.5f;
    
    // XOR ↔ XNOR (Inverse pair)
    public float xorXnorLengthMult = 5.0f;
    public float xorXnorStiffnessMult = 0.3f;
    public float xorXnorBreakMult = 0.4f;
    
    // NOT ↔ NOT (Double negative)
    public float notNotLengthMult = 2.5f;
    public float notNotStiffnessMult = 0.5f;
    public float notNotBreakMult = 0.8f;
    
    // XOR ↔ OR
    public float xorOrLengthMult = 1.6f;
    public float xorOrStiffnessMult = 1.0f;
    public float xorOrBreakMult = 1.1f;
    
    // NAND ↔ NOR
    public float nandNorLengthMult = 1.8f;
    public float nandNorStiffnessMult = 0.9f;
    public float nandNorBreakMult = 1.0f;
    
    // Same Gate ↔ Same Gate
    public float sameGateLengthMult = 1.2f;
    public float sameGateStiffnessMult = 1.3f;
    public float sameGateBreakMult = 1.5f;
    
    // Default for undefined combinations
    public float defaultLengthMult = 1.0f;
    public float defaultStiffnessMult = 1.0f;
    public float defaultBreakMult = 1.0f;
    
    /**
     * Get bond multipliers for a specific particle type combination.
     * Returns [lengthMult, stiffnessMult, breakMult]
     */
    public float[] getMultipliers(ParticleType typeA, ParticleType typeB) {
        // Normalize order (smaller ordinal first) for consistent lookup
        ParticleType t1 = typeA.ordinal() < typeB.ordinal() ? typeA : typeB;
        ParticleType t2 = typeA.ordinal() < typeB.ordinal() ? typeB : typeA;
        
        // Logic State Connections
        if (t1 == ParticleType.TRUE && t2 == ParticleType.TRUE) {
            return new float[]{trueTrueLengthMult, trueTrueStiffnessMult, trueTrueBreakMult};
        }
        if (t1 == ParticleType.FALSE && t2 == ParticleType.FALSE) {
            return new float[]{falseFalseLengthMult, falseFalseStiffnessMult, falseFalseBreakMult};
        }
        if (t1 == ParticleType.FALSE && t2 == ParticleType.TRUE) {
            return new float[]{trueFalseLengthMult, trueFalseStiffnessMult, trueFalseBreakMult};
        }
        
        // AND Gate Connections
        if (t1 == ParticleType.TRUE && t2 == ParticleType.AND) {
            return new float[]{andTrueLengthMult, andTrueStiffnessMult, andTrueBreakMult};
        }
        if (t1 == ParticleType.FALSE && t2 == ParticleType.AND) {
            return new float[]{andFalseLengthMult, andFalseStiffnessMult, andFalseBreakMult};
        }
        
        // OR Gate Connections
        if (t1 == ParticleType.TRUE && t2 == ParticleType.OR) {
            return new float[]{orTrueLengthMult, orTrueStiffnessMult, orTrueBreakMult};
        }
        if (t1 == ParticleType.FALSE && t2 == ParticleType.OR) {
            return new float[]{orFalseLengthMult, orFalseStiffnessMult, orFalseBreakMult};
        }
        
        // NOT Gate Connections
        if (t1 == ParticleType.TRUE && t2 == ParticleType.NOT) {
            return new float[]{notTrueLengthMult, notTrueStiffnessMult, notTrueBreakMult};
        }
        if (t1 == ParticleType.FALSE && t2 == ParticleType.NOT) {
            return new float[]{notFalseLengthMult, notFalseStiffnessMult, notFalseBreakMult};
        }
        
        // NAND Gate Connections
        if (t1 == ParticleType.TRUE && t2 == ParticleType.NAND) {
            return new float[]{nandTrueLengthMult, nandTrueStiffnessMult, nandTrueBreakMult};
        }
        if (t1 == ParticleType.FALSE && t2 == ParticleType.NAND) {
            return new float[]{nandFalseLengthMult, nandFalseStiffnessMult, nandFalseBreakMult};
        }
        
        // NOR Gate Connections
        if (t1 == ParticleType.TRUE && t2 == ParticleType.NOR) {
            return new float[]{norTrueLengthMult, norTrueStiffnessMult, norTrueBreakMult};
        }
        if (t1 == ParticleType.FALSE && t2 == ParticleType.NOR) {
            return new float[]{norFalseLengthMult, norFalseStiffnessMult, norFalseBreakMult};
        }
        
        // XOR Gate Connections
        if (t1 == ParticleType.TRUE && t2 == ParticleType.XOR) {
            return new float[]{xorTrueLengthMult, xorTrueStiffnessMult, xorTrueBreakMult};
        }
        if (t1 == ParticleType.FALSE && t2 == ParticleType.XOR) {
            return new float[]{xorFalseLengthMult, xorFalseStiffnessMult, xorFalseBreakMult};
        }
        
        // XNOR Gate Connections
        if (t1 == ParticleType.TRUE && t2 == ParticleType.XNOR) {
            return new float[]{xnorTrueLengthMult, xnorTrueStiffnessMult, xnorTrueBreakMult};
        }
        if (t1 == ParticleType.FALSE && t2 == ParticleType.XNOR) {
            return new float[]{xnorFalseLengthMult, xnorFalseStiffnessMult, xnorFalseBreakMult};
        }
        
        // Gate-to-Gate Connections
        if (t1 == ParticleType.AND && t2 == ParticleType.OR) {
            return new float[]{andOrLengthMult, andOrStiffnessMult, andOrBreakMult};
        }
        if (t1 == ParticleType.AND && t2 == ParticleType.NAND) {
            return new float[]{andNandLengthMult, andNandStiffnessMult, andNandBreakMult};
        }
        if (t1 == ParticleType.OR && t2 == ParticleType.NOR) {
            return new float[]{orNorLengthMult, orNorStiffnessMult, orNorBreakMult};
        }
        if (t1 == ParticleType.XOR && t2 == ParticleType.XNOR) {
            return new float[]{xorXnorLengthMult, xorXnorStiffnessMult, xorXnorBreakMult};
        }
        if (t1 == ParticleType.NOT && t2 == ParticleType.NOT) {
            return new float[]{notNotLengthMult, notNotStiffnessMult, notNotBreakMult};
        }
        if (t1 == ParticleType.OR && t2 == ParticleType.XOR) {
            return new float[]{xorOrLengthMult, xorOrStiffnessMult, xorOrBreakMult};
        }
        if (t1 == ParticleType.NAND && t2 == ParticleType.NOR) {
            return new float[]{nandNorLengthMult, nandNorStiffnessMult, nandNorBreakMult};
        }
        
        // Same gate type
        if (t1 == t2 && t1.isGate()) {
            return new float[]{sameGateLengthMult, sameGateStiffnessMult, sameGateBreakMult};
        }
        
        // Default fallback
        return new float[]{defaultLengthMult, defaultStiffnessMult, defaultBreakMult};
    }
}
