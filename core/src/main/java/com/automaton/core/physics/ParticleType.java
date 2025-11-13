package com.automaton.core.physics;

import com.badlogic.gdx.graphics.Color;

/**
 * Enumerates particle types representing logic gates and states.
 * Logic Gate Implementation:
 * - TRUE (1): Green - represents logic HIGH
 * - FALSE (0): Gray - represents logic LOW
 * - AND: Red - bonds with TRUE+TRUE only
 * - OR: Orange - bonds with any TRUE
 * - NOT: Purple - inverts input
 * - NAND: Pink - bonds unless both TRUE
 * - NOR: Cyan - bonds only if both FALSE
 * - XOR: Yellow - bonds with one TRUE, one FALSE
 * - XNOR: Magenta - bonds with matching states
 */
public enum ParticleType {
    // Logic States
    TRUE(new Color(0.2f, 1.0f, 0.2f, 1.0f), true),        // Green - logic 1
    FALSE(new Color(0.4f, 0.4f, 0.4f, 1.0f), false),      // Gray - logic 0
    
    // Logic Gates
    AND(new Color(1.0f, 0.2f, 0.2f, 1.0f), false),        // Red
    OR(new Color(1.0f, 0.6f, 0.2f, 1.0f), false),         // Orange
    NOT(new Color(0.6f, 0.2f, 1.0f, 1.0f), false),        // Purple
    NAND(new Color(1.0f, 0.4f, 0.6f, 1.0f), false),       // Pink
    NOR(new Color(0.2f, 0.8f, 0.8f, 1.0f), false),        // Cyan
    XOR(new Color(1.0f, 0.9f, 0.2f, 1.0f), false),        // Yellow
    XNOR(new Color(1.0f, 0.2f, 1.0f, 1.0f), false);       // Magenta
    
    private final Color color;
    private final boolean logicState;  // For TRUE/FALSE types
    
    ParticleType(Color color, boolean logicState) {
        this.color = color;
        this.logicState = logicState;
    }
    
    public Color getColor() {
        return color;
    }
    
    public boolean isLogicState() {
        return this == TRUE || this == FALSE;
    }
    
    public boolean isGate() {
        return !isLogicState();
    }
    
    public boolean getLogicValue() {
        return logicState;
    }
    
    /**
     * Check if this particle type can bond with another type based on logic gate rules.
     * Gates bond with logic states based on their truth tables.
     * Logic states bond with gates and other logic states.
     */
    public boolean canBondWith(ParticleType other) {
        // If both are logic states, they can always bond
        if (this.isLogicState() && other.isLogicState()) {
            return true;
        }
        
        // Gates can bond with logic states
        if (this.isGate() && other.isLogicState()) {
            return true;
        }
        if (this.isLogicState() && other.isGate()) {
            return true;
        }
        
        // Gates can bond with other gates
        if (this.isGate() && other.isGate()) {
            return true;
        }
        
        return true;
    }
    
    /**
     * Evaluate logic gate with two inputs.
     * Returns true if the gate should create/maintain a bond.
     */
    public boolean evaluateGate(ParticleType input1, ParticleType input2) {
        if (!this.isGate()) return true;
        
        // Convert inputs to boolean (default to false for non-logic types)
        boolean a = input1 == TRUE;
        boolean b = input2 == TRUE;
        
        switch (this) {
            case AND:
                return a && b;
            case OR:
                return a || b;
            case NOT:
                return !a;  // NOT gate only considers first input
            case NAND:
                return !(a && b);
            case NOR:
                return !(a || b);
            case XOR:
                return a ^ b;  // XOR: true if inputs differ
            case XNOR:
                return !(a ^ b);  // XNOR: true if inputs match
            default:
                return true;
        }
    }
    
    /**
     * Check if reaction between these types should be violent (strong repulsion).
     * Gates reject invalid logic combinations with force.
     */
    public boolean isViolentReaction(ParticleType other) {
        // XOR gate violently rejects matching inputs
        if (this == XOR && other.isLogicState()) {
            return false;  // XOR handles this through bond breaking
        }
        
        // NOR gate violently rejects TRUE inputs
        if (this == NOR && other == TRUE) {
            return true;
        }
        
        // NAND gate violently rejects when both inputs are TRUE
        if (this == NAND && other == TRUE) {
            return true;
        }
        
        return false;
    }
}
