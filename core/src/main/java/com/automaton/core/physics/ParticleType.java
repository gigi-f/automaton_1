package com.automaton.core.physics;

import com.badlogic.gdx.graphics.Color;

/**
 * Enumerates particle types with reaction rules.
 * RED: Cannot connect to BLUE
 * BLUE: Cannot connect to RED
 * YELLOW: Can connect to RED and BLUE, but not to other YELLOW
 */
public enum ParticleType {
    RED(new Color(1.0f, 0.2f, 0.2f, 1.0f)),      // Red
    BLUE(new Color(0.2f, 0.5f, 1.0f, 1.0f)),     // Blue
    YELLOW(new Color(1.0f, 0.9f, 0.2f, 1.0f));   // Yellow
    
    private final Color color;
    
    ParticleType(Color color) {
        this.color = color;
    }
    
    public Color getColor() {
        return color;
    }
    
    /**
     * Check if this particle type can bond with another type.
     */
    public boolean canBondWith(ParticleType other) {
        if (this == RED && other == BLUE) return false;
        if (this == BLUE && other == RED) return false;
        if (this == YELLOW && other == YELLOW) return false;
        return true;
    }
    
    /**
     * Check if reaction between these types should be violent (strong repulsion).
     */
    public boolean isViolentReaction(ParticleType other) {
        return (this == YELLOW && other == YELLOW);
    }
}
