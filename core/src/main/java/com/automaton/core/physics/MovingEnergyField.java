package com.automaton.core.physics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;

/**
 * Represents a moving energy field with a specific type/color.
 * Fields move across the world like environmental waves.
 * Each field type attracts/repels different particle types.
 */
public class MovingEnergyField {
    public enum FieldType {
        // All different shades of blue with reduced saturation
        DEEP_BLUE(new Color(0.2f, 0.3f, 0.5f, 1f)),      // Dark blue
        SKY_BLUE(new Color(0.3f, 0.4f, 0.6f, 1f)),       // Medium-light blue
        CYAN_BLUE(new Color(0.25f, 0.45f, 0.55f, 1f)),   // Cyan-tinted blue
        STEEL_BLUE(new Color(0.3f, 0.35f, 0.5f, 1f));    // Gray-blue

        public final Color color;
        
        FieldType(Color color) {
            this.color = color;
        }
        
        /**
         * Get the affinity (attraction/repulsion) for a particle type.
         * Positive = attracted, Negative = repelled, 0 = neutral
         */
        public float getAffinityFor(ParticleType particleType) {
            switch (this) {
                case DEEP_BLUE:
                    // Deep blue field: attracts AND, NAND gates; repels TRUE, FALSE states
                    if (particleType == ParticleType.AND) return 1.0f;
                    if (particleType == ParticleType.NAND) return 0.8f;
                    if (particleType == ParticleType.TRUE) return -0.6f;
                    if (particleType == ParticleType.FALSE) return -0.4f;
                    return 0.2f;
                    
                case SKY_BLUE:
                    // Sky blue field: attracts TRUE state and OR gates; repels FALSE
                    if (particleType == ParticleType.TRUE) return 1.0f;
                    if (particleType == ParticleType.OR) return 0.8f;
                    if (particleType == ParticleType.FALSE) return -0.8f;
                    if (particleType == ParticleType.NOR) return -0.5f;
                    return 0.1f;
                    
                case CYAN_BLUE:
                    // Cyan blue field: attracts NOT, NOR gates; repels XOR, XNOR
                    if (particleType == ParticleType.NOT) return 1.0f;
                    if (particleType == ParticleType.NOR) return 0.7f;
                    if (particleType == ParticleType.XOR) return -0.7f;
                    if (particleType == ParticleType.XNOR) return -0.5f;
                    return 0.0f;
                    
                case STEEL_BLUE:
                    // Steel blue field: attracts XOR, XNOR; neutral to logic states
                    if (particleType == ParticleType.XOR) return 1.0f;
                    if (particleType == ParticleType.XNOR) return 0.9f;
                    if (particleType == ParticleType.FALSE) return 0.3f;
                    return 0.0f;
            }
            return 0.0f;
        }
    }
    
    private final FieldType type;
    private final Vector2 position; // Center position
    private final Vector2 velocity; // Movement direction/speed
    private float radius;
    private float strength; // Energy intensity (0-1)
    private boolean active;
    
    // World boundaries for respawning
    private float worldWidth;
    private float worldHeight;
    
    public MovingEnergyField(FieldType type, float worldWidth, float worldHeight) {
        this.type = type;
        this.position = new Vector2();
        this.velocity = new Vector2();
        this.worldWidth = worldWidth;
        this.worldHeight = worldHeight;
        this.active = false;
        
        // Random initial properties
        randomize();
    }
    
    /**
     * Randomize field properties and spawn from edge.
     */
    public void randomize() {
        // Random radius with more variation (15-50 units)
        this.radius = MathUtils.random(15f, 50f);
        
        // Random strength
        this.strength = MathUtils.random(0.5f, 1.0f);
        
        // Spawn from random edge
        int edge = MathUtils.random(3); // 0=left, 1=right, 2=top, 3=bottom
        float halfWidth = worldWidth * 0.5f;
        float halfHeight = worldHeight * 0.5f;
        
        switch (edge) {
            case 0: // Left edge
                position.set(-halfWidth - radius, MathUtils.random(-halfHeight, halfHeight));
                velocity.set(MathUtils.random(5f, 15f), MathUtils.random(-5f, 5f));
                break;
            case 1: // Right edge
                position.set(halfWidth + radius, MathUtils.random(-halfHeight, halfHeight));
                velocity.set(MathUtils.random(-15f, -5f), MathUtils.random(-5f, 5f));
                break;
            case 2: // Top edge
                position.set(MathUtils.random(-halfWidth, halfWidth), halfHeight + radius);
                velocity.set(MathUtils.random(-5f, 5f), MathUtils.random(-15f, -5f));
                break;
            case 3: // Bottom edge
                position.set(MathUtils.random(-halfWidth, halfWidth), -halfHeight - radius);
                velocity.set(MathUtils.random(-5f, 5f), MathUtils.random(5f, 15f));
                break;
        }
        
        this.active = true;
    }
    
    /**
     * Update field position.
     * @param delta Time step in seconds
     */
    public void update(float delta) {
        if (!active) return;
        
        position.x += velocity.x * delta;
        position.y += velocity.y * delta;
        
        // Check if field has left the world bounds (with margin)
        float halfWidth = worldWidth * 0.5f;
        float halfHeight = worldHeight * 0.5f;
        float margin = radius + 10f;
        
        if (position.x < -halfWidth - margin || position.x > halfWidth + margin ||
            position.y < -halfHeight - margin || position.y > halfHeight + margin) {
            // Respawn from another edge
            randomize();
        }
    }
    
    /**
     * Get energy level at a world position from this field.
     * Returns 0-1 based on distance from field center with quadratic falloff
     * for stronger effects at center that fade to weak at edges.
     */
    public float getEnergyAt(float worldX, float worldY) {
        if (!active) return 0f;
        
        float dx = worldX - position.x;
        float dy = worldY - position.y;
        float distance = (float) Math.sqrt(dx * dx + dy * dy);
        
        if (distance >= radius) return 0f;
        
        // Quadratic falloff: very strong at center, fades to weak at edges
        float normalizedDist = distance / radius;
        float falloff = 1.0f - normalizedDist;
        // Square the falloff for more dramatic center-to-edge gradient
        return falloff * falloff * strength;
    }
    
    /**
     * Get gradient direction at a world position (normalized vector pointing toward higher energy).
     */
    public void getGradientDirection(float worldX, float worldY, float[] outDirection) {
        if (!active) {
            outDirection[0] = 0f;
            outDirection[1] = 0f;
            return;
        }
        
        float dx = position.x - worldX;
        float dy = position.y - worldY;
        float distance = (float) Math.sqrt(dx * dx + dy * dy);
        
        if (distance < 0.001f || distance >= radius) {
            outDirection[0] = 0f;
            outDirection[1] = 0f;
            return;
        }
        
        // Normalize direction toward center
        outDirection[0] = dx / distance;
        outDirection[1] = dy / distance;
    }
    
    public FieldType getType() {
        return type;
    }
    
    public Vector2 getPosition() {
        return position;
    }
    
    public float getRadius() {
        return radius;
    }
    
    public float getStrength() {
        return strength;
    }
    
    public boolean isActive() {
        return active;
    }
    
    public void setActive(boolean active) {
        this.active = active;
    }
    
    public void setWorldSize(float worldWidth, float worldHeight) {
        this.worldWidth = worldWidth;
        this.worldHeight = worldHeight;
    }
}
