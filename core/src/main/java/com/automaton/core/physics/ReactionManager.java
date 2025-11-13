package com.automaton.core.physics;

import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.utils.Array;

import java.util.Random;

/**
 * Manages stochastic reaction rules for bond formation and breaking.
 * Uses seedable RNG for deterministic randomness.
 * 
 * Reaction rules:
 * - Bond Formation: A + B -> A-B (with probability p when within distance d)
 * - Bond Breaking: A-B -> A + B (based on stress threshold or reaction rules)
 */
public class ReactionManager {
    private final PhysicsWorld physicsWorld;
    private final Random random;
    
    // Reaction parameters
    private float bondFormationRadius = 3.0f;
    private float bondFormationProbability = 0.01f;
    private float bondBreakProbability = 0.001f;
    
    // Default bond properties for reactions
    private BondType defaultBondType = BondType.ELASTIC;
    private float defaultStiffness = 50f;
    private float defaultDamping = 2f;
    private float defaultBreakForce = 200f;
    
    public ReactionManager(PhysicsWorld physicsWorld, long seed) {
        this.physicsWorld = physicsWorld;
        this.random = new Random(seed);
    }
    
    /**
     * Process reaction rules for all particles.
     * Should be called each tick after physics step.
     */
    public void processReactions() {
        processBondFormation();
        processBondBreaking();
    }
    
    /**
     * Bond formation rule: nearby unbonded particles may form bonds.
     * Uses spatial hashing for efficient neighbor queries.
     */
    private void processBondFormation() {
        Array<Particle> particles = physicsWorld.getActiveParticles();
        
        for (Particle particle : particles) {
            if (!particle.isActive()) continue;
            
            // Query nearby particles using spatial hash grid
            Array<Particle> neighbors = physicsWorld.queryNeighbors(particle, bondFormationRadius);
            
            for (Particle neighbor : neighbors) {
                if (!neighbor.isActive()) continue;
                
                // Check if already bonded
                if (areBonded(particle, neighbor)) continue;
                
                // Check if particle types can bond
                if (!particle.getType().canBondWith(neighbor.getType())) {
                    // Types cannot bond - check for violent reaction
                    if (particle.getType().isViolentReaction(neighbor.getType())) {
                        applyViolentReaction(particle, neighbor);
                    }
                    continue;
                }
                
                // Stochastic bond formation
                if (random.nextFloat() < bondFormationProbability) {
                    createReactionBond(particle, neighbor);
                }
            }
        }
    }
    
    /**
     * Bond breaking rule: bonds may spontaneously break (beyond stress threshold).
     * This simulates chemical instability or external factors.
     */
    private void processBondBreaking() {
        Array<Bond> bonds = physicsWorld.getActiveBonds();
        
        for (int i = bonds.size - 1; i >= 0; i--) {
            Bond bond = bonds.get(i);
            if (!bond.isActive()) continue;
            
            // Stochastic bond breaking
            if (random.nextFloat() < bondBreakProbability) {
                physicsWorld.destroyBond(bond);
            }
        }
    }
    
    /**
     * Check if two particles are already bonded.
     */
    private boolean areBonded(Particle a, Particle b) {
        Array<Bond> bonds = physicsWorld.getActiveBonds();
        for (Bond bond : bonds) {
            if (!bond.isActive()) continue;
            if ((bond.getParticleA() == a && bond.getParticleB() == b) ||
                (bond.getParticleA() == b && bond.getParticleB() == a)) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * Create a bond between two particles using default reaction parameters.
     */
    private void createReactionBond(Particle a, Particle b) {
        physicsWorld.createBond(a, b, defaultBondType, defaultStiffness, 
                                defaultDamping, defaultBreakForce);
    }
    
    /**
     * Apply violent repulsive force when incompatible particles collide (e.g. yellow-yellow).
     */
    private void applyViolentReaction(Particle a, Particle b) {
        float violentForce = 2000f;  // Much stronger than normal crossing repulsion
        
        // Calculate direction from a to b
        float dx = b.getPosition().x - a.getPosition().x;
        float dy = b.getPosition().y - a.getPosition().y;
        float distSq = dx * dx + dy * dy;
        
        if (distSq < 0.001f) return;  // Too close, avoid division by zero
        
        float dist = (float) Math.sqrt(distSq);
        float invDist = 1.0f / dist;
        
        // Normalize and scale by force
        float fx = (dx * invDist) * violentForce;
        float fy = (dy * invDist) * violentForce;
        
        // Apply opposite forces
        a.getBody().applyForceToCenter(-fx, -fy, true);
        b.getBody().applyForceToCenter(fx, fy, true);
    }
    
    // Getters and setters for reaction parameters
    
    public float getBondFormationRadius() {
        return bondFormationRadius;
    }
    
    public void setBondFormationRadius(float radius) {
        this.bondFormationRadius = radius;
    }
    
    public float getBondFormationProbability() {
        return bondFormationProbability;
    }
    
    public void setBondFormationProbability(float probability) {
        this.bondFormationProbability = MathUtils.clamp(probability, 0f, 1f);
    }
    
    public float getBondBreakProbability() {
        return bondBreakProbability;
    }
    
    public void setBondBreakProbability(float probability) {
        this.bondBreakProbability = MathUtils.clamp(probability, 0f, 1f);
    }
    
    public void setDefaultBondProperties(BondType type, float stiffness, 
                                        float damping, float breakForce) {
        this.defaultBondType = type;
        this.defaultStiffness = stiffness;
        this.defaultDamping = damping;
        this.defaultBreakForce = breakForce;
    }
    
    /**
     * Reseed the random number generator for reproducibility.
     */
    public void reseed(long seed) {
        random.setSeed(seed);
    }
}
