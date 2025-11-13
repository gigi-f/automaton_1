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
    private float activationEnergy = 20f;  // Minimum energy required to form bonds
    
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
        
        // Use indexed loop to avoid nested iterator issues
        for (int i = 0; i < particles.size; i++) {
            Particle particle = particles.get(i);
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
                
                // Check activation energy - both particles need sufficient energy
                if (particle.getEnergy() < activationEnergy || 
                    neighbor.getEnergy() < activationEnergy) {
                    continue;  // Not enough energy to form bond
                }
                
                // Calculate catalytic boost from nearby catalyst particles
                float catalyticMultiplier = calculateCatalyticBoost(particle, neighbor);
                float effectiveProbability = bondFormationProbability * catalyticMultiplier;
                
                // Stochastic bond formation (with catalytic boost)
                if (random.nextFloat() < effectiveProbability) {
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
     * Calculate catalytic boost from nearby catalyst particles.
     * NOT and XNOR particles increase bond formation rates within a radius.
     * Returns a multiplier (1.0 = no boost, up to 3.0 = 3x boost).
     */
    private float calculateCatalyticBoost(Particle a, Particle b) {
        float catalystRadius = 5.0f;  // Catalyst influence radius
        float boostPerCatalyst = 0.5f;  // Each catalyst adds 50% to formation rate
        float maxBoost = 3.0f;  // Maximum 3x boost
        
        // Calculate midpoint between the two reacting particles
        float midX = (a.getPosition().x + b.getPosition().x) * 0.5f;
        float midY = (a.getPosition().y + b.getPosition().y) * 0.5f;
        
        // Count nearby catalyst particles
        int catalystCount = 0;
        Array<Particle> allParticles = physicsWorld.getActiveParticles();
        
        // Use indexed loop to avoid nested iterator issues
        for (int i = 0; i < allParticles.size; i++) {
            Particle particle = allParticles.get(i);
            if (!particle.isActive()) continue;
            if (particle == a || particle == b) continue;
            if (!particle.getType().isCatalyst()) continue;
            
            // Check distance to midpoint
            float dx = particle.getPosition().x - midX;
            float dy = particle.getPosition().y - midY;
            float distSq = dx * dx + dy * dy;
            
            if (distSq < catalystRadius * catalystRadius) {
                catalystCount++;
            }
        }
        
        // Calculate multiplier
        float multiplier = 1.0f + (catalystCount * boostPerCatalyst);
        return Math.min(multiplier, maxBoost);
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
    
    public float getActivationEnergy() {
        return activationEnergy;
    }
    
    public void setActivationEnergy(float energy) {
        this.activationEnergy = MathUtils.clamp(energy, 0f, 100f);
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
