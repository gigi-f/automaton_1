package com.automaton.core.physics;

import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.CircleShape;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.World;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Disposable;

/**
 * Wrapper around the Box2D {@link World} managing particle lifecycle, pooling and
 * stepping. Bonds/springs are handled elsewhere but are expected to act on the
 * bodies exposed by this class.
 */
public final class PhysicsWorld implements Disposable {
    private static final float DEFAULT_LINEAR_DAMPING = 0.05f;  // Minimal damping for endless motion
    private static final float DEFAULT_ANGULAR_DAMPING = 0.05f;
    private static final float DEFAULT_PARTICLE_DENSITY = 1f;
    private static final float DEFAULT_PARTICLE_FRICTION = 0.0f;  // Zero friction
    private static final float DEFAULT_PARTICLE_RESTITUTION = 1.0f;  // Perfectly elastic collisions

    private final World world;
    private final ParticlePool particlePool;
    private final BondPool bondPool;
    private final Array<Particle> activeParticles;
    private final Array<Bond> activeBonds;
    private final SpatialHashGrid spatialGrid;
    private final UnionFind unionFind;
    private final BondProperties bondProperties;
    private float reactionViolenceMultiplier;
    private float energyDecayRate;
    private float bondEnergyCost;
    private float foodSpawnRate;
    private float foodSpawnAccumulator;

    public PhysicsWorld(Vector2 gravity, int initialParticleCapacity) {
        this.reactionViolenceMultiplier = 0.4f;  // Default: 40% violence (reduced from 1.0)
        this.energyDecayRate = 0.1f;  // Default: 0.1 energy/second decay
        this.bondEnergyCost = 0.5f;  // Default: 0.5 energy/second per bond
        this.foodSpawnRate = 2f;  // Default: 2 food particles per second
        this.foodSpawnAccumulator = 0f;
        this.world = new World(gravity, true);
        this.particlePool = new ParticlePool(initialParticleCapacity, Integer.MAX_VALUE);
        this.bondPool = new BondPool(initialParticleCapacity * 2, Integer.MAX_VALUE);
        this.activeParticles = new Array<>(false, initialParticleCapacity);
        this.activeBonds = new Array<>(false, initialParticleCapacity * 2);
        // Cell size = 2.5 units (typical particle spacing in demo)
        this.spatialGrid = new SpatialHashGrid(2.5f);
        this.unionFind = new UnionFind();
        this.bondProperties = new BondProperties();
    }
    
    public BondProperties getBondProperties() {
        return bondProperties;
    }
    
    public void setReactionViolenceMultiplier(float multiplier) {
        this.reactionViolenceMultiplier = Math.max(0f, multiplier);
    }
    
    public float getReactionViolenceMultiplier() {
        return reactionViolenceMultiplier;
    }
    
    public void setEnergyDecayRate(float rate) {
        this.energyDecayRate = Math.max(0f, rate);
    }
    
    public float getEnergyDecayRate() {
        return energyDecayRate;
    }
    
    public void setBondEnergyCost(float cost) {
        this.bondEnergyCost = Math.max(0f, cost);
    }
    
    public float getBondEnergyCost() {
        return bondEnergyCost;
    }
    
    public void setFoodSpawnRate(float rate) {
        this.foodSpawnRate = Math.max(0f, rate);
    }
    
    public float getFoodSpawnRate() {
        return foodSpawnRate;
    }

    public World getWorld() {
        return world;
    }

    public Array<Particle> getActiveParticles() {
        return activeParticles;
    }

    public Array<Bond> getActiveBonds() {
        return activeBonds;
    }

    public SpatialHashGrid getSpatialGrid() {
        return spatialGrid;
    }

    public UnionFind getUnionFind() {
        return unionFind;
    }

    /**
     * Query particles within a radius of a position using spatial hashing.
     * Efficient O(k) operation for finding nearby particles.
     */
    public Array<Particle> queryParticlesInRadius(float x, float y, float radius) {
        return spatialGrid.queryRadius(x, y, radius);
    }

    /**
     * Query neighbors of a particle within a given radius (excluding the particle itself).
     */
    public Array<Particle> queryNeighbors(Particle particle, float radius) {
        return spatialGrid.queryNeighbors(particle, radius);
    }

    /**
     * Check if two particles are in the same connected component.
     */
    public boolean areConnected(Particle a, Particle b) {
        return unionFind.connected(a.hashCode(), b.hashCode());
    }

    /**
     * Get the size of the connected component containing this particle.
     */
    public int getComponentSize(Particle particle) {
        return unionFind.getComponentSize(particle.hashCode());
    }

    public Particle spawnParticle(ParticleType type, Vector2 position, float visualRadius, float mass) {
        float collisionRadius = visualRadius * 0.6f;
        CircleShape shape = new CircleShape();
        shape.setRadius(collisionRadius);

        BodyDef bodyDef = new BodyDef();
        bodyDef.type = BodyDef.BodyType.DynamicBody;
        bodyDef.position.set(position);
        bodyDef.linearDamping = DEFAULT_LINEAR_DAMPING;
        bodyDef.angularDamping = DEFAULT_ANGULAR_DAMPING;

        Body body = world.createBody(bodyDef);

        FixtureDef fixtureDef = new FixtureDef();
        fixtureDef.shape = shape;
        float area = MathUtils.PI * collisionRadius * collisionRadius;
        float density = area > MathUtils.FLOAT_ROUNDING_ERROR ? mass / area : DEFAULT_PARTICLE_DENSITY;
        fixtureDef.density = density;
        fixtureDef.friction = DEFAULT_PARTICLE_FRICTION;
        fixtureDef.restitution = DEFAULT_PARTICLE_RESTITUTION;

        body.createFixture(fixtureDef);
        shape.dispose();

        Particle particle = particlePool.obtain();
        // Initialize with random energy between 50-100
        float initialEnergy = 50f + MathUtils.random(50f);
        particle.init(body, type, collisionRadius, visualRadius, initialEnergy);
        activeParticles.add(particle);
        
        // Add to union-find as its own component
        unionFind.addParticle(particle.hashCode());
        particle.setComponentId(particle.hashCode());
        
        return particle;
    }

    public void destroyParticle(Particle particle) {
        if (particle == null || !particle.isActive()) {
            return;
        }
        
        // Remove from union-find
        unionFind.removeParticle(particle.hashCode());
        
        particle.deactivate(world);
        activeParticles.removeValue(particle, true);
        particlePool.free(particle);
    }

    public Bond createBond(Particle a, Particle b, BondType type, float stiffness, 
                           float damping, float breakForceThreshold) {
        if (a == null || b == null || !a.isActive() || !b.isActive()) {
            return null;
        }
        
        // Get multipliers based on particle types
        float[] multipliers = bondProperties.getMultipliers(a.getType(), b.getType());
        float lengthMult = multipliers[0];
        float stiffnessMult = multipliers[1];
        float breakMult = multipliers[2];
        
        Vector2 posA = a.getPosition();
        Vector2 posB = b.getPosition();
        float baseRestLength = posA.dst(posB);
        
        // Apply multipliers
        float restLength = baseRestLength * lengthMult;
        float finalStiffness = stiffness * stiffnessMult;
        float finalBreakForce = breakForceThreshold * breakMult;
        
        Bond bond = bondPool.obtain();
        bond.init(a, b, type, restLength, finalStiffness, damping, finalBreakForce);
        activeBonds.add(bond);
        
        // Union components when bond forms
        if (unionFind.union(a.hashCode(), b.hashCode())) {
            // Update component IDs
            int newComponentId = unionFind.find(a.hashCode());
            a.setComponentId(newComponentId);
            b.setComponentId(newComponentId);
        }
        
        return bond;
    }

    public void destroyBond(Bond bond) {
        destroyBond(bond, true);
    }
    
    /**
     * Destroy a bond, optionally applying a violent reaction force.
     * The strength of the reaction is proportional to the bond's break force threshold.
     */
    public void destroyBond(Bond bond, boolean applyViolentReaction) {
        if (bond == null || !bond.isActive()) {
            return;
        }
        
        // Apply violent reaction before deactivating
        if (applyViolentReaction) {
            applyBondBreakReaction(bond);
        }
        
        bond.deactivate();
        activeBonds.removeValue(bond, true);
        bondPool.free(bond);
        
        // Note: Union-Find doesn't support efficient split operations.
        // For accurate component tracking after many bond breaks, call rebuildUnionFind()
    }
    
    /**
     * Apply violent reaction force when a bond breaks.
     * Stronger bonds create more violent reactions (stronger repulsion).
     * Force is modulated by the reactionViolenceMultiplier slider.
     */
    private void applyBondBreakReaction(Bond bond) {
        Particle a = bond.getParticleA();
        Particle b = bond.getParticleB();
        
        // Reaction force scales with bond strength (break force threshold)
        // Base: 200, Range: 60-600, Scale: 2-12x multiplier
        float breakForce = bond.getBreakForceThreshold();
        float reactionMultiplier = breakForce / 50f;  // 60->1.2x, 200->4x, 600->12x
        float baseRepulsion = 300f;
        float repulsionStrength = baseRepulsion * reactionMultiplier * reactionViolenceMultiplier;
        
        // Calculate direction from A to B
        Vector2 posA = a.getPosition();
        Vector2 posB = b.getPosition();
        Vector2 direction = new Vector2(posB).sub(posA);
        float distance = direction.len();
        
        if (distance > 0.001f) {
            direction.nor();
            
            // Apply explosive force pushing particles apart
            Vector2 forceA = new Vector2(direction).scl(-repulsionStrength);
            Vector2 forceB = new Vector2(direction).scl(repulsionStrength);
            
            a.getBody().applyForceToCenter(forceA, true);
            b.getBody().applyForceToCenter(forceB, true);
        }
    }

    public void step(float delta, int velocityIterations, int positionIterations) {
        // Spawn food particles at configured rate
        spawnFoodParticles(delta);
        
        // Process energy absorption from food particles
        processEnergyAbsorption(delta);
        
        // Apply energy decay and bond costs
        applyEnergyMechanics(delta);
        
        // Remove particles that ran out of energy
        removeDeadParticles();
        
        // Check for crossing bonds within same molecule and break them
        detectAndBreakCrossingBonds();
        
        // Apply bond constraints before Box2D step
        for (int i = activeBonds.size - 1; i >= 0; i--) {
            Bond bond = activeBonds.get(i);
            if (bond.applyConstraint()) {
                destroyBond(bond);
            }
        }
        
        world.step(delta, velocityIterations, positionIterations);
        
        // Update spatial grid after physics step
        spatialGrid.update(activeParticles);
    }
    
    /**
     * Process metabolic byproducts from active bonds.
     * Certain bond combinations create energy-rich particles as byproducts.
     * This creates a metabolism-like system where bonds produce consumable energy.
     */
    private void spawnFoodParticles(float delta) {
        if (foodSpawnRate <= 0) return;
        
        foodSpawnAccumulator += delta;
        float spawnInterval = 1f / foodSpawnRate;  // Seconds between spawns
        
        while (foodSpawnAccumulator >= spawnInterval) {
            foodSpawnAccumulator -= spawnInterval;
            
            // Find productive bonds (bonds that generate energy byproducts)
            Array<Bond> productiveBonds = new Array<>();
            for (Bond bond : activeBonds) {
                if (!bond.isActive()) continue;
                if (isProductiveBond(bond)) {
                    productiveBonds.add(bond);
                }
            }
            
            if (productiveBonds.size == 0) continue;
            
            // Pick a random productive bond
            Bond selectedBond = productiveBonds.get(MathUtils.random(productiveBonds.size - 1));
            
            // Spawn energy-rich particle near the bond midpoint
            Particle a = selectedBond.getParticleA();
            Particle b = selectedBond.getParticleB();
            Vector2 midpoint = new Vector2(a.getPosition()).add(b.getPosition()).scl(0.5f);
            
            // Add some random offset
            float offsetX = MathUtils.random(-2f, 2f);
            float offsetY = MathUtils.random(-2f, 2f);
            midpoint.add(offsetX, offsetY);
            
            // Determine particle type based on bond types
            ParticleType byproductType = getByproductType(a.getType(), b.getType());
            
            // Create energy-rich particle
            Particle byproduct = spawnParticle(byproductType, midpoint, 0.5f, 1f);
            byproduct.setEnergy(60f);  // Rich in energy
            byproduct.setEnergyRich(true);  // Mark as consumable
            
            // Give it velocity away from bond (ejection)
            Vector2 direction = new Vector2(midpoint).sub(a.getPosition()).nor();
            float vx = direction.x * MathUtils.random(8f, 15f) + MathUtils.random(-3f, 3f);
            float vy = direction.y * MathUtils.random(8f, 15f) + MathUtils.random(-3f, 3f);
            byproduct.getBody().setLinearVelocity(vx, vy);
        }
    }
    
    /**
     * Determine if a bond produces energy byproducts.
     * Productive bonds: different types, especially logic states + gates.
     */
    private boolean isProductiveBond(Bond bond) {
        ParticleType typeA = bond.getParticleA().getType();
        ParticleType typeB = bond.getParticleB().getType();
        
        // Same type bonds don't produce byproducts
        if (typeA == typeB) return false;
        
        // Logic states bonded to gates are productive
        if (typeA.isLogicState() && typeB.isGate()) return true;
        if (typeB.isLogicState() && typeA.isGate()) return true;
        
        // Different logic states are productive
        if (typeA.isLogicState() && typeB.isLogicState()) return true;
        
        // Different gates together are moderately productive
        if (typeA.isGate() && typeB.isGate()) {
            return MathUtils.randomBoolean(0.3f); // 30% chance
        }
        
        return false;
    }
    
    /**
     * Determine the particle type of the metabolic byproduct based on parent types.
     * Creates interesting combinations - e.g., TRUE+AND -> FALSE, NOT+OR -> TRUE
     */
    private ParticleType getByproductType(ParticleType typeA, ParticleType typeB) {
        // TRUE + FALSE -> Random gate
        if ((typeA == ParticleType.TRUE && typeB == ParticleType.FALSE) ||
            (typeA == ParticleType.FALSE && typeB == ParticleType.TRUE)) {
            ParticleType[] gates = {ParticleType.AND, ParticleType.OR, ParticleType.XOR};
            return gates[MathUtils.random(gates.length - 1)];
        }
        
        // TRUE + Gate -> FALSE (consumption/inhibition)
        if (typeA == ParticleType.TRUE && typeB.isGate()) return ParticleType.FALSE;
        if (typeB == ParticleType.TRUE && typeA.isGate()) return ParticleType.FALSE;
        
        // FALSE + Gate -> TRUE (activation)
        if (typeA == ParticleType.FALSE && typeB.isGate()) return ParticleType.TRUE;
        if (typeB == ParticleType.FALSE && typeA.isGate()) return ParticleType.TRUE;
        
        // Gate + Gate -> Logic state (random)
        if (typeA.isGate() && typeB.isGate()) {
            return MathUtils.randomBoolean() ? ParticleType.TRUE : ParticleType.FALSE;
        }
        
        // Default: opposite of first type if logic state
        if (typeA.isLogicState()) {
            return typeA == ParticleType.TRUE ? ParticleType.FALSE : ParticleType.TRUE;
        }
        
        return ParticleType.TRUE;
    }
    
    /**
     * Process energy absorption when particles contact energy-rich particles.
     * Energy-rich particles transfer energy and are consumed when depleted.
     */
    private void processEnergyAbsorption(float delta) {
        float absorptionRadius = 1.5f;  // Distance for energy transfer
        float transferRate = 30f;  // Energy units per second
        
        Array<Particle> foodToRemove = new Array<>();
        
        for (Particle food : activeParticles) {
            if (!food.isActive() || !food.isEnergyRich()) continue;
            if (food.getEnergy() <= 0) {
                foodToRemove.add(food);
                continue;
            }
            
            // Find nearby particles that can absorb energy
            Array<Particle> nearbyParticles = spatialGrid.queryRadius(
                food.getPosition().x, food.getPosition().y, absorptionRadius);
            
            for (Particle consumer : nearbyParticles) {
                if (!consumer.isActive() || consumer == food) continue;
                if (consumer.isEnergyRich()) continue; // Energy-rich particles don't consume each other
                
                // Check if close enough for energy transfer
                float distSq = food.getPosition().dst2(consumer.getPosition());
                if (distSq < absorptionRadius * absorptionRadius) {
                    // Transfer energy from food to consumer
                    float transferAmount = transferRate * delta;
                    float actualTransfer = Math.min(transferAmount, food.getEnergy());
                    
                    food.consumeEnergy(actualTransfer);
                    consumer.addEnergy(actualTransfer);
                    
                    // Mark as depleted (will be cleaned up by removeDeadParticles)
                    if (food.getEnergy() <= 0) {
                        break;
                    }
                }
            }
        }
        
        // Note: Depleted food particles (energy = 0) are removed by removeDeadParticles()
    }
    
    /**
     * Apply energy decay and bond energy costs to all particles.
     */
    private void applyEnergyMechanics(float delta) {
        // Count bonds per particle
        int[] bondCount = new int[activeParticles.size];
        for (Bond bond : activeBonds) {
            if (!bond.isActive()) continue;
            
            for (int i = 0; i < activeParticles.size; i++) {
                Particle p = activeParticles.get(i);
                if (p == bond.getParticleA() || p == bond.getParticleB()) {
                    bondCount[i]++;
                }
            }
        }
        
        // Apply energy decay and bond costs
        for (int i = 0; i < activeParticles.size; i++) {
            Particle particle = activeParticles.get(i);
            if (!particle.isActive()) continue;
            
            // Base energy decay
            float decay = energyDecayRate * delta;
            
            // Additional cost per bond
            float bondCost = bondEnergyCost * bondCount[i] * delta;
            
            particle.consumeEnergy(decay + bondCost);
        }
    }
    
    /**
     * Remove particles that have zero energy.
     */
    private void removeDeadParticles() {
        Array<Particle> particlesToRemove = new Array<>();
        
        for (Particle particle : activeParticles) {
            if (particle.isActive() && particle.getEnergy() <= 0f) {
                particlesToRemove.add(particle);
            }
        }
        
        for (Particle particle : particlesToRemove) {
            // Remove all bonds connected to this particle first
            Array<Bond> bondsToRemove = new Array<>();
            for (Bond bond : activeBonds) {
                if (bond.isActive() && 
                    (bond.getParticleA() == particle || bond.getParticleB() == particle)) {
                    bondsToRemove.add(bond);
                }
            }
            
            for (Bond bond : bondsToRemove) {
                destroyBond(bond, false);  // No violent reaction on energy death
            }
            
            destroyParticle(particle);
        }
    }

    /**
     * Detect bonds that cross within the same molecule and break them with repulsive force.
     */
    private void detectAndBreakCrossingBonds() {
        Array<Bond> bondsToBreak = new Array<>();
        
        // Check all pairs of bonds
        for (int i = 0; i < activeBonds.size; i++) {
            Bond bondA = activeBonds.get(i);
            if (!bondA.isActive()) continue;
            
            // Only check bonds within the same molecule (same component)
            int componentA = bondA.getParticleA().getComponentId();
            
            for (int j = i + 1; j < activeBonds.size; j++) {
                Bond bondB = activeBonds.get(j);
                if (!bondB.isActive()) continue;
                
                // Check if in same molecule
                int componentB = bondB.getParticleA().getComponentId();
                if (componentA != componentB) continue;
                
                // Check if bonds intersect
                if (bondA.intersects(bondB)) {
                    bondsToBreak.add(bondA);
                    bondsToBreak.add(bondB);
                }
            }
        }
        
        // Break all crossing bonds (violent reactions applied automatically)
        for (Bond bond : bondsToBreak) {
            if (bond.isActive()) {
                destroyBond(bond, true);
            }
        }
    }

    public void clearAllBonds() {
        for (int i = activeBonds.size - 1; i >= 0; i--) {
            destroyBond(activeBonds.get(i), false);  // No violent reactions on cleanup
        }
    }

    public void clearAllParticles() {
        for (int i = activeParticles.size - 1; i >= 0; i--) {
            destroyParticle(activeParticles.get(i));
        }
    }

    /**
     * Rebuild union-find from scratch based on current bonds.
     * Call this periodically or after many bond breaks to ensure accurate component tracking.
     */
    public void rebuildUnionFind() {
        unionFind.rebuild(activeParticles, activeBonds);
        
        // Update component IDs in particles
        for (Particle particle : activeParticles) {
            if (particle.isActive()) {
                int componentId = unionFind.find(particle.hashCode());
                particle.setComponentId(componentId);
            }
        }
    }

    @Override
    public void dispose() {
        clearAllBonds();
        clearAllParticles();
        world.dispose();
    }
}
