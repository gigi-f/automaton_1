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
    private final EnergyField energyField;
    private final Array<MovingEnergyField> movingFields;
    private final CellInteriorDetector cellInteriorDetector;
    private final Array<Particle> corpsesToRemove;
    private float reactionViolenceMultiplier;
    private float energyDecayRate;
    private float bondEnergyCost;
    private float foodSpawnRate;
    private float foodSpawnAccumulator;
    private float replicationEnergyThreshold;
    private float replicationCheckInterval;
    private float replicationCheckAccumulator;
    private float mutationRate;
    private float environmentalEnergyRate;
    private float chemotaxisStrength;
    private int mitosisThreshold;  // Maximum molecule size before division
    private float mitosisCheckInterval;
    private float mitosisCheckAccumulator;

    public PhysicsWorld(Vector2 gravity, int initialParticleCapacity) {
        this.reactionViolenceMultiplier = 0.4f;  // Default: 40% violence (reduced from 1.0)
        this.energyDecayRate = 0.1f;  // Default: 0.1 energy/second decay
        this.bondEnergyCost = 0.5f;  // Default: 0.5 energy/second per bond
        this.foodSpawnRate = 2f;  // Default: 2 food particles per second
        this.foodSpawnAccumulator = 0f;
        this.replicationEnergyThreshold = 80f;  // Default: need 80+ energy to replicate
        this.replicationCheckInterval = 2f;  // Default: check every 2 seconds
        this.replicationCheckAccumulator = 0f;
        this.mutationRate = 0.1f;  // Default: 10% chance of mutation during replication
        this.environmentalEnergyRate = 5f;  // Default: 5 energy/sec from field
        this.chemotaxisStrength = 50f;  // Default: 50 force units for chemotaxis
        this.mitosisThreshold = 25;  // Default: molecules with 25+ particles undergo division
        this.mitosisCheckInterval = 3f;  // Default: check every 3 seconds
        this.mitosisCheckAccumulator = 0f;
        this.world = new World(gravity, true);
        this.particlePool = new ParticlePool(initialParticleCapacity, Integer.MAX_VALUE);
        this.bondPool = new BondPool(initialParticleCapacity * 2, Integer.MAX_VALUE);
        
        // Initialize energy field (100x56 world, 5 unit cells, 0.3 strength)
        this.energyField = new EnergyField(100f, 56f, 5f, 0.3f);
        
        // Initialize moving energy fields (one of each type)
        this.movingFields = new Array<>();
        for (MovingEnergyField.FieldType type : MovingEnergyField.FieldType.values()) {
            this.movingFields.add(new MovingEnergyField(type, 100f, 56f));
        }
        
        this.activeParticles = new Array<>(false, initialParticleCapacity);
        this.activeBonds = new Array<>(false, initialParticleCapacity * 2);
        // Cell size = 2.5 units (typical particle spacing in demo)
        this.spatialGrid = new SpatialHashGrid(2.5f);
        this.unionFind = new UnionFind();
        this.bondProperties = new BondProperties();
        this.cellInteriorDetector = new CellInteriorDetector();
        this.corpsesToRemove = new Array<>();
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
    
    public void setReplicationEnergyThreshold(float threshold) {
        this.replicationEnergyThreshold = Math.max(0f, Math.min(100f, threshold));
    }
    
    public float getReplicationEnergyThreshold() {
        return replicationEnergyThreshold;
    }
    
    public void setMutationRate(float rate) {
        this.mutationRate = Math.max(0f, Math.min(1f, rate));
    }
    
    public float getMutationRate() {
        return mutationRate;
    }
    
    public EnergyField getEnergyField() {
        return energyField;
    }
    
    public void setEnvironmentalEnergyRate(float rate) {
        this.environmentalEnergyRate = Math.max(0f, rate);
    }
    
    public float getEnvironmentalEnergyRate() {
        return environmentalEnergyRate;
    }
    
    public void setChemotaxisStrength(float strength) {
        this.chemotaxisStrength = Math.max(0f, strength);
    }
    
    public float getChemotaxisStrength() {
        return chemotaxisStrength;
    }
    
    public void setMitosisThreshold(int threshold) {
        this.mitosisThreshold = Math.max(3, threshold); // Minimum 3 for valid molecule
    }
    
    public int getMitosisThreshold() {
        return mitosisThreshold;
    }
    
    public Array<MovingEnergyField> getMovingFields() {
        return movingFields;
    }
    
    public void setWorldSize(float worldWidth, float worldHeight) {
        for (MovingEnergyField field : movingFields) {
            field.setWorldSize(worldWidth, worldHeight);
        }
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
        // Update moving energy fields
        for (MovingEnergyField field : movingFields) {
            field.update(delta);
        }
        
        // Spawn food particles at configured rate
        spawnFoodParticles(delta);
        
        // Process energy absorption from food particles
        processEnergyAbsorption(delta);
        
        // Apply energy decay and bond costs
        applyEnergyMechanics(delta);
        
        // Apply chemotaxis forces (gradient following)
        applyChemotaxis();
        
        // Apply crowding pressure (repulsion in dense regions)
        applyCrowdingPressure();
        
        // Age particles and bonds
        updateAging(delta);
        
        // Check for replication opportunities
        processReplication(delta);
        
        // Check for oversized molecules and trigger mitosis
        processMitosis(delta);
        
        // Remove particles that ran out of energy
        removeDeadParticles();
        
        // Process corpse consumption by molecular structures
        processCellInteriorConsumption();
        
        // Check for crossing bonds within same molecule and break them
        detectAndBreakCrossingBonds();
        
        // Apply rotational shear forces to break peripheral bonds
        applyRotationalShear();
        
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
        
        // Apply energy decay, bond costs, and environmental absorption
        for (int i = 0; i < activeParticles.size; i++) {
            Particle particle = activeParticles.get(i);
            if (!particle.isActive()) continue;
            
            // Base energy decay
            float decay = energyDecayRate * delta;
            
            // Metabolic scaling: larger molecules have higher energy costs
            // Get molecule size for this particle
            int moleculeSize = getComponentSize(particle);
            
            // Scaling factor: starts at 1.0 for small molecules, increases for larger ones
            // Formula: 1.0 + (size - 10) * 0.05, capped at 3x cost
            // Examples: size 10 = 1.0x, size 20 = 1.5x, size 30 = 2.0x, size 40+ = 3.0x
            float scalingFactor = Math.min(3.0f, 1.0f + Math.max(0f, (moleculeSize - 10) * 0.05f));
            
            // Additional cost per bond with metabolic scaling
            float bondCost = bondEnergyCost * bondCount[i] * scalingFactor * delta;
            
            // Environmental energy absorption from moving fields
            float environmentalGain = 0f;
            Vector2 particlePos = particle.getPosition();
            ParticleType particleType = particle.getType();
            
            for (MovingEnergyField field : movingFields) {
                if (!field.isActive()) continue;
                
                // Get field energy at particle position
                float fieldEnergy = field.getEnergyAt(particlePos.x, particlePos.y);
                
                // Get type-specific affinity (only positive affinity provides energy)
                float affinity = field.getType().getAffinityFor(particleType);
                if (affinity > 0f) {
                    // Energy field disruption: large molecules absorb less efficiently
                    // Penalty formula: 1.0 / (1.0 + size * 0.02)
                    // Examples: size 10 = 0.83x, size 20 = 0.71x, size 30 = 0.63x
                    float absorptionPenalty = 1.0f / (1.0f + moleculeSize * 0.02f);
                    environmentalGain += fieldEnergy * affinity * environmentalEnergyRate * absorptionPenalty * delta;
                }
            }
            
            // Net energy change
            particle.consumeEnergy(decay + bondCost);
            particle.addEnergy(environmentalGain);
        }
    }
    
    /**
     * Apply chemotaxis forces - particles follow energy gradients.
     * TRUE particles move toward high energy (positive chemotaxis).
     * FALSE particles move toward low energy (negative chemotaxis).
     */
    private void applyChemotaxis() {
        if (chemotaxisStrength <= 0f) return;
        
        float[] gradient = new float[2];
        
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            
            Vector2 particlePos = particle.getPosition();
            ParticleType particleType = particle.getType();
            
            // Accumulate forces from all active moving fields
            float totalForceX = 0f;
            float totalForceY = 0f;
            
            for (MovingEnergyField field : movingFields) {
                if (!field.isActive()) continue;
                
                // Get gradient direction toward this field
                field.getGradientDirection(particlePos.x, particlePos.y, gradient);
                
                // Get type-specific affinity (positive=attracted, negative=repelled)
                float affinity = field.getType().getAffinityFor(particleType);
                
                // Get field energy at particle position
                float fieldEnergy = field.getEnergyAt(particlePos.x, particlePos.y);
                
                // Apply force based on affinity and field strength
                float baseForce = chemotaxisStrength * affinity * fieldEnergy;
                
                // Energy field disruption: large molecules experience stronger pulling forces
                // This creates osmotic-like pressure that can tear molecules apart
                int moleculeSize = getComponentSize(particle);
                if (moleculeSize > 15) {
                    // Disruption multiplier increases with size
                    float disruptionMultiplier = 1.0f + (moleculeSize - 15) * 0.05f;
                    baseForce *= disruptionMultiplier;
                }
                
                totalForceX += gradient[0] * baseForce;
                totalForceY += gradient[1] * baseForce;
            }
            
            // Apply accumulated force
            particle.getBody().applyForceToCenter(totalForceX, totalForceY, true);
        }
    }
    
    /**
     * Apply crowding pressure - particles in dense regions experience repulsive forces.
     * This simulates cytoskeletal stress and membrane tension in cells.
     * Helps break up overly dense molecular structures.
     */
    private void applyCrowdingPressure() {
        // Crowding radius - distance at which particles start to feel repulsion
        final float CROWDING_RADIUS = 2.5f;
        final float CROWDING_STRENGTH = 30f; // Force magnitude
        
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            
            Vector2 particlePos = particle.getPosition();
            
            // Query nearby particles
            Array<Particle> neighbors = queryNeighbors(particle, CROWDING_RADIUS);
            
            if (neighbors.size < 3) continue; // Not crowded enough
            
            // Calculate crowding density (more neighbors = stronger pressure)
            float crowdingFactor = Math.min(1.0f, neighbors.size / 8.0f); // Cap at 8 neighbors
            
            // Calculate net repulsion direction
            float repulsionX = 0f;
            float repulsionY = 0f;
            
            for (Particle neighbor : neighbors) {
                if (!neighbor.isAlive()) continue; // Don't interact with corpses
                
                Vector2 neighborPos = neighbor.getPosition();
                float dx = particlePos.x - neighborPos.x;
                float dy = particlePos.y - neighborPos.y;
                float distSq = dx * dx + dy * dy;
                
                if (distSq < 0.01f) continue; // Too close, avoid division by zero
                
                float dist = (float) Math.sqrt(distSq);
                
                // Inverse square law for repulsion (stronger when very close)
                float repulsionMag = 1.0f / (distSq + 0.1f);
                
                // Normalized direction away from neighbor
                repulsionX += (dx / dist) * repulsionMag;
                repulsionY += (dy / dist) * repulsionMag;
            }
            
            // Apply crowding pressure force
            float forceX = repulsionX * CROWDING_STRENGTH * crowdingFactor;
            float forceY = repulsionY * CROWDING_STRENGTH * crowdingFactor;
            
            particle.getBody().applyForceToCenter(forceX, forceY, true);
        }
    }
    
    /**
     * Update the age of all active particles and bonds.
     * Aging causes bonds to become more fragile over time.
     */
    private void updateAging(float delta) {
        // Age particles
        for (Particle particle : activeParticles) {
            if (particle.isActive()) {
                particle.updateAge(delta);
            }
        }
        
        // Age bonds
        for (Bond bond : activeBonds) {
            if (bond.isActive()) {
                bond.updateAge(delta);
            }
        }
    }
    
    /**
     * Process molecular replication.
     * Detect simple bonded patterns (pairs, triangles) and replicate them
     * if conditions are met (high energy, space available).
     */
    private void processReplication(float delta) {
        replicationCheckAccumulator += delta;
        
        if (replicationCheckAccumulator < replicationCheckInterval) {
            return;  // Not time to check yet
        }
        
        replicationCheckAccumulator = 0f;
        
        // Find replicable molecules (simple pairs with high energy)
        for (int i = 0; i < activeBonds.size; i++) {
            Bond bond = activeBonds.get(i);
            if (!bond.isActive()) continue;
            
            Particle a = bond.getParticleA();
            Particle b = bond.getParticleB();
            
            // Check if both particles have sufficient energy
            if (a.getEnergy() < replicationEnergyThreshold || 
                b.getEnergy() < replicationEnergyThreshold) {
                continue;
            }
            
            // Check if this is a simple pair (each particle has only 1 bond)
            int bondsA = countBonds(a);
            int bondsB = countBonds(b);
            
            if (bondsA != 1 || bondsB != 1) {
                continue;  // Not a simple pair
            }
            
            // Find space for replication (nearby but not too close)
            Vector2 midpoint = new Vector2(a.getPosition()).add(b.getPosition()).scl(0.5f);
            Vector2 replicaPos = findReplicationSpace(midpoint, 4f, 8f);
            
            if (replicaPos == null) {
                continue;  // No space available
            }
            
            // Replicate the pair!
            replicatePair(a, b, replicaPos, bond);
            
            // Consume significant energy from parents
            a.consumeEnergy(40f);
            b.consumeEnergy(40f);
            
            break;  // Only one replication per check interval
        }
    }
    
    /**
     * Count how many bonds a particle has.
     */
    private int countBonds(Particle particle) {
        int count = 0;
        for (int i = 0; i < activeBonds.size; i++) {
            Bond bond = activeBonds.get(i);
            if (!bond.isActive()) continue;
            if (bond.getParticleA() == particle || bond.getParticleB() == particle) {
                count++;
            }
        }
        return count;
    }
    
    /**
     * Find empty space for replication near a position.
     * Returns null if no suitable space found.
     */
    private Vector2 findReplicationSpace(Vector2 center, float minDist, float maxDist) {
        // Try 8 directions around the center
        float[] angles = {0, 45, 90, 135, 180, 225, 270, 315};
        
        for (float angle : angles) {
            float rad = (float) Math.toRadians(angle);
            float distance = minDist + (maxDist - minDist) * 0.5f;
            
            float x = center.x + (float) Math.cos(rad) * distance;
            float y = center.y + (float) Math.sin(rad) * distance;
            
            // Check if position is clear (no particles within 3 units)
            boolean isClear = true;
            for (Particle p : activeParticles) {
                if (!p.isActive()) continue;
                float dx = p.getPosition().x - x;
                float dy = p.getPosition().y - y;
                if (dx * dx + dy * dy < 9f) {  // 3 units radius
                    isClear = false;
                    break;
                }
            }
            
            if (isClear) {
                return new Vector2(x, y);
            }
        }
        
        return null;  // No space found
    }
    
    /**
     * Create a replica of a bonded pair at the specified position.
     */
    private void replicatePair(Particle parentA, Particle parentB, Vector2 replicaCenter, Bond parentBond) {
        // Calculate offset for the two child particles
        Vector2 direction = new Vector2(parentB.getPosition()).sub(parentA.getPosition());
        float bondLength = direction.len();
        direction.nor();
        
        Vector2 posA = new Vector2(replicaCenter).sub(direction.scl(bondLength * 0.5f));
        Vector2 posB = new Vector2(replicaCenter).add(direction.scl(bondLength * 0.5f));
        
        // Determine child types (with possible mutation)
        ParticleType typeA = shouldMutate() ? getRandomParticleType() : parentA.getType();
        ParticleType typeB = shouldMutate() ? getRandomParticleType() : parentB.getType();
        
        // Spawn replica particles (possibly mutated)
        Particle childA = spawnParticle(typeA, posA, 0.5f, 1f);
        Particle childB = spawnParticle(typeB, posB, 0.5f, 1f);
        
        // Give children moderate starting energy
        childA.setEnergy(60f);
        childB.setEnergy(60f);
        
        // Create bond between children (copy parent bond properties)
        createBond(childA, childB, BondType.ELASTIC, 
                   50f, 2f, parentBond.getBreakForceThreshold());
        
        // Give children small random velocity (ejection from parent)
        Vector2 perpendicular = new Vector2(-direction.y, direction.x);
        float vx = perpendicular.x * 5f;
        float vy = perpendicular.y * 5f;
        childA.getBody().setLinearVelocity(vx, vy);
        childB.getBody().setLinearVelocity(vx, vy);
    }
    
    /**
     * Check if mutation should occur based on mutation rate.
     */
    private boolean shouldMutate() {
        return Math.random() < mutationRate;
    }
    
    /**
     * Get a random particle type for mutation.
     */
    private ParticleType getRandomParticleType() {
        ParticleType[] types = ParticleType.values();
        return types[(int) (Math.random() * types.length)];
    }
    
    /**
     * Process mitosis (cell division) for oversized molecules.
     * When a molecule exceeds the size threshold, find the best place to divide it.
     */
    private void processMitosis(float delta) {
        if (mitosisThreshold <= 0) return;
        
        mitosisCheckAccumulator += delta;
        if (mitosisCheckAccumulator < mitosisCheckInterval) return;
        mitosisCheckAccumulator = 0f;
        
        // Group particles by component ID to identify molecules
        com.badlogic.gdx.utils.IntMap<Array<Particle>> componentGroups = new com.badlogic.gdx.utils.IntMap<>();
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            
            int componentId = particle.getComponentId();
            Array<Particle> group = componentGroups.get(componentId);
            if (group == null) {
                group = new Array<>();
                componentGroups.put(componentId, group);
            }
            group.add(particle);
        }
        
        // Check each molecule for size threshold or metabolic stress
        for (com.badlogic.gdx.utils.IntMap.Entry<Array<Particle>> entry : componentGroups) {
            Array<Particle> molecule = entry.value;
            
            // Trigger 1: Size threshold exceeded
            boolean oversized = molecule.size >= mitosisThreshold;
            
            // Trigger 2: Metabolic stress - average energy is critically low and molecule is large
            float avgEnergy = 0f;
            for (Particle p : molecule) {
                avgEnergy += p.getEnergy();
            }
            avgEnergy /= molecule.size;
            boolean metabolicStress = molecule.size > 15 && avgEnergy < 30f;
            
            if (oversized || metabolicStress) {
                divideMolecule(molecule);
            }
        }
    }
    
    /**
     * Divide a molecule by breaking bonds along its narrowest cross-section.
     * Strategy: Find the bond whose removal would best split the molecule into two equal halves.
     */
    private void divideMolecule(Array<Particle> moleculeParticles) {
        if (moleculeParticles.size < 3) return; // Too small to divide
        
        // Calculate center of mass
        float cx = 0, cy = 0;
        for (Particle p : moleculeParticles) {
            Vector2 pos = p.getPosition();
            cx += pos.x;
            cy += pos.y;
        }
        cx /= moleculeParticles.size;
        cy /= moleculeParticles.size;
        
        // Find all bonds within this molecule
        Array<Bond> moleculeBonds = new Array<>();
        for (Bond bond : activeBonds) {
            if (!bond.isActive()) continue;
            Particle a = bond.getParticleA();
            Particle b = bond.getParticleB();
            if (moleculeParticles.contains(a, true) && moleculeParticles.contains(b, true)) {
                moleculeBonds.add(bond);
            }
        }
        
        if (moleculeBonds.size == 0) return;
        
        // Find bond closest to center of mass (breaking here splits molecule most evenly)
        Bond divisionBond = null;
        float minDistToCenter = Float.MAX_VALUE;
        
        for (Bond bond : moleculeBonds) {
            Vector2 posA = bond.getParticleA().getPosition();
            Vector2 posB = bond.getParticleB().getPosition();
            float midX = (posA.x + posB.x) * 0.5f;
            float midY = (posA.y + posB.y) * 0.5f;
            float distToCenter = (float) Math.sqrt((midX - cx) * (midX - cx) + (midY - cy) * (midY - cy));
            
            if (distToCenter < minDistToCenter) {
                minDistToCenter = distToCenter;
                divisionBond = bond;
            }
        }
        
        // Break the division bond (with some repulsive force)
        if (divisionBond != null) {
            destroyBond(divisionBond, true); // Violent break to push halves apart
        }
    }
    
    /**
     * Remove particles that have zero energy.
     */
    private void removeDeadParticles() {
        Array<Particle> particlesToKill = new Array<>();
        
        for (Particle particle : activeParticles) {
            if (particle.isActive() && particle.getEnergy() <= 0f && particle.isAlive()) {
                particlesToKill.add(particle);
            }
        }
        
        for (Particle particle : particlesToKill) {
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
            
            // Convert to corpse instead of destroying
            particle.kill(); // Sets isAlive=false, isEnergyRich=true
            
            // Reduce velocity to simulate "death"
            particle.getBody().setLinearVelocity(
                particle.getBody().getLinearVelocity().scl(0.3f)
            );
        }
    }
    
    /**
     * Process corpses that enter the interior space of molecular structures.
     * When a corpse (dead particle) is detected inside a "cell" (molecule with 3+ bonded particles),
     * its energy is distributed to all living particles in that molecule and the corpse is removed.
     */
    private void processCellInteriorConsumption() {
        // Detect which corpses are inside molecular structures
        cellInteriorDetector.processCorpseConsumption(activeParticles, corpsesToRemove);
        
        // Remove consumed corpses
        for (Particle corpse : corpsesToRemove) {
            destroyParticle(corpse);
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
    
    /**
     * Apply rotational shear forces to break peripheral bonds in fast-spinning molecules.
     * When angular velocity exceeds threshold, centrifugal forces tear molecules apart.
     */
    private void applyRotationalShear() {
        final float ROTATION_THRESHOLD = 2.0f; // rad/s - angular velocity threshold
        final float CENTRIFUGAL_FORCE = 100f; // Force multiplier
        
        // Group particles by component to analyze molecular rotation
        com.badlogic.gdx.utils.IntMap<Array<Particle>> componentGroups = new com.badlogic.gdx.utils.IntMap<>();
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            
            int componentId = particle.getComponentId();
            Array<Particle> group = componentGroups.get(componentId);
            if (group == null) {
                group = new Array<>();
                componentGroups.put(componentId, group);
            }
            group.add(particle);
        }
        
        // Check each molecule for excessive rotation
        for (com.badlogic.gdx.utils.IntMap.Entry<Array<Particle>> entry : componentGroups) {
            Array<Particle> molecule = entry.value;
            
            if (molecule.size < 3) continue; // Too small to experience significant shear
            
            // Calculate average angular velocity
            float avgAngularVel = 0f;
            for (Particle p : molecule) {
                avgAngularVel += Math.abs(p.getBody().getAngularVelocity());
            }
            avgAngularVel /= molecule.size;
            
            if (avgAngularVel < ROTATION_THRESHOLD) continue;
            
            // Calculate center of mass
            float cx = 0, cy = 0;
            for (Particle p : molecule) {
                Vector2 pos = p.getPosition();
                cx += pos.x;
                cy += pos.y;
            }
            cx /= molecule.size;
            cy /= molecule.size;
            
            // Apply centrifugal force to peripheral particles
            // Find and break bonds on particles far from center
            Array<Bond> peripheralBonds = new Array<>();
            
            for (Bond bond : activeBonds) {
                if (!bond.isActive()) continue;
                
                Particle a = bond.getParticleA();
                Particle b = bond.getParticleB();
                
                // Check if both particles are in this molecule
                if (!molecule.contains(a, true) || !molecule.contains(b, true)) continue;
                
                // Calculate distance from center for both particles
                Vector2 posA = a.getPosition();
                Vector2 posB = b.getPosition();
                float distA = (float) Math.sqrt((posA.x - cx) * (posA.x - cx) + (posA.y - cy) * (posA.y - cy));
                float distB = (float) Math.sqrt((posB.x - cx) * (posB.x - cx) + (posB.y - cy) * (posB.y - cy));
                
                // If at least one particle is peripheral (far from center)
                float maxDist = Math.max(distA, distB);
                if (maxDist > 3.0f) { // Peripheral threshold
                    // Apply outward centrifugal force
                    float forceX = (posA.x - cx) * CENTRIFUGAL_FORCE * avgAngularVel;
                    float forceY = (posA.y - cy) * CENTRIFUGAL_FORCE * avgAngularVel;
                    a.getBody().applyForceToCenter(forceX, forceY, true);
                    
                    forceX = (posB.x - cx) * CENTRIFUGAL_FORCE * avgAngularVel;
                    forceY = (posB.y - cy) * CENTRIFUGAL_FORCE * avgAngularVel;
                    b.getBody().applyForceToCenter(forceX, forceY, true);
                    
                    // Mark bond for potential breaking if rotation is extreme
                    if (avgAngularVel > ROTATION_THRESHOLD * 2f) {
                        peripheralBonds.add(bond);
                    }
                }
            }
            
            // Break peripheral bonds under extreme rotation
            for (Bond bond : peripheralBonds) {
                if (bond.isActive() && Math.random() < 0.3f) { // 30% chance per check
                    destroyBond(bond, true); // Violent break
                }
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
