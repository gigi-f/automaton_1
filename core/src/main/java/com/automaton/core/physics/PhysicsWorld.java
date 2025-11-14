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
import com.badlogic.gdx.utils.IntFloatMap;
import com.badlogic.gdx.utils.IntIntMap;
import com.badlogic.gdx.utils.LongMap;

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
    private static final int MAX_MITOSIS_DIVISIONS_PER_CHECK = 2;
    private static final float MIN_MITOSIS_SPLIT_BAND = 0.75f;
    private static final float MIN_MITOSIS_PUSH_FORCE = 400f;
    private static final float MAX_MITOSIS_PUSH_FORCE = 2500f;
    private static final float MITOSIS_PUSH_FORCE_SCALE = 6f;

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
    private float mitosisFastCheckInterval;
    private float huntingRange;  // How far molecules look for food
    private float huntingForce;  // Force strength toward food
    private float suctionRange;  // How far molecules pull food
    private float suctionForce;  // Base suction force strength
    private float organelleEnergyRate;  // Energy generation per organelle per second
    private boolean componentsNeedRebuild;
    
    // Mitosis event tracking
    private static class MitosisEvent {
        java.util.Set<Particle> particles;
        float lockTimeRemaining;
        float flashTimeRemaining;
        
        MitosisEvent(java.util.Set<Particle> particles, float lockDuration, float flashDuration) {
            this.particles = particles;
            this.lockTimeRemaining = lockDuration;
            this.flashTimeRemaining = flashDuration;
        }
    }

    private static class BondBucket {
        final int cellX;
        final int cellY;
        final Array<Bond> bonds = new Array<>();
        BondBucket(int cellX, int cellY) {
            this.cellX = cellX;
            this.cellY = cellY;
        }
    }
    private final Array<MitosisEvent> activeMitosisEvents;

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
    this.mitosisFastCheckInterval = 1f;  // Fast-pass cadence to keep mitosis responsive
        this.mitosisCheckAccumulator = 0f;
        this.huntingRange = 15f;  // Default: look for food within 15 units
        this.huntingForce = 40f;  // Default: 40 force units toward food
        this.suctionRange = 8f;  // Default: pull food within 8 units
        this.suctionForce = 60f;  // Default: 60 force units for suction
        this.organelleEnergyRate = 8f;  // Default: 8 energy/sec per organelle
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
        this.activeMitosisEvents = new Array<>();
        this.componentsNeedRebuild = false;
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
    
    public void setHuntingRange(float range) {
        this.huntingRange = Math.max(0f, range);
    }
    
    public float getHuntingRange() {
        return huntingRange;
    }
    
    public void setHuntingForce(float force) {
        this.huntingForce = Math.max(0f, force);
    }
    
    public float getHuntingForce() {
        return huntingForce;
    }
    
    public void setSuctionRange(float range) {
        this.suctionRange = Math.max(0f, range);
    }
    
    public float getSuctionRange() {
        return suctionRange;
    }
    
    public void setSuctionForce(float force) {
        this.suctionForce = Math.max(0f, force);
    }
    
    public float getSuctionForce() {
        return suctionForce;
    }
    
    public void setOrganelleEnergyRate(float rate) {
        this.organelleEnergyRate = Math.max(0f, rate);
    }
    
    public float getOrganelleEnergyRate() {
        return organelleEnergyRate;
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

    public Array<Particle> queryRadius(float x, float y, float radius) {
        return spatialGrid.queryRadius(x, y, radius);
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
        componentsNeedRebuild = true;
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
        
        // Apply multipliers and add 10% extra length for spacing
        float restLength = baseRestLength * lengthMult * 1.1f;
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
        destroyBond(bond, true, false);
    }
    
    /**
     * Destroy a bond, optionally applying a violent reaction force.
     * The strength of the reaction is proportional to the bond's break force threshold.
     */
    public void destroyBond(Bond bond, boolean applyViolentReaction) {
        destroyBond(bond, applyViolentReaction, false);
    }

    public void destroyBond(Bond bond, boolean applyViolentReaction, boolean forceDestroy) {
        if (bond == null || !bond.isActive()) {
            return;
        }

        // Skip destruction while mitosis protection is active unless forced
        if (!forceDestroy && isBondProtectedByMitosis(bond)) {
            return;
        }
        
        // Apply violent reaction before deactivating
        if (applyViolentReaction) {
            applyBondBreakReaction(bond);
        }
        
        bond.deactivate();
        activeBonds.removeValue(bond, true);
        bondPool.free(bond);
        componentsNeedRebuild = true;
        
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
        
        // Apply hunting behavior (weak molecules seek food)
        applyHuntingBehavior();
        
        // Apply food suction (molecules pull nearby food toward them)
        applyFoodSuction();
        
        // Apply crowding pressure (repulsion in dense regions)
        applyCrowdingPressure();
        
        // Apply intramolecular spacing (prevent clumping within molecules)
        applyIntramolecularSpacing();
        
        // Age particles and bonds
        updateAging(delta);
        
        // Check for replication opportunities
        processReplication(delta);
        
        // Check for oversized molecules and trigger mitosis
        processMitosis(delta);
        
        // Remove particles that ran out of energy
        removeDeadParticles();
        
        // Eject or consume dead particles from living molecules
        processDeadParticlesInMolecules();
        
        // Process corpse consumption by molecular structures
        processCellInteriorConsumption();
        
        // Process organelle absorption (GREEN particles become trapped energy generators)
        processOrganelleAbsorption();
        
        // Process organelle containment, boundary pushing, and inter-organelle bonding
        processOrganelleContainment();
        
        // Break organelle bonds when cells separate
        pruneOrganelleBonds();
        
        // Check for crossing bonds within same molecule and break them
        detectAndBreakCrossingBonds();
        
        // Apply rotational shear forces to break peripheral bonds
        applyRotationalShear();
        
        // Apply bond constraints before Box2D step (with organelle strengthening)
        for (int i = activeBonds.size - 1; i >= 0; i--) {
            Bond bond = activeBonds.get(i);
            if (bond.applyConstraint(activeParticles)) {
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
        
        // Organelle energy generation - organelles produce energy for nearby bonded particles
        applyOrganelleEnergyGeneration(delta);
    }
    
    /**
     * Organelles generate energy for nearby particles in the same molecule.
     * Acts like photosynthesis - passive energy production that scales with organelle count.
     */
    private void applyOrganelleEnergyGeneration(float delta) {
        final float ORGANELLE_RANGE = 5.0f; // How far organelles can provide energy
        
        if (organelleEnergyRate <= 0f) return; // Organelle energy generation disabled
        
        // Group particles by molecule
        com.badlogic.gdx.utils.IntMap<Array<Particle>> moleculeGroups = new com.badlogic.gdx.utils.IntMap<>();
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            if (particle.isOrganelle()) continue; // Organelles themselves don't receive energy
            
            int componentId = particle.getComponentId();
            Array<Particle> group = moleculeGroups.get(componentId);
            if (group == null) {
                group = new Array<>();
                moleculeGroups.put(componentId, group);
            }
            group.add(particle);
        }
        
        // Find all organelles
        Array<Particle> organelles = new Array<>();
        for (Particle p : activeParticles) {
            if (p.isActive() && p.isOrganelle()) {
                organelles.add(p);
            }
        }
        
        // For each molecule, calculate energy gain from nearby organelles
        for (com.badlogic.gdx.utils.IntMap.Entry<Array<Particle>> entry : moleculeGroups) {
            Array<Particle> molecule = entry.value;
            int moleculeComponentId = entry.key;
            
            // Find organelles that belong to this molecule (same component ID)
            Array<Particle> moleculeOrganelles = new Array<>();
            for (Particle org : organelles) {
                // Organelles don't bond, so we check spatial proximity to determine membership
                // An organelle belongs to a molecule if it's close to any particle in that molecule
                for (Particle p : molecule) {
                    Vector2 orgPos = org.getPosition();
                    Vector2 pPos = p.getPosition();
                    float dx = orgPos.x - pPos.x;
                    float dy = orgPos.y - pPos.y;
                    float distSq = dx * dx + dy * dy;
                    
                    if (distSq < ORGANELLE_RANGE * ORGANELLE_RANGE) {
                        moleculeOrganelles.add(org);
                        break; // Found membership, don't need to check other particles
                    }
                }
            }
            
            if (moleculeOrganelles.size == 0) continue; // No organelles, no energy generation
            
            // Calculate energy generation: scales with organelle count (diminishing returns)
            // Formula: base * count * (1 - 0.05 * count) to prevent exponential growth
            // 1 organelle = 1.0x, 2 = 1.9x, 3 = 2.7x, 4 = 3.4x, 5 = 3.75x (plateaus)
            float organelleCount = moleculeOrganelles.size;
            float generationMultiplier = organelleCount * (1.0f - 0.05f * Math.min(organelleCount, 10f));
            float totalEnergyGeneration = organelleEnergyRate * generationMultiplier * delta;
            
            // Distribute energy equally to all particles in molecule
            float energyPerParticle = totalEnergyGeneration / molecule.size;
            for (Particle p : molecule) {
                p.addEnergy(energyPerParticle);
            }
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
     * Apply hunting behavior - molecules with weak bonds actively seek food particles.
     * This creates predator-like behavior where energy-depleted structures pursue sustenance.
     */
    private void applyHuntingBehavior() {
        if (huntingRange <= 0f || huntingForce <= 0f) return; // Hunting disabled
        
        final float WEAK_THRESHOLD = 0.6f; // Consider bonds weak if < 60% of base strength
        
        // Group particles by molecule to calculate average bond strength
        com.badlogic.gdx.utils.IntMap<Array<Particle>> moleculeGroups = new com.badlogic.gdx.utils.IntMap<>();
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            
            int componentId = particle.getComponentId();
            Array<Particle> group = moleculeGroups.get(componentId);
            if (group == null) {
                group = new Array<>();
                moleculeGroups.put(componentId, group);
            }
            group.add(particle);
        }

        // Precompute bond strength aggregates per component (avoids O(M * B) scans)
        IntFloatMap componentBondForceSum = new IntFloatMap();
        IntIntMap componentBondCounts = new IntIntMap();
        for (Bond bond : activeBonds) {
            if (!bond.isActive()) continue;
            Particle a = bond.getParticleA();
            Particle b = bond.getParticleB();
            int componentA = a.getComponentId();
            if (componentA != b.getComponentId()) continue; // Only consider intra-molecule bonds
            float sum = componentBondForceSum.get(componentA, 0f) + bond.getEffectiveBreakForce();
            componentBondForceSum.put(componentA, sum);
            componentBondCounts.put(componentA, componentBondCounts.get(componentA, 0) + 1);
        }
        
        // For each molecule, check if it needs to hunt
        for (com.badlogic.gdx.utils.IntMap.Entry<Array<Particle>> entry : moleculeGroups) {
            Array<Particle> molecule = entry.value;
            int componentId = entry.key;
            
            if (molecule.size < 2) continue; // Solo particles don't hunt cooperatively
            
            // Calculate average energy level of the molecule
            float avgEnergy = 0f;
            for (Particle p : molecule) {
                avgEnergy += p.getEnergy();
            }
            avgEnergy /= molecule.size;
            
            // Check average bond strength for this molecule
            int bondCount = componentBondCounts.get(componentId, 0);
            if (bondCount == 0) continue;
            float avgBondStrength = componentBondForceSum.get(componentId, 0f) / bondCount;
            
            // Determine if molecule is "hungry" (weak bonds or low energy)
            float normalizedStrength = avgBondStrength / 200f; // 200 is base break force
            boolean isHungry = normalizedStrength < WEAK_THRESHOLD || avgEnergy < 40f;
            
            if (!isHungry) continue;
            
            // Calculate molecule center of mass
            float cx = 0, cy = 0;
            for (Particle p : molecule) {
                Vector2 pos = p.getPosition();
                cx += pos.x;
                cy += pos.y;
            }
            cx /= molecule.size;
            cy /= molecule.size;
            
            // Find nearest food particle (energy-rich particle)
            Particle nearestFood = null;
            float nearestDistSq = huntingRange * huntingRange;
            
            for (Particle food : activeParticles) {
                if (!food.isActive() || !food.isEnergyRich()) continue;
                
                Vector2 foodPos = food.getPosition();
                float dx = foodPos.x - cx;
                float dy = foodPos.y - cy;
                float distSq = dx * dx + dy * dy;
                
                if (distSq < nearestDistSq) {
                    nearestDistSq = distSq;
                    nearestFood = food;
                }
            }
            
            // If food found, all particles in molecule move toward it
            if (nearestFood != null) {
                Vector2 foodPos = nearestFood.getPosition();
                
                // Calculate hunting intensity based on how desperate the molecule is
                float desperation = 1.0f - normalizedStrength; // 0 = strong, 1 = very weak
                desperation = Math.max(desperation, (40f - avgEnergy) / 40f); // Also factor in energy
                desperation = Math.max(0f, Math.min(1f, desperation));
                
                for (Particle p : molecule) {
                    Vector2 pPos = p.getPosition();
                    float dx = foodPos.x - pPos.x;
                    float dy = foodPos.y - pPos.y;
                    float dist = (float) Math.sqrt(dx * dx + dy * dy);
                    
                    if (dist > 0.1f) {
                        // Apply hunting force
                        float forceX = (dx / dist) * huntingForce * desperation;
                        float forceY = (dy / dist) * huntingForce * desperation;
                        p.getBody().applyForceToCenter(forceX, forceY, true);
                    }
                }
            }
        }
    }
    
    /**
     * Apply food suction - molecules pull nearby energy-rich particles toward them.
     * This dramatically increases eating efficiency by actively drawing food into consumption range.
     * Larger molecules exert stronger suction (more "mouths" to feed).
     */
    private void applyFoodSuction() {
        if (suctionRange <= 0f || suctionForce <= 0f) return; // Suction disabled
        
        // Group living particles by molecule
        com.badlogic.gdx.utils.IntMap<Array<Particle>> moleculeGroups = new com.badlogic.gdx.utils.IntMap<>();
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            if (particle.isOrganelle()) continue; // Organelles don't participate in suction
            
            int componentId = particle.getComponentId();
            Array<Particle> group = moleculeGroups.get(componentId);
            if (group == null) {
                group = new Array<>();
                moleculeGroups.put(componentId, group);
            }
            group.add(particle);
        }
        
        // For each molecule, find nearby food and pull it in
        for (com.badlogic.gdx.utils.IntMap.Entry<Array<Particle>> entry : moleculeGroups) {
            Array<Particle> molecule = entry.value;
            
            if (molecule.size < 2) continue; // Solo particles don't create suction
            
            // Calculate molecule center of mass
            float cx = 0, cy = 0;
            for (Particle p : molecule) {
                Vector2 pos = p.getPosition();
                cx += pos.x;
                cy += pos.y;
            }
            cx /= molecule.size;
            cy /= molecule.size;
            
            // Suction strength scales with molecule size (more particles = stronger pull)
            // Formula: log-based scaling to prevent extreme forces
            float sizeMultiplier = 1.0f + (float) Math.log(molecule.size) * 0.3f;
            
            // Find all food particles within suction range (both living energy-rich and corpses)
            Array<Particle> nearbyCandidates = queryRadius(cx, cy, suctionRange);
            for (Particle food : nearbyCandidates) {
                if (!food.isActive()) continue;
                
                // Pull in energy-rich particles (living food) and corpses (dead particles)
                boolean isFood = food.isEnergyRich() && food.isAlive();
                boolean isCorpse = !food.isAlive();
                
                if (!isFood && !isCorpse) continue;
                
                // Don't pull particles from this molecule
                if (molecule.contains(food, true)) continue;
                
                Vector2 foodPos = food.getPosition();
                float dx = foodPos.x - cx;
                float dy = foodPos.y - cy;
                float distSq = dx * dx + dy * dy;
                float dist = (float) Math.sqrt(distSq);
                
                if (dist > suctionRange || dist < 0.5f) continue; // Too far or too close
                
                // Calculate suction force using inverse-square-like falloff
                // Stronger when food is closer, scales with molecule size
                float distanceFactor = 1.0f - (dist / suctionRange); // 1.0 at center, 0.0 at edge
                float suctionForceMagnitude = suctionForce * sizeMultiplier * distanceFactor * distanceFactor;
                
                // Corpses are pulled more strongly (2x) since they're the primary food source
                if (isCorpse) {
                    suctionForceMagnitude *= 2.0f;
                }
                
                // Apply force toward molecule center
                float forceX = -(dx / dist) * suctionForceMagnitude;
                float forceY = -(dy / dist) * suctionForceMagnitude;
                food.getBody().applyForceToCenter(forceX, forceY, true);
            }
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
     * Apply intramolecular spacing forces - particles within the same molecule 
     * repel each other slightly to prevent clumping and encourage surface area expansion.
     * This creates more open, spread-out molecular structures without breaking bonds.
     */
    private void applyIntramolecularSpacing() {
        final float SPACING_RADIUS = 3.5f; // Particles within this distance repel
        final float SPACING_STRENGTH = 25f; // Base repulsion force
        
        // Group particles by molecule
        com.badlogic.gdx.utils.IntMap<Array<Particle>> moleculeGroups = new com.badlogic.gdx.utils.IntMap<>();
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            
            int componentId = particle.getComponentId();
            Array<Particle> group = moleculeGroups.get(componentId);
            if (group == null) {
                group = new Array<>();
                moleculeGroups.put(componentId, group);
            }
            group.add(particle);
        }
        
        // For each molecule, apply spacing forces between its particles
        for (com.badlogic.gdx.utils.IntMap.Entry<Array<Particle>> entry : moleculeGroups) {
            Array<Particle> molecule = entry.value;
            
            if (molecule.size < 2) continue; // Solo particles don't need spacing
            
            // Apply repulsion between all pairs within the molecule
            for (int i = 0; i < molecule.size; i++) {
                Particle pA = molecule.get(i);
                Vector2 posA = pA.getPosition();
                
                for (int j = i + 1; j < molecule.size; j++) {
                    Particle pB = molecule.get(j);
                    Vector2 posB = pB.getPosition();
                    
                    float dx = posA.x - posB.x;
                    float dy = posA.y - posB.y;
                    float distSq = dx * dx + dy * dy;
                    float dist = (float) Math.sqrt(distSq);
                    
                    // Only apply spacing if within range
                    if (dist < SPACING_RADIUS && dist > 0.1f) {
                        // Spacing force falls off with distance (stronger when closer)
                        float spacingFactor = (SPACING_RADIUS - dist) / SPACING_RADIUS;
                        
                        // Reduce force if particles are directly bonded (let bonds handle it)
                        boolean directlyBonded = false;
                        for (Bond bond : activeBonds) {
                            if (!bond.isActive()) continue;
                            if ((bond.getParticleA() == pA && bond.getParticleB() == pB) ||
                                (bond.getParticleA() == pB && bond.getParticleB() == pA)) {
                                directlyBonded = true;
                                break;
                            }
                        }
                        
                        // If directly bonded, reduce spacing force by 70% (bonds already separate them)
                        float forceMult = directlyBonded ? 0.3f : 1.0f;
                        float force = SPACING_STRENGTH * spacingFactor * forceMult;
                        
                        // Apply equal and opposite forces
                        float forceX = (dx / dist) * force;
                        float forceY = (dy / dist) * force;
                        
                        pA.getBody().applyForceToCenter(forceX, forceY, true);
                        pB.getBody().applyForceToCenter(-forceX, -forceY, true);
                    }
                }
            }
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
        
        // Update mitosis events and remove expired ones
        for (int i = activeMitosisEvents.size - 1; i >= 0; i--) {
            MitosisEvent event = activeMitosisEvents.get(i);
            event.lockTimeRemaining -= delta;
            event.flashTimeRemaining -= delta;
            
            // Remove event when both timers expire
            if (event.lockTimeRemaining <= 0f && event.flashTimeRemaining <= 0f) {
                activeMitosisEvents.removeIndex(i);
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
        float targetInterval = Math.min(mitosisCheckInterval, mitosisFastCheckInterval);
        if (targetInterval <= 0f) {
            targetInterval = 1f;
        }
        if (mitosisCheckAccumulator < targetInterval) return;
        mitosisCheckAccumulator -= targetInterval;
        if (mitosisCheckAccumulator < 0f) {
            mitosisCheckAccumulator = 0f;
        }

        // Ensure component IDs reflect current connectivity before grouping molecules
        if (componentsNeedRebuild) {
            rebuildUnionFind();
        }
        
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
        int divisionsThisFrame = 0;
        for (com.badlogic.gdx.utils.IntMap.Entry<Array<Particle>> entry : componentGroups) {
            Array<Particle> molecule = entry.value;
            
            // Skip single particles or molecules too small to divide
            if (molecule.size < 3) continue;
            
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
                divisionsThisFrame++;
                if (divisionsThisFrame >= MAX_MITOSIS_DIVISIONS_PER_CHECK) {
                    break;
                }
            }
        }
    }
    
    /**
     * Divide a molecule by breaking bonds along its narrowest cross-section.
     * Strategy: Find the bond whose removal would best split the molecule into two equal halves.
     */
    private void divideMolecule(Array<Particle> moleculeParticles) {
        if (moleculeParticles.size < 3) return;

        Vector2 centerOfMass = calculateCenterOfMass(moleculeParticles);
        float maxRadius = calculateMaxRadius(moleculeParticles, centerOfMass);

        java.util.Set<Particle> moleculeSet = new java.util.HashSet<>(moleculeParticles.size);
        for (Particle particle : moleculeParticles) {
            moleculeSet.add(particle);
        }

        Array<Bond> moleculeBonds = collectMoleculeBonds(moleculeSet);
        if (moleculeBonds.size == 0) return;

        Vector2 primaryAxis = findPrimaryAxis(moleculeParticles, centerOfMass);
        float splittingBand = Math.max(MIN_MITOSIS_SPLIT_BAND, maxRadius * 0.2f);

        Array<Bond> bondsToBreak = new Array<>();
        for (Bond bond : moleculeBonds) {
            if (!bond.isActive()) continue;
            if (crossesCenterBand(bond, centerOfMass, primaryAxis, splittingBand)) {
                bondsToBreak.add(bond);
            }
        }

        if (bondsToBreak.size == 0) {
            Bond fallback = findClosestBondToPoint(moleculeBonds, centerOfMass);
            if (fallback != null) {
                bondsToBreak.add(fallback);
            }
        }

        if (bondsToBreak.size == 0) {
            return;
        }

        float releasedBondEnergy = 0f;
        for (Bond bond : bondsToBreak) {
            releasedBondEnergy += bond.getBreakForceThreshold();
            destroyBond(bond, false, true);
        }

        Array<Array<Particle>> daughterCells = extractMoleculeComponents(moleculeParticles, moleculeBonds, moleculeSet);
        if (daughterCells.size < 2) {
            return;
        }

        Array<Particle> daughterCell1 = null;
        Array<Particle> daughterCell2 = null;
        for (Array<Particle> cell : daughterCells) {
            if (daughterCell1 == null || cell.size > daughterCell1.size) {
                daughterCell2 = daughterCell1;
                daughterCell1 = cell;
            } else if (daughterCell2 == null || cell.size > daughterCell2.size) {
                daughterCell2 = cell;
            }
        }

        if (daughterCell1 == null || daughterCell2 == null) {
            return;
        }

        for (Array<Particle> cell : daughterCells) {
            if (cell == daughterCell1 || cell == daughterCell2) continue;
            if (daughterCell1.size <= daughterCell2.size) {
                daughterCell1.addAll(cell);
            } else {
                daughterCell2.addAll(cell);
            }
        }

        float energyPerCell = releasedBondEnergy * 0.5f;
        redistributeBondEnergy(daughterCell1, energyPerCell);
        redistributeBondEnergy(daughterCell2, energyPerCell);

        Vector2 center1 = calculateCenterOfMass(daughterCell1);
        Vector2 center2 = calculateCenterOfMass(daughterCell2);
        float pushForce = MathUtils.clamp(
            releasedBondEnergy * MITOSIS_PUSH_FORCE_SCALE,
            MIN_MITOSIS_PUSH_FORCE,
            MAX_MITOSIS_PUSH_FORCE
        );
        applyEqualOpposingPush(center1, center2, daughterCell1, daughterCell2, pushForce, primaryAxis);

        registerMitosisEvent(daughterCell1, daughterCell2);
    }

    private Array<Bond> collectMoleculeBonds(java.util.Set<Particle> moleculeSet) {
        Array<Bond> bonds = new Array<>();
        for (Bond bond : activeBonds) {
            if (!bond.isActive()) continue;
            if (moleculeSet.contains(bond.getParticleA()) && moleculeSet.contains(bond.getParticleB())) {
                bonds.add(bond);
            }
        }
        return bonds;
    }

    private Vector2 calculateCenterOfMass(Array<Particle> particles) {
        Vector2 center = new Vector2();
        if (particles.size == 0) {
            return center;
        }
        for (Particle particle : particles) {
            center.add(particle.getPosition());
        }
        center.scl(1f / particles.size);
        return center;
    }

    private float calculateMaxRadius(Array<Particle> particles, Vector2 center) {
        float maxRadius = 0f;
        for (Particle particle : particles) {
            Vector2 pos = particle.getPosition();
            float dx = pos.x - center.x;
            float dy = pos.y - center.y;
            float dist = (float) Math.sqrt(dx * dx + dy * dy);
            if (dist > maxRadius) {
                maxRadius = dist;
            }
        }
        return maxRadius;
    }

    private Vector2 findPrimaryAxis(Array<Particle> particles, Vector2 center) {
        Vector2 axis = new Vector2(1f, 0f);
        float maxDistSq = 0f;
        for (Particle particle : particles) {
            Vector2 pos = particle.getPosition();
            float dx = pos.x - center.x;
            float dy = pos.y - center.y;
            float distSq = dx * dx + dy * dy;
            if (distSq > maxDistSq) {
                maxDistSq = distSq;
                axis.set(dx, dy);
            }
        }
        if (axis.isZero(0.0001f)) {
            axis.set(1f, 0f);
        } else {
            axis.nor();
        }
        return axis;
    }

    private boolean crossesCenterBand(Bond bond, Vector2 center, Vector2 normal, float bandWidth) {
        Vector2 posA = bond.getParticleA().getPosition();
        Vector2 posB = bond.getParticleB().getPosition();
        float relAx = posA.x - center.x;
        float relAy = posA.y - center.y;
        float relBx = posB.x - center.x;
        float relBy = posB.y - center.y;
        float projA = relAx * normal.x + relAy * normal.y;
        float projB = relBx * normal.x + relBy * normal.y;
        if (Math.abs(projA) < 0.0001f) projA = 0f;
        if (Math.abs(projB) < 0.0001f) projB = 0f;

        if ((projA > 0f && projB > 0f) || (projA < 0f && projB < 0f)) {
            return false;
        }

        float midpointProjection = Math.abs((projA + projB) * 0.5f);
        return midpointProjection <= bandWidth;
    }

    private Bond findClosestBondToPoint(Array<Bond> bonds, Vector2 point) {
        Bond closest = null;
        float minDist = Float.MAX_VALUE;
        for (Bond bond : bonds) {
            if (!bond.isActive()) continue;
            Vector2 posA = bond.getParticleA().getPosition();
            Vector2 posB = bond.getParticleB().getPosition();
            float midX = (posA.x + posB.x) * 0.5f;
            float midY = (posA.y + posB.y) * 0.5f;
            float dist = Vector2.dst(midX, midY, point.x, point.y);
            if (dist < minDist) {
                minDist = dist;
                closest = bond;
            }
        }
        return closest;
    }

    private Array<Array<Particle>> extractMoleculeComponents(
        Array<Particle> moleculeParticles,
        Array<Bond> moleculeBonds,
        java.util.Set<Particle> moleculeSet
    ) {
        Array<Array<Particle>> components = new Array<>();
        java.util.Set<Particle> visited = new java.util.HashSet<>();
        Array<Particle> queue = new Array<>();

        for (Particle start : moleculeParticles) {
            if (!moleculeSet.contains(start) || visited.contains(start)) continue;
            queue.clear();
            queue.add(start);
            visited.add(start);

            Array<Particle> component = new Array<>();
            while (queue.size > 0) {
                Particle current = queue.pop();
                component.add(current);
                for (Bond bond : moleculeBonds) {
                    if (!bond.isActive()) continue;
                    Particle neighbor = null;
                    if (bond.getParticleA() == current) {
                        neighbor = bond.getParticleB();
                    } else if (bond.getParticleB() == current) {
                        neighbor = bond.getParticleA();
                    }

                    if (neighbor != null && moleculeSet.contains(neighbor) && !visited.contains(neighbor)) {
                        visited.add(neighbor);
                        queue.add(neighbor);
                    }
                }
            }

            if (component.size > 0) {
                components.add(component);
            }
        }

        return components;
    }

    private void redistributeBondEnergy(Array<Particle> particles, float energyShare) {
        if (particles.size == 0 || energyShare <= 0f) {
            return;
        }

        java.util.Set<Particle> particleSet = new java.util.HashSet<>(particles.size);
        for (Particle particle : particles) {
            particleSet.add(particle);
        }

        Array<Bond> recipientBonds = new Array<>();
        for (Bond bond : activeBonds) {
            if (!bond.isActive()) continue;
            if (particleSet.contains(bond.getParticleA()) && particleSet.contains(bond.getParticleB())) {
                recipientBonds.add(bond);
            }
        }

        if (recipientBonds.size == 0) {
            float perParticleEnergy = energyShare / particles.size;
            for (Particle particle : particles) {
                particle.addEnergy(perParticleEnergy);
            }
            return;
        }

        float energyPerBond = energyShare / recipientBonds.size;
        for (Bond bond : recipientBonds) {
            bond.increaseBreakForceThreshold(energyPerBond);
        }
    }

    private void applyEqualOpposingPush(
        Vector2 center1,
        Vector2 center2,
        Array<Particle> cell1,
        Array<Particle> cell2,
        float forceMagnitude,
        Vector2 fallbackDirection
    ) {
        if (cell1.size == 0 || cell2.size == 0) {
            return;
        }

        Vector2 direction = new Vector2(center2).sub(center1);
        if (direction.isZero(0.0001f)) {
            if (fallbackDirection != null) {
                direction.set(fallbackDirection);
            } else {
                direction.set(1f, 0f);
            }
        }

        if (!direction.isZero(0.0001f)) {
            direction.nor();
        } else {
            direction.set(1f, 0f);
        }

        float perParticleForce1 = forceMagnitude / cell1.size;
        float perParticleForce2 = forceMagnitude / cell2.size;

        for (Particle particle : cell1) {
            Body body = particle.getBody();
            if (body != null) {
                body.applyForceToCenter(-direction.x * perParticleForce1, -direction.y * perParticleForce1, true);
            }
        }

        for (Particle particle : cell2) {
            Body body = particle.getBody();
            if (body != null) {
                body.applyForceToCenter(direction.x * perParticleForce2, direction.y * perParticleForce2, true);
            }
        }
    }

    private void registerMitosisEvent(Array<Particle> daughterCell1, Array<Particle> daughterCell2) {
        float lockDuration = 3.5f;
        float flashDuration = 0.8f;
        java.util.Set<Particle> mitosisParticles = new java.util.HashSet<>();
        for (Particle particle : daughterCell1) {
            mitosisParticles.add(particle);
        }
        for (Particle particle : daughterCell2) {
            mitosisParticles.add(particle);
        }
        activeMitosisEvents.add(new MitosisEvent(mitosisParticles, lockDuration, flashDuration));
    }
    
    /**
     * Check if a particle is currently locked from mitosis (can't form bonds).
     */
    public boolean isParticleMitosisLocked(Particle particle) {
        for (MitosisEvent event : activeMitosisEvents) {
            if (event.lockTimeRemaining > 0f && event.particles.contains(particle)) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * Check if a particle is currently flashing from mitosis (visual effect).
     */
    public boolean isParticleMitosisFlashing(Particle particle) {
        for (MitosisEvent event : activeMitosisEvents) {
            if (event.flashTimeRemaining > 0f && event.particles.contains(particle)) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * Get the flash intensity for a particle (0.0 to 1.0).
     */
    public float getParticleMitosisFlashIntensity(Particle particle) {
        float maxFlash = 0f;
        for (MitosisEvent event : activeMitosisEvents) {
            if (event.flashTimeRemaining > 0f && event.particles.contains(particle)) {
                float intensity = event.flashTimeRemaining / 0.8f; // Normalize by flash duration
                maxFlash = Math.max(maxFlash, intensity);
            }
        }
        return maxFlash;
    }

    /**
     * Determine if a bond should be protected from breaking while mitosis cooldown is active.
     */
    private boolean isBondProtectedByMitosis(Bond bond) {
        Particle a = bond.getParticleA();
        Particle b = bond.getParticleB();
        for (MitosisEvent event : activeMitosisEvents) {
            if (event.lockTimeRemaining > 0f &&
                event.particles.contains(a) &&
                event.particles.contains(b)) {
                return true;
            }
        }
        return false;
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
                destroyBond(bond, false, true);  // No violent reaction on energy death
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
     * Process dead particles that are still bonded within living molecules.
     * Dead particles (corpses) should be ejected or consumed by the living structure.
     */
    private void processDeadParticlesInMolecules() {
        Array<Particle> deadParticlesToHandle = new Array<>();
        
        // Find all dead particles that still have bonds
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || particle.isAlive()) continue;
            
            // Check if this dead particle has any bonds
            boolean hasBonds = false;
            for (Bond bond : activeBonds) {
                if (!bond.isActive()) continue;
                if (bond.getParticleA() == particle || bond.getParticleB() == particle) {
                    hasBonds = true;
                    break;
                }
            }
            
            if (hasBonds) {
                deadParticlesToHandle.add(particle);
            }
        }
        
        // Process each dead particle
        for (Particle deadParticle : deadParticlesToHandle) {
            // Find all living particles bonded to this dead one
            Array<Particle> livingNeighbors = new Array<>();
            Array<Bond> bondsToBreak = new Array<>();
            
            for (Bond bond : activeBonds) {
                if (!bond.isActive()) continue;
                
                Particle other = null;
                if (bond.getParticleA() == deadParticle) {
                    other = bond.getParticleB();
                } else if (bond.getParticleB() == deadParticle) {
                    other = bond.getParticleA();
                }
                
                if (other != null) {
                    bondsToBreak.add(bond);
                    if (other.isAlive()) {
                        livingNeighbors.add(other);
                    }
                }
            }
            
            // If there are living neighbors, they consume the dead particle's energy
            if (livingNeighbors.size > 0) {
                float energyPerNeighbor = deadParticle.getEnergy() / livingNeighbors.size;
                for (Particle living : livingNeighbors) {
                    living.addEnergy(energyPerNeighbor);
                }
            }
            
            // Break all bonds to the dead particle (eject it)
            for (Bond bond : bondsToBreak) {
                destroyBond(bond, false, true); // Non-violent break (quiet ejection)
            }
            
            // Apply small outward force to eject the corpse
            if (livingNeighbors.size > 0) {
                Vector2 deadPos = deadParticle.getPosition();
                
                // Calculate center of living neighbors
                float cx = 0, cy = 0;
                for (Particle living : livingNeighbors) {
                    Vector2 pos = living.getPosition();
                    cx += pos.x;
                    cy += pos.y;
                }
                cx /= livingNeighbors.size;
                cy /= livingNeighbors.size;
                
                // Push corpse away from center (gentle ejection)
                float dx = deadPos.x - cx;
                float dy = deadPos.y - cy;
                float dist = (float) Math.sqrt(dx * dx + dy * dy);
                if (dist > 0.1f) {
                    float forceX = (dx / dist) * 6f; // Reduced from 20 to 6 (70% reduction)
                    float forceY = (dy / dist) * 6f;
                    deadParticle.getBody().applyForceToCenter(forceX, forceY, true);
                }
            }
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
     * Process organelle absorption - GREEN (TRUE) particles entering molecular interiors
     * become trapped organelles that strengthen bonds and generate energy.
     * This simulates cellular organelles like chloroplasts.
     */
    private void processOrganelleAbsorption() {
        // Group living particles by component ID (molecules)
        com.badlogic.gdx.utils.IntMap<Array<Particle>> moleculeGroups = new com.badlogic.gdx.utils.IntMap<>();
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive() || particle.isOrganelle()) continue;
            
            int componentId = particle.getComponentId();
            Array<Particle> group = moleculeGroups.get(componentId);
            if (group == null) {
                group = new Array<>();
                moleculeGroups.put(componentId, group);
            }
            group.add(particle);
        }
        
        // For each molecule with 3+ particles, check for GREEN particles inside
        for (com.badlogic.gdx.utils.IntMap.Entry<Array<Particle>> entry : moleculeGroups) {
            Array<Particle> molecule = entry.value;
            
            if (molecule.size < 3) continue; // Need at least 3 bonded particles for interior
            
            // Build convex hull of the molecule
            Array<Particle> hull = buildConvexHull(molecule);
            if (hull.size < 3) continue;
            
            // Check all GREEN particles to see if they're inside this molecule
            for (Particle candidate : activeParticles) {
                if (!candidate.isActive() || !candidate.isAlive()) continue;
                if (candidate.isOrganelle()) continue; // Already an organelle
                if (candidate.getType() != ParticleType.TRUE) continue; // Only GREEN particles
                if (molecule.contains(candidate, true)) continue; // Don't absorb our own particles
                
                // Check if candidate is inside the hull
                if (isPointInPolygon(candidate.getPosition(), hull)) {
                    // Convert to organelle!
                    candidate.setOrganelle(true);
                    candidate.setEnergy(100f); // Organelles are energy-saturated
                    
                    // Assign to this cell's component (so it knows which cell it belongs to)
                    int cellComponentId = molecule.get(0).getComponentId();
                    candidate.setComponentId(cellComponentId);
                    
                    // Keep some velocity so organelles can push against boundaries
                    // Don't completely stop them like before
                    Body body = candidate.getBody();
                    Vector2 vel = body.getLinearVelocity();
                    body.setLinearVelocity(vel.x * 0.3f, vel.y * 0.3f);
                    body.setAngularVelocity(0f);
                }
            }
        }
    }
    
    /**
     * Process organelle containment and boundary interactions.
     * Organelles push against the cell membrane (bonds) to create space while being contained.
     * Also handles organelle-to-organelle bonding within the same cell.
     */
    private void processOrganelleContainment() {
        final float BOUNDARY_PUSH_FORCE = 80f; // Force organelles exert on boundaries
        final float BOUNDARY_PUSH_DISTANCE = 1.5f; // Distance at which organelles start pushing
        final float ORGANELLE_BOND_DISTANCE = 2.0f; // Distance for organelle bonding
        
        // Find all organelles
        Array<Particle> organelles = new Array<>();
        for (Particle p : activeParticles) {
            if (p.isActive() && p.isOrganelle()) {
                organelles.add(p);
            }
        }
        
        if (organelles.size == 0) return;
        
        // For each organelle, apply containment and boundary pushing
        for (Particle organelle : organelles) {
            Vector2 orgPos = organelle.getPosition();
            int cellComponentId = organelle.getComponentId();
            
            // Find all bonds in the same cell (these form the boundary)
            Array<Bond> cellBonds = new Array<>();
            for (Bond bond : activeBonds) {
                if (!bond.isActive()) continue;
                
                Particle a = bond.getParticleA();
                Particle b = bond.getParticleB();
                
                // Skip bonds involving organelles
                if (a.isOrganelle() || b.isOrganelle()) continue;
                
                // Check if both particles are in the same cell as organelle
                if (a.getComponentId() == cellComponentId && b.getComponentId() == cellComponentId) {
                    cellBonds.add(bond);
                }
            }
            
            // Apply collision/pushing forces with cell boundary bonds
            for (Bond bond : cellBonds) {
                Particle a = bond.getParticleA();
                Particle b = bond.getParticleB();
                Vector2 posA = a.getPosition();
                Vector2 posB = b.getPosition();
                
                // Find closest point on bond to organelle
                Vector2 closestPoint = getClosestPointOnSegment(orgPos, posA, posB);
                float dx = orgPos.x - closestPoint.x;
                float dy = orgPos.y - closestPoint.y;
                float dist = (float) Math.sqrt(dx * dx + dy * dy);
                
                if (dist < BOUNDARY_PUSH_DISTANCE && dist > 0.01f) {
                    // Organelle is close to boundary - create pushing force
                    float pushStrength = (BOUNDARY_PUSH_DISTANCE - dist) / BOUNDARY_PUSH_DISTANCE;
                    
                    // Push organelle back (containment)
                    float containForceX = -(dx / dist) * BOUNDARY_PUSH_FORCE * pushStrength * 0.3f;
                    float containForceY = -(dy / dist) * BOUNDARY_PUSH_FORCE * pushStrength * 0.3f;
                    organelle.getBody().applyForceToCenter(containForceX, containForceY, true);
                    
                    // Push boundary particles outward (expansion)
                    float expandForceX = (dx / dist) * BOUNDARY_PUSH_FORCE * pushStrength;
                    float expandForceY = (dy / dist) * BOUNDARY_PUSH_FORCE * pushStrength;
                    
                    // Distribute force to both bond endpoints based on proximity
                    float distToA = orgPos.dst(posA);
                    float distToB = orgPos.dst(posB);
                    float totalDist = distToA + distToB;
                    
                    if (totalDist > 0.01f) {
                        // Closer endpoint gets more force
                        float weightA = (1.0f - distToA / totalDist);
                        float weightB = (1.0f - distToB / totalDist);
                        
                        a.getBody().applyForceToCenter(expandForceX * weightA, expandForceY * weightA, true);
                        b.getBody().applyForceToCenter(expandForceX * weightB, expandForceY * weightB, true);
                    }
                }
            }
        }
        
        // Process organelle-to-organelle bonding (only within same cell)
        processOrganelleBonding(organelles, ORGANELLE_BOND_DISTANCE);
    }
    
    /**
     * Get the closest point on a line segment to a given point.
     */
    private Vector2 getClosestPointOnSegment(Vector2 point, Vector2 segStart, Vector2 segEnd) {
        float dx = segEnd.x - segStart.x;
        float dy = segEnd.y - segStart.y;
        float lengthSq = dx * dx + dy * dy;
        
        if (lengthSq < 0.0001f) {
            // Segment is essentially a point
            return new Vector2(segStart);
        }
        
        // Project point onto line segment
        float t = ((point.x - segStart.x) * dx + (point.y - segStart.y) * dy) / lengthSq;
        t = Math.max(0f, Math.min(1f, t)); // Clamp to segment
        
        return new Vector2(segStart.x + t * dx, segStart.y + t * dy);
    }
    
    /**
     * Process bonding between organelles in the same cell.
     * Organelles can only bond with other organelles in the same cell.
     */
    private void processOrganelleBonding(Array<Particle> organelles, float bondDistance) {
        if (organelles.size < 2) return;
        
        // Check each pair of organelles
        for (int i = 0; i < organelles.size; i++) {
            Particle orgA = organelles.get(i);
            
            for (int j = i + 1; j < organelles.size; j++) {
                Particle orgB = organelles.get(j);
                
                // Only bond organelles in the same cell
                if (orgA.getComponentId() != orgB.getComponentId()) continue;
                
                // Check if already bonded
                boolean alreadyBonded = false;
                for (Bond bond : activeBonds) {
                    if (!bond.isActive()) continue;
                    if ((bond.getParticleA() == orgA && bond.getParticleB() == orgB) ||
                        (bond.getParticleA() == orgB && bond.getParticleB() == orgA)) {
                        alreadyBonded = true;
                        break;
                    }
                }
                
                if (alreadyBonded) continue;
                
                // Check distance
                float dist = orgA.getPosition().dst(orgB.getPosition());
                if (dist < bondDistance) {
                    // Create organelle bond (flexible, moderate strength)
                    Bond bond = createBond(orgA, orgB, BondType.ELASTIC, 80f, 3f, 250f);
                    
                    // The createBond method already handles union-find,
                    // but organelles should maintain their cell component ID,
                    // not form a new independent component
                    // (They're already part of the cell's component)
                }
            }
        }
    }
    
    /**
     * Break organelle bonds when their host cells separate.
     * If two organelles are no longer in the same component, their bond should break.
     */
    private void pruneOrganelleBonds() {
        Array<Bond> bondsToBreak = new Array<>();
        
        for (Bond bond : activeBonds) {
            if (!bond.isActive()) continue;
            
            Particle a = bond.getParticleA();
            Particle b = bond.getParticleB();
            
            // Check if this is an organelle-organelle bond
            if (a.isOrganelle() && b.isOrganelle()) {
                // If organelles are in different cells now, break the bond
                if (a.getComponentId() != b.getComponentId()) {
                    bondsToBreak.add(bond);
                }
            }
            
            // Also break bonds between organelles and non-organelles
            // Organelles should NEVER bond with regular particles
            if ((a.isOrganelle() && !b.isOrganelle()) || (!a.isOrganelle() && b.isOrganelle())) {
                bondsToBreak.add(bond);
            }
        }
        
        for (Bond bond : bondsToBreak) {
            destroyBond(bond, false, true); // Non-violent break
        }
    }
    
    /**
     * Build a convex hull from a set of particles for interior detection.
     */
    private Array<Particle> buildConvexHull(Array<Particle> particles) {
        if (particles.size < 3) return new Array<>();
        
        // Calculate centroid
        float cx = 0, cy = 0;
        for (Particle p : particles) {
            Vector2 pos = p.getPosition();
            cx += pos.x;
            cy += pos.y;
        }
        cx /= particles.size;
        cy /= particles.size;
        
        // Sort by angle from centroid
        Array<Particle> sorted = new Array<>(particles);
        final float centroidX = cx;
        final float centroidY = cy;
        
        sorted.sort((a, b) -> {
            Vector2 posA = a.getPosition();
            Vector2 posB = b.getPosition();
            float angleA = (float) Math.atan2(posA.y - centroidY, posA.x - centroidX);
            float angleB = (float) Math.atan2(posB.y - centroidY, posB.x - centroidX);
            return Float.compare(angleA, angleB);
        });
        
        return sorted;
    }
    
    /**
     * Check if a point is inside a polygon using ray casting algorithm.
     */
    private boolean isPointInPolygon(Vector2 point, Array<Particle> polygon) {
        if (polygon.size < 3) return false;
        
        int intersections = 0;
        for (int i = 0; i < polygon.size; i++) {
            Particle p1 = polygon.get(i);
            Particle p2 = polygon.get((i + 1) % polygon.size);
            
            Vector2 pos1 = p1.getPosition();
            Vector2 pos2 = p2.getPosition();
            
            if (rayIntersectsSegment(point, pos1, pos2)) {
                intersections++;
            }
        }
        
        return (intersections % 2) == 1;
    }
    
    /**
     * Check if a horizontal ray from point intersects the line segment.
     */
    private boolean rayIntersectsSegment(Vector2 point, Vector2 segA, Vector2 segB) {
        if (segA.y > segB.y) {
            Vector2 temp = segA;
            segA = segB;
            segB = temp;
        }
        
        if (point.y < segA.y || point.y >= segB.y) return false;
        if (point.x >= Math.max(segA.x, segB.x)) return false;
        if (point.x < Math.min(segA.x, segB.x)) return true;
        
        float xIntersection = segA.x + (point.y - segA.y) / (segB.y - segA.y) * (segB.x - segA.x);
        return point.x < xIntersection;
    }

    /**
     * Detect bonds that cross within the same molecule and break them with repulsive force.
     */
    private void detectAndBreakCrossingBonds() {
        Array<Bond> bondsToBreak = new Array<>();

        if (activeBonds.size == 0) return;

        float cellSize = spatialGrid.getCellSize();
        LongMap<BondBucket> bucketMap = new LongMap<>();
        Array<BondBucket> bucketList = new Array<>();

        // Bucket bonds by the cell containing their midpoint
        for (Bond bond : activeBonds) {
            if (!bond.isActive()) continue;
            Particle a = bond.getParticleA();
            Particle b = bond.getParticleB();
            if (a.getComponentId() != b.getComponentId()) continue; // Skip inter-component bonds

            Vector2 posA = a.getPosition();
            Vector2 posB = b.getPosition();
            float midX = (posA.x + posB.x) * 0.5f;
            float midY = (posA.y + posB.y) * 0.5f;
            int cellX = (int) Math.floor(midX / cellSize);
            int cellY = (int) Math.floor(midY / cellSize);
            long key = packCellKey(cellX, cellY);

            BondBucket bucket = bucketMap.get(key);
            if (bucket == null) {
                bucket = new BondBucket(cellX, cellY);
                bucketMap.put(key, bucket);
                bucketList.add(bucket);
            }
            bucket.bonds.add(bond);
        }

        // Compare bonds within each bucket and neighboring buckets
        for (BondBucket bucket : bucketList) {
            checkBucketForCrossings(bucket, bucket, bondsToBreak, true);

            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;
                    int neighborX = bucket.cellX + dx;
                    int neighborY = bucket.cellY + dy;
                    if (!shouldProcessBucketPair(bucket.cellX, bucket.cellY, neighborX, neighborY)) continue;
                    long neighborKey = packCellKey(neighborX, neighborY);
                    BondBucket neighbor = bucketMap.get(neighborKey);
                    if (neighbor == null) continue;
                    checkBucketForCrossings(bucket, neighbor, bondsToBreak, false);
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
    
    private void checkBucketForCrossings(BondBucket bucketA, BondBucket bucketB, Array<Bond> bondsToBreak, boolean sameBucket) {
        Array<Bond> bondsA = bucketA.bonds;
        Array<Bond> bondsB = bucketB.bonds;
        if (sameBucket) {
            for (int i = 0; i < bondsA.size; i++) {
                Bond bondA = bondsA.get(i);
                if (!bondA.isActive()) continue;
                for (int j = i + 1; j < bondsA.size; j++) {
                    Bond bondB = bondsA.get(j);
                    if (!bondB.isActive()) continue;
                    if (!areBondsInSameComponent(bondA, bondB)) continue;
                    if (bondA.intersects(bondB)) {
                        bondsToBreak.add(bondA);
                        bondsToBreak.add(bondB);
                    }
                }
            }
        } else {
            for (int i = 0; i < bondsA.size; i++) {
                Bond bondA = bondsA.get(i);
                if (!bondA.isActive()) continue;
                for (int j = 0; j < bondsB.size; j++) {
                    Bond bondB = bondsB.get(j);
                    if (!bondB.isActive()) continue;
                    if (!areBondsInSameComponent(bondA, bondB)) continue;
                    if (bondA.intersects(bondB)) {
                        bondsToBreak.add(bondA);
                        bondsToBreak.add(bondB);
                    }
                }
            }
        }
    }

    private boolean areBondsInSameComponent(Bond a, Bond b) {
        return a.getParticleA().getComponentId() == b.getParticleA().getComponentId();
    }

    private boolean shouldProcessBucketPair(int cellAX, int cellAY, int cellBX, int cellBY) {
        if (cellBX > cellAX) return true;
        if (cellBX == cellAX && cellBY > cellAY) return true;
        return false;
    }

    private long packCellKey(int cellX, int cellY) {
        return (((long) cellX) << 32) ^ (cellY & 0xffffffffL);
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
            destroyBond(activeBonds.get(i), false, true);  // No violent reactions on cleanup
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
        componentsNeedRebuild = false;
    }

    @Override
    public void dispose() {
        clearAllBonds();
        clearAllParticles();
        world.dispose();
    }
}
