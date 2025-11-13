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

    public PhysicsWorld(Vector2 gravity, int initialParticleCapacity) {
        this.world = new World(gravity, true);
        this.particlePool = new ParticlePool(initialParticleCapacity, Integer.MAX_VALUE);
        this.bondPool = new BondPool(initialParticleCapacity * 2, Integer.MAX_VALUE);
        this.activeParticles = new Array<>(false, initialParticleCapacity);
        this.activeBonds = new Array<>(false, initialParticleCapacity * 2);
        // Cell size = 2.5 units (typical particle spacing in demo)
        this.spatialGrid = new SpatialHashGrid(2.5f);
        this.unionFind = new UnionFind();
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
        particle.init(body, type, collisionRadius, visualRadius, 0f);
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
        
        Vector2 posA = a.getPosition();
        Vector2 posB = b.getPosition();
        float restLength = posA.dst(posB);
        
        Bond bond = bondPool.obtain();
        bond.init(a, b, type, restLength, stiffness, damping, breakForceThreshold);
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
        if (bond == null || !bond.isActive()) {
            return;
        }
        bond.deactivate();
        activeBonds.removeValue(bond, true);
        bondPool.free(bond);
        
        // Note: Union-Find doesn't support efficient split operations.
        // For accurate component tracking after many bond breaks, call rebuildUnionFind()
    }

    public void step(float delta, int velocityIterations, int positionIterations) {
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
                    
                    // Apply repulsive force at intersection point
                    applyRepulsiveForceAtCrossing(bondA, bondB);
                }
            }
        }
        
        // Break all crossing bonds
        for (Bond bond : bondsToBreak) {
            if (bond.isActive()) {
                destroyBond(bond);
            }
        }
    }

    /**
     * Apply explosive repulsive force to push apart nodes when bonds cross.
     */
    private void applyRepulsiveForceAtCrossing(Bond bondA, Bond bondB) {
        float repulsionStrength = 500f;  // Strong force to separate nodes
        
        // Get all 4 particles involved
        Particle a1 = bondA.getParticleA();
        Particle a2 = bondA.getParticleB();
        Particle b1 = bondB.getParticleA();
        Particle b2 = bondB.getParticleB();
        
        // Calculate rough intersection point (midpoint approximation)
        Vector2 centerA = new Vector2(a1.getPosition()).add(a2.getPosition()).scl(0.5f);
        Vector2 centerB = new Vector2(b1.getPosition()).add(b2.getPosition()).scl(0.5f);
        Vector2 crossPoint = new Vector2(centerA).add(centerB).scl(0.5f);
        
        // Apply forces pushing each particle away from crossing point
        applyRepulsionFromPoint(a1, crossPoint, repulsionStrength);
        applyRepulsionFromPoint(a2, crossPoint, repulsionStrength);
        applyRepulsionFromPoint(b1, crossPoint, repulsionStrength);
        applyRepulsionFromPoint(b2, crossPoint, repulsionStrength);
    }

    /**
     * Apply repulsive force to push particle away from a point.
     */
    private void applyRepulsionFromPoint(Particle particle, Vector2 point, float strength) {
        Vector2 pos = particle.getPosition();
        Vector2 dir = new Vector2(pos).sub(point);
        float dist = dir.len();
        
        if (dist > 0.001f) {
            dir.nor().scl(strength / (dist + 0.1f));  // Inverse distance falloff
            particle.getBody().applyForceToCenter(dir, true);
        }
    }

    public void clearAllBonds() {
        for (int i = activeBonds.size - 1; i >= 0; i--) {
            destroyBond(activeBonds.get(i));
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
