package com.automaton.core.physics;

import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Pool;

/**
 * Represents a bond (spring constraint) between two particles. Unlike Box2D joints,
 * this applies forces directly to the particle bodies each frame using verlet-style
 * soft constraints. This avoids the performance bottleneck of Box2D's joint solver
 * when dealing with thousands of bonds.
 */
public final class Bond implements Pool.Poolable {
    private static final float MIN_BREAK_FORCE = 5f;
    private static final float MAX_BREAK_FORCE = 2000f;
    private Particle particleA;
    private Particle particleB;
    private BondType bondType;
    private float restLength;
    private float stiffness;
    private float damping;
    private float breakForceThreshold;
    private boolean active;
    private float currentForce;
    private float age; // Age in seconds since bond creation
    private float initialRestLength;
    private float initialBreakForceThreshold;
    private boolean unbreakable;

    private final Vector2 tmpVec = new Vector2();
    private final Vector2 tmpVel = new Vector2();

    void init(Particle a, Particle b, BondType type, float restLength, float stiffness, 
              float damping, float breakForceThreshold) {
        this.particleA = a;
        this.particleB = b;
        this.bondType = type;
        this.restLength = restLength;
        this.stiffness = stiffness;
        this.damping = damping;
        this.breakForceThreshold = breakForceThreshold;
        this.initialRestLength = restLength;
        this.initialBreakForceThreshold = breakForceThreshold;
        this.active = true;
        this.currentForce = 0f;
        this.age = 0f; // Start at age 0
        this.unbreakable = false;
    }

    public boolean isActive() {
        return active;
    }

    public Particle getParticleA() {
        return particleA;
    }

    public Particle getParticleB() {
        return particleB;
    }

    public BondType getBondType() {
        return bondType;
    }

    public float getCurrentForce() {
        return currentForce;
    }

    public float getRestLength() {
        return restLength;
    }

    public float getBreakForceThreshold() {
        return breakForceThreshold;
    }

    public float getInitialRestLength() {
        return initialRestLength;
    }

    public float getInitialBreakForceThreshold() {
        return initialBreakForceThreshold;
    }

    public boolean isUnbreakable() {
        return unbreakable;
    }

    public void setUnbreakable(boolean unbreakable) {
        this.unbreakable = unbreakable;
    }

    public void increaseBreakForceThreshold(float delta) {
        if (Math.abs(delta) < MathUtils.FLOAT_ROUNDING_ERROR) {
            return;
        }
        breakForceThreshold = MathUtils.clamp(breakForceThreshold + delta, MIN_BREAK_FORCE, MAX_BREAK_FORCE);
    }

    public void setBreakForceThreshold(float value) {
        breakForceThreshold = MathUtils.clamp(value, MIN_BREAK_FORCE, MAX_BREAK_FORCE);
    }

    public void setRestLength(float value) {
        restLength = Math.max(0.05f, value);
    }
    
    public float getAge() {
        return age;
    }
    
    public void updateAge(float delta) {
        this.age += delta;
    }

    /**
     * Reduce the accumulated age on this bond, restoring its effective strength.
     */
    public void rejuvenate(float delta) {
        if (delta <= 0f) {
            return;
        }
        this.age = Math.max(0f, this.age - delta);
    }
    
    /**
     * Get the effective break force threshold, reduced by aging but strengthened by nearby organelles.
     * Bonds become more fragile over time (lose 50% strength after 60 seconds).
     * Organelles within range strengthen bonds exponentially.
     */
    public float getEffectiveBreakForce() {
        if (unbreakable) {
            return Float.MAX_VALUE;
        }
        // Aging formula: strength = base * (1 - 0.5 * min(age/60, 1))
        // After 60 seconds, bond is at 50% original strength
        float agingFactor = 1.0f - (0.5f * Math.min(age / 60f, 1.0f));
        return breakForceThreshold * agingFactor;
    }
    
    /**
     * Get the effective break force with organelle strengthening applied.
     * Each organelle within range multiplies bond strength.
     */
    public float getEffectiveBreakForce(com.badlogic.gdx.utils.Array<Particle> allParticles) {
        float baseStrength = getEffectiveBreakForce();
        
        if (allParticles == null || !particleA.isActive() || !particleB.isActive()) {
            return baseStrength;
        }
        
        // Find midpoint of bond
        Vector2 posA = particleA.getPosition();
        Vector2 posB = particleB.getPosition();
        float midX = (posA.x + posB.x) / 2f;
        float midY = (posA.y + posB.y) / 2f;
        
        // Count organelles within influence range
        final float ORGANELLE_RANGE = 4.0f; // How far organelles can strengthen bonds
        final float STRENGTH_PER_ORGANELLE = 1.5f; // Each organelle multiplies strength by this
        
        int nearbyOrganelles = 0;
        for (Particle p : allParticles) {
            if (!p.isActive() || !p.isOrganelle()) continue;
            
            Vector2 pPos = p.getPosition();
            float dx = pPos.x - midX;
            float dy = pPos.y - midY;
            float distSq = dx * dx + dy * dy;
            
            if (distSq < ORGANELLE_RANGE * ORGANELLE_RANGE) {
                nearbyOrganelles++;
            }
        }
        
        // Apply exponential strengthening: strength * (1.5^organelleCount)
        float organelleBoost = (float) Math.pow(STRENGTH_PER_ORGANELLE, nearbyOrganelles);
        return baseStrength * organelleBoost;
    }

    /**
     * Check if this bond intersects with another bond.
     * Uses line segment intersection test.
     */
    public boolean intersects(Bond other) {
        if (!active || !other.active) return false;
        if (this == other) return false;
        
        // Safety check: ensure particles are still active
        if (!particleA.isActive() || !particleB.isActive() ||
            !other.particleA.isActive() || !other.particleB.isActive()) {
            return false;
        }
        
        // Don't check bonds that share a particle
        if (particleA == other.particleA || particleA == other.particleB ||
            particleB == other.particleA || particleB == other.particleB) {
            return false;
        }
        
        Vector2 p1 = particleA.getPosition();
        Vector2 p2 = particleB.getPosition();
        Vector2 p3 = other.particleA.getPosition();
        Vector2 p4 = other.particleB.getPosition();
        
        return lineSegmentsIntersect(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y);
    }

    /**
     * Test if two line segments intersect.
     */
    private boolean lineSegmentsIntersect(float x1, float y1, float x2, float y2,
                                          float x3, float y3, float x4, float y4) {
        float denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
        
        // Parallel or coincident lines
        if (Math.abs(denom) < 0.0001f) {
            return false;
        }
        
        float ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
        float ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;
        
        // Intersection occurs if both ua and ub are between 0 and 1
        return ua >= 0f && ua <= 1f && ub >= 0f && ub <= 1f;
    }

    /**
     * Applies spring constraint forces to both particles. Should be called each physics frame.
     * Returns true if the bond should break (force exceeded threshold).
     */
    public boolean applyConstraint() {
        return applyConstraint(null);
    }
    
    /**
     * Applies spring constraint forces to both particles with organelle strengthening.
     * Returns true if the bond should break (force exceeded threshold).
     */
    public boolean applyConstraint(com.badlogic.gdx.utils.Array<Particle> allParticles) {
        if (!active || particleA == null || particleB == null 
            || !particleA.isActive() || !particleB.isActive()) {
            return true;
        }

        Vector2 posA = particleA.getPosition();
        Vector2 posB = particleB.getPosition();
        
        tmpVec.set(posB).sub(posA);
        float distance = tmpVec.len();
        
        if (distance < 0.0001f) {
            return false;
        }

        float extension = distance - restLength;
        tmpVec.scl(1f / distance);

        Vector2 velA = particleA.getBody().getLinearVelocity();
        Vector2 velB = particleB.getBody().getLinearVelocity();
        tmpVel.set(velB).sub(velA);
        float relativeVelocity = tmpVel.dot(tmpVec);

        float springForce = extension * stiffness;
        float dampingForce = relativeVelocity * damping;
        float totalForce = springForce + dampingForce;

        currentForce = Math.abs(totalForce);
        
        // Use effective break force (reduced by aging, boosted by organelles)
        float effectiveBreakForce = allParticles != null ? 
            getEffectiveBreakForce(allParticles) : getEffectiveBreakForce();
            
        if (currentForce > effectiveBreakForce) {
            return true;
        }

        tmpVec.scl(totalForce);
        particleA.getBody().applyForceToCenter(tmpVec, true);
        tmpVec.scl(-1f);
        particleB.getBody().applyForceToCenter(tmpVec, true);

        return false;
    }

    void deactivate() {
        active = false;
    }

    @Override
    public void reset() {
        particleA = null;
        particleB = null;
        bondType = null;
        restLength = 0f;
        stiffness = 0f;
        damping = 0f;
        breakForceThreshold = 0f;
        active = false;
        currentForce = 0f;
        age = 0f;
        initialRestLength = 0f;
        initialBreakForceThreshold = 0f;
        unbreakable = false;
    }
}
