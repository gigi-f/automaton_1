package com.automaton.core.physics;

import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.Filter;
import com.badlogic.gdx.physics.box2d.World;
import com.badlogic.gdx.utils.Pool;

/**
 * Represents a single particle (node) backed by a Box2D body. The {@link Particle}
 * is poolable so that large particle counts can be handled without generating
 * excessive garbage.
 */
public final class Particle implements Pool.Poolable {
    /** Collision category for all particles. */
    public static final short CATEGORY_PARTICLE = 0x0001;
    /**
     * Collision mask for particles. Set to CATEGORY_PARTICLE to enable particle-particle collisions.
     */
    public static final short MASK_PARTICLE = 0x0001;  // Enabled: particles collide with each other

    private Body body;
    private ParticleType type;
    private float collisionRadius;
    private float visualRadius;
    private float energy;
    private boolean active;
    private int componentId; // Component ID from union-find
    private boolean isEnergyRich; // Can this particle be consumed for energy?
    private float age; // Age in seconds since creation
    private boolean isAlive; // Living particles can form bonds, dead ones cannot
    private boolean isOrganelle; // Is this particle trapped as an organelle within a molecule?
    private float mitosisLockTimer; // Time remaining before can rebond after mitosis
    private float mitosisFlashTimer; // Time remaining for visual flash effect

    void init(Body body,
              ParticleType type,
              float collisionRadius,
              float visualRadius,
              float energy) {
        this.body = body;
        this.type = type;
        this.collisionRadius = collisionRadius;
        this.visualRadius = visualRadius;
        this.energy = energy;
        this.active = true;
        this.componentId = this.hashCode(); // Initially its own component
        this.isEnergyRich = false; // Normal particles start as non-edible
        this.age = 0f; // Start at age 0
        this.isAlive = true; // Start alive
        this.isOrganelle = false; // Start as normal particle
        this.mitosisLockTimer = 0f; // No lock initially
        this.mitosisFlashTimer = 0f; // No flash initially

        Filter filter = body.getFixtureList().first().getFilterData();
        filter.categoryBits = CATEGORY_PARTICLE;
        filter.maskBits = MASK_PARTICLE;
        body.getFixtureList().first().setFilterData(filter);
    }

    public boolean isActive() {
        return active;
    }

    public ParticleType getType() {
        return type;
    }

    public Vector2 getPosition() {
        return body.getPosition();
    }

    public Body getBody() {
        return body;
    }

    public float getVisualRadius() {
        return visualRadius;
    }

    public float getCollisionRadius() {
        return collisionRadius;
    }

    public float getEnergy() {
        return energy;
    }
    
    public void setEnergy(float energy) {
        this.energy = Math.max(0f, Math.min(100f, energy));
    }

    public void addEnergy(float delta) {
        setEnergy(energy + delta);
    }
    
    public void consumeEnergy(float amount) {
        setEnergy(energy - amount);
    }
    
    public boolean isEnergyRich() {
        return isEnergyRich;
    }
    
    public void setEnergyRich(boolean energyRich) {
        this.isEnergyRich = energyRich;
    }
    
    public float getAge() {
        return age;
    }
    
    public void updateAge(float delta) {
        this.age += delta;
    }
    
    public boolean isAlive() {
        return isAlive;
    }
    
    public void setAlive(boolean alive) {
        this.isAlive = alive;
    }
    
    public void kill() {
        this.isAlive = false;
        this.isEnergyRich = true; // Dead particles become consumable
    }
    
    public boolean isOrganelle() {
        return isOrganelle;
    }
    
    public void setOrganelle(boolean organelle) {
        this.isOrganelle = organelle;
    }

    public int getComponentId() {
        return componentId;
    }

    public void setComponentId(int componentId) {
        this.componentId = componentId;
    }
    
    public float getMitosisLockTimer() {
        return mitosisLockTimer;
    }
    
    public void setMitosisLock(float duration) {
        this.mitosisLockTimer = duration;
    }
    
    public void updateMitosisLockTimer(float delta) {
        if (mitosisLockTimer > 0f) {
            mitosisLockTimer -= delta;
            if (mitosisLockTimer < 0f) mitosisLockTimer = 0f;
        }
    }
    
    public boolean isMitosisLocked() {
        return mitosisLockTimer > 0f;
    }
    
    public float getMitosisFlashTimer() {
        return mitosisFlashTimer;
    }
    
    public void setMitosisFlash(float duration) {
        this.mitosisFlashTimer = duration;
    }
    
    public void updateMitosisFlashTimer(float delta) {
        if (mitosisFlashTimer > 0f) {
            mitosisFlashTimer -= delta;
            if (mitosisFlashTimer < 0f) mitosisFlashTimer = 0f;
        }
    }
    
    public boolean isMitosisFlashing() {
        return mitosisFlashTimer > 0f;
    }

    void deactivate(World world) {
        if (!active) {
            return;
        }
        active = false;
        if (body != null && world != null && !world.isLocked()) {
            world.destroyBody(body);
        }
        body = null;
    }

    @Override
    public void reset() {
        body = null;
        type = null;
        collisionRadius = 0f;
        visualRadius = 0f;
        energy = 0f;
        active = false;
        isEnergyRich = false;
        age = 0f;
        isAlive = true;
        isOrganelle = false;
        mitosisLockTimer = 0f;
        mitosisFlashTimer = 0f;
    }
}
