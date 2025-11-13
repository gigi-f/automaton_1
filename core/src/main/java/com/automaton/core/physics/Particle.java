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

    public void addEnergy(float delta) {
        energy += delta;
    }

    public int getComponentId() {
        return componentId;
    }

    public void setComponentId(int componentId) {
        this.componentId = componentId;
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
    }
}
