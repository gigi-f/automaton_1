package com.automaton.core.physics;

import com.badlogic.gdx.utils.Pool;

/**
 * Pool for {@link Particle} instances to minimise garbage at high particle counts.
 */
final class ParticlePool extends Pool<Particle> {

    ParticlePool(int initialCapacity, int max) {
        super(initialCapacity, max);
    }

    @Override
    protected Particle newObject() {
        return new Particle();
    }
}
