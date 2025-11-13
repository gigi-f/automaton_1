package com.automaton.core.physics;

import com.badlogic.gdx.utils.Pool;

/**
 * Pool for {@link Bond} instances to minimise garbage at high bond counts.
 */
final class BondPool extends Pool<Bond> {

    BondPool(int initialCapacity, int max) {
        super(initialCapacity, max);
    }

    @Override
    protected Bond newObject() {
        return new Bond();
    }
}
