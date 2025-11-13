package com.automaton.core.render;

import com.automaton.core.physics.Bond;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer.ShapeType;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Disposable;

/**
 * Renders bonds as lines using ShapeRenderer. Note: this is a temporary solution
 * for the MVP. For thousands of bonds, this should be replaced with mesh batching
 * or instanced rendering for better performance.
 */
public final class BondRenderer implements Disposable {
    private final ShapeRenderer shapeRenderer;
    private final Color bondColor;

    public BondRenderer() {
        this.shapeRenderer = new ShapeRenderer();
        this.bondColor = new Color(0.5f, 0.5f, 0.5f, 1f);
    }

    public void render(Array<Bond> bonds, OrthographicCamera camera) {
        shapeRenderer.setProjectionMatrix(camera.combined);
        shapeRenderer.begin(ShapeType.Line);
        shapeRenderer.setColor(bondColor);
        
        for (Bond bond : bonds) {
            Vector2 posA = bond.getParticleA().getPosition();
            Vector2 posB = bond.getParticleB().getPosition();
            shapeRenderer.line(posA.x, posA.y, posB.x, posB.y);
        }
        
        shapeRenderer.end();
    }

    @Override
    public void dispose() {
        shapeRenderer.dispose();
    }
}
