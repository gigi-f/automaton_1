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
 * Renders bonds as lines using ShapeRenderer with color gradient based on strength.
 * Red = fragile (low break force), Green = strong (high break force).
 * Note: this is a temporary solution for the MVP. For thousands of bonds, 
 * this should be replaced with mesh batching or instanced rendering for better performance.
 */
public final class BondRenderer implements Disposable {
    private final ShapeRenderer shapeRenderer;
    private final Color tmpColor;
    
    // Base break force value for normalization (200 is the default)
    private static final float BASE_BREAK_FORCE = 200f;
    private static final float MIN_BREAK_FORCE = 50f;   // Very fragile (red)
    private static final float MAX_BREAK_FORCE = 600f;  // Very strong (green)

    public BondRenderer() {
        this.shapeRenderer = new ShapeRenderer();
        this.tmpColor = new Color();
    }

    public void render(Array<Bond> bonds, OrthographicCamera camera) {
        shapeRenderer.setProjectionMatrix(camera.combined);
        shapeRenderer.begin(ShapeType.Line);
        
        for (Bond bond : bonds) {
            Vector2 posA = bond.getParticleA().getPosition();
            Vector2 posB = bond.getParticleB().getPosition();
            
            // Calculate color based on EFFECTIVE break force (includes aging)
            float breakForce = bond.getEffectiveBreakForce();
            getColorForStrength(breakForce, tmpColor);
            shapeRenderer.setColor(tmpColor);
            
            shapeRenderer.line(posA.x, posA.y, posB.x, posB.y);
        }
        
        shapeRenderer.end();
    }
    
    /**
     * Calculate bond color based on break force strength.
     * Red (fragile) -> Yellow (medium) -> Green (strong)
     */
    private void getColorForStrength(float breakForce, Color out) {
        // Normalize break force to 0-1 range
        float normalized = (breakForce - MIN_BREAK_FORCE) / (MAX_BREAK_FORCE - MIN_BREAK_FORCE);
        normalized = Math.max(0f, Math.min(1f, normalized)); // Clamp to [0, 1]
        
        if (normalized < 0.5f) {
            // Red to Yellow (0.0 to 0.5)
            float t = normalized * 2f;
            out.r = 1f;
            out.g = t;
            out.b = 0f;
        } else {
            // Yellow to Green (0.5 to 1.0)
            float t = (normalized - 0.5f) * 2f;
            out.r = 1f - t;
            out.g = 1f;
            out.b = 0f;
        }
        out.a = 1f;
    }

    @Override
    public void dispose() {
        shapeRenderer.dispose();
    }
}
