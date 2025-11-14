package com.automaton.core.render;

import com.automaton.core.physics.MovingEnergyField;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;

/**
 * Renders moving energy fields as tile-based gradients with opacity reflecting strength.
 */
public class MovingEnergyFieldRenderer {
    private final ShapeRenderer shapeRenderer;
    private final Color tmpColor = new Color();
    
    private static final float TILE_SIZE = 2.0f; // Size of each rendered tile
    
    public MovingEnergyFieldRenderer() {
        shapeRenderer = new ShapeRenderer();
        shapeRenderer.setAutoShapeType(true);
    }
    
    /**
     * Render all moving energy fields as tile grids with opacity gradient.
     * Each tile's opacity reflects the actual field strength at that position.
     */
    public void render(Array<MovingEnergyField> fields, OrthographicCamera camera) {
        // Enable blending for transparency
        Gdx.gl.glEnable(GL20.GL_BLEND);
        Gdx.gl.glBlendFunc(GL20.GL_SRC_ALPHA, GL20.GL_ONE_MINUS_SRC_ALPHA);
        
        shapeRenderer.setProjectionMatrix(camera.combined);
        shapeRenderer.begin(ShapeRenderer.ShapeType.Filled);
        
        for (MovingEnergyField field : fields) {
            if (!field.isActive()) continue;
            
            Vector2 pos = field.getPosition();
            float radius = field.getRadius();
            Color baseColor = field.getType().color;
            
            // Calculate tile bounds around the field
            float minX = pos.x - radius;
            float maxX = pos.x + radius;
            float minY = pos.y - radius;
            float maxY = pos.y + radius;
            
            // Render tiles within the field's influence
            for (float x = minX; x < maxX; x += TILE_SIZE) {
                for (float y = minY; y < maxY; y += TILE_SIZE) {
                    // Calculate actual energy at this tile position
                    float tileX = x + TILE_SIZE/2;
                    float tileY = y + TILE_SIZE/2;
                    float energy = field.getEnergyAt(tileX, tileY);
                    
                    if (energy > 0.01f) { // Skip nearly transparent tiles
                        // Opacity directly reflects field energy strength
                        // energy is 0-1 with quadratic falloff (1 at center, 0 at edge)
                        float alpha = energy * 0.35f; // Max 35% opacity at center
                        tmpColor.set(baseColor.r, baseColor.g, baseColor.b, alpha);
                        shapeRenderer.setColor(tmpColor);
                        shapeRenderer.rect(x, y, TILE_SIZE, TILE_SIZE);
                    }
                }
            }
        }
        
        shapeRenderer.end();
        
        // Restore blending state
        Gdx.gl.glDisable(GL20.GL_BLEND);
    }
    
    public void dispose() {
        shapeRenderer.dispose();
    }
}
