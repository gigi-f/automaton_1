package com.automaton.core.render;

import com.automaton.core.physics.EnergyField;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.Vector2;

/**
 * Renders the environmental energy field as a colored background grid.
 * High energy regions appear as darker gray, low energy as pure black.
 */
public class EnergyFieldRenderer {
    private final ShapeRenderer shapeRenderer;
    private final Color tmpColor = new Color();
    
    public EnergyFieldRenderer() {
        shapeRenderer = new ShapeRenderer();
        shapeRenderer.setAutoShapeType(true);
    }
    
    /**
     * Render the energy field as a grid of colored rectangles.
     * @param field The energy field to visualize
     * @param camera The camera for projection
     * @param worldWidth Width of the world in physics units
     * @param worldHeight Height of the world in physics units
     */
    public void render(EnergyField field, OrthographicCamera camera, float worldWidth, float worldHeight) {
        shapeRenderer.setProjectionMatrix(camera.combined);
        shapeRenderer.begin(ShapeRenderer.ShapeType.Filled);
        
        float cellSize = field.getCellSize();
        int gridWidth = field.getGridWidth();
        int gridHeight = field.getGridHeight();
        
        // Calculate starting position (bottom-left corner)
        float startX = -worldWidth / 2f;
        float startY = -worldHeight / 2f;
        
        // Draw each grid cell with color based on energy level
        for (int x = 0; x < gridWidth; x++) {
            for (int y = 0; y < gridHeight; y++) {
                float energy = field.getEnergyAtGrid(x, y);
                
                // Map energy (0-1) to grayscale: 0 = black, 1 = dark gray (0.3)
                float intensity = energy * 0.3f;
                tmpColor.set(intensity, intensity, intensity, 1f);
                
                float worldX = startX + x * cellSize;
                float worldY = startY + y * cellSize;
                
                shapeRenderer.setColor(tmpColor);
                shapeRenderer.rect(worldX, worldY, cellSize, cellSize);
            }
        }
        
        shapeRenderer.end();
    }
    
    public void dispose() {
        shapeRenderer.dispose();
    }
}
