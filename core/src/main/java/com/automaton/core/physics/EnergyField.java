package com.automaton.core.physics;

import com.badlogic.gdx.math.MathUtils;

/**
 * Represents a spatial energy field using a 2D grid.
 * Different regions have varying energy concentrations that particles can absorb.
 * Creates environmental gradients for chemotaxis and spatial heterogeneity.
 */
public class EnergyField {
    private final float[][] grid;
    private final int gridWidth;
    private final int gridHeight;
    private final float cellSize;
    private final float worldWidth;
    private final float worldHeight;
    private float fieldStrength;
    
    // Gradient parameters
    private float gradientCenterX;
    private float gradientCenterY;
    private float gradientRadius;
    
    /**
     * Create an energy field covering the specified world dimensions.
     * 
     * @param worldWidth Width of the world in physics units
     * @param worldHeight Height of the world in physics units
     * @param cellSize Size of each grid cell
     * @param fieldStrength Overall strength multiplier (0-1)
     */
    public EnergyField(float worldWidth, float worldHeight, float cellSize, float fieldStrength) {
        this.worldWidth = worldWidth;
        this.worldHeight = worldHeight;
        this.cellSize = cellSize;
        this.fieldStrength = fieldStrength;
        
        this.gridWidth = (int) Math.ceil(worldWidth / cellSize);
        this.gridHeight = (int) Math.ceil(worldHeight / cellSize);
        this.grid = new float[gridWidth][gridHeight];
        
        // Initialize with center gradient
        this.gradientCenterX = 0f;
        this.gradientCenterY = 0f;
        this.gradientRadius = Math.min(worldWidth, worldHeight) * 0.4f;
        
        updateGradient();
    }
    
    /**
     * Update the energy gradient pattern.
     * Creates a radial gradient centered at the specified position.
     */
    private void updateGradient() {
        float halfWidth = worldWidth * 0.5f;
        float halfHeight = worldHeight * 0.5f;
        
        for (int x = 0; x < gridWidth; x++) {
            for (int y = 0; y < gridHeight; y++) {
                // Convert grid coordinates to world coordinates
                float worldX = (x * cellSize) - halfWidth;
                float worldY = (y * cellSize) - halfHeight;
                
                // Calculate distance from gradient center
                float dx = worldX - gradientCenterX;
                float dy = worldY - gradientCenterY;
                float distance = (float) Math.sqrt(dx * dx + dy * dy);
                
                // Radial gradient: high at center, low at edges
                float normalizedDist = Math.min(distance / gradientRadius, 1.0f);
                float energy = (1.0f - normalizedDist) * fieldStrength;
                
                grid[x][y] = Math.max(0f, energy);
            }
        }
    }
    
    /**
     * Get the energy level at a world position.
     * Returns 0 if position is outside the field.
     */
    public float getEnergyAt(float worldX, float worldY) {
        int gridX = worldToGridX(worldX);
        int gridY = worldToGridY(worldY);
        
        if (gridX < 0 || gridX >= gridWidth || gridY < 0 || gridY >= gridHeight) {
            return 0f;
        }
        
        return grid[gridX][gridY];
    }
    
    /**
     * Calculate the energy gradient direction at a position.
     * Returns a normalized vector pointing toward higher energy.
     */
    public void getGradientDirection(float worldX, float worldY, float[] outDirection) {
        int gridX = worldToGridX(worldX);
        int gridY = worldToGridY(worldY);
        
        if (gridX < 1 || gridX >= gridWidth - 1 || gridY < 1 || gridY >= gridHeight - 1) {
            outDirection[0] = 0f;
            outDirection[1] = 0f;
            return;
        }
        
        // Calculate gradient using finite differences
        float energyRight = grid[gridX + 1][gridY];
        float energyLeft = grid[gridX - 1][gridY];
        float energyUp = grid[gridX][gridY + 1];
        float energyDown = grid[gridX][gridY - 1];
        
        float gradX = (energyRight - energyLeft) * 0.5f;
        float gradY = (energyUp - energyDown) * 0.5f;
        
        // Normalize
        float length = (float) Math.sqrt(gradX * gradX + gradY * gradY);
        if (length > 0.001f) {
            outDirection[0] = gradX / length;
            outDirection[1] = gradY / length;
        } else {
            outDirection[0] = 0f;
            outDirection[1] = 0f;
        }
    }
    
    /**
     * Set the gradient center position and radius.
     */
    public void setGradient(float centerX, float centerY, float radius) {
        this.gradientCenterX = centerX;
        this.gradientCenterY = centerY;
        this.gradientRadius = radius;
        updateGradient();
    }
    
    /**
     * Set the overall field strength (0-1).
     */
    public void setFieldStrength(float strength) {
        this.fieldStrength = MathUtils.clamp(strength, 0f, 1f);
        updateGradient();
    }
    
    public float getFieldStrength() {
        return fieldStrength;
    }
    
    /**
     * Randomize the gradient center position.
     */
    public void randomizeGradient() {
        float halfWidth = worldWidth * 0.5f;
        float halfHeight = worldHeight * 0.5f;
        
        gradientCenterX = MathUtils.random(-halfWidth * 0.5f, halfWidth * 0.5f);
        gradientCenterY = MathUtils.random(-halfHeight * 0.5f, halfHeight * 0.5f);
        gradientRadius = MathUtils.random(
            Math.min(worldWidth, worldHeight) * 0.2f,
            Math.min(worldWidth, worldHeight) * 0.6f
        );
        
        updateGradient();
    }
    
    private int worldToGridX(float worldX) {
        float halfWidth = worldWidth * 0.5f;
        return (int) ((worldX + halfWidth) / cellSize);
    }
    
    private int worldToGridY(float worldY) {
        float halfHeight = worldHeight * 0.5f;
        return (int) ((worldY + halfHeight) / cellSize);
    }
    
    // Getters for rendering
    public float getCellSize() {
        return cellSize;
    }
    
    public int getGridWidth() {
        return gridWidth;
    }
    
    public int getGridHeight() {
        return gridHeight;
    }
    
    /**
     * Get energy at a specific grid cell.
     */
    public float getEnergyAtGrid(int gridX, int gridY) {
        if (gridX < 0 || gridX >= gridWidth || gridY < 0 || gridY >= gridHeight) {
            return 0f;
        }
        return grid[gridX][gridY];
    }
}
