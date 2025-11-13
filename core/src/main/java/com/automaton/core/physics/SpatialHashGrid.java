package com.automaton.core.physics;

import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.ObjectSet;

import java.util.HashMap;
import java.util.Map;

/**
 * Spatial hash grid for efficient proximity queries.
 * Divides space into uniform cells and tracks which particles are in each cell.
 * Provides O(k) neighbor queries instead of O(n²) brute force.
 */
public class SpatialHashGrid {
    private final float cellSize;
    private final Map<Long, Array<Particle>> grid;
    private final Array<Particle> queryResults;
    private final ObjectSet<Particle> nearbySet;
    
    public SpatialHashGrid(float cellSize) {
        this.cellSize = cellSize;
        this.grid = new HashMap<>();
        this.queryResults = new Array<>();
        this.nearbySet = new ObjectSet<>();
    }
    
    /**
     * Clear all cells in the grid.
     */
    public void clear() {
        for (Array<Particle> cell : grid.values()) {
            cell.clear();
        }
        grid.clear();
    }
    
    /**
     * Insert a particle into the grid based on its current position.
     */
    public void insert(Particle particle) {
        if (!particle.isActive()) return;
        
        Vector2 pos = particle.getPosition();
        long cellKey = getCellKey(pos.x, pos.y);
        
        Array<Particle> cell = grid.get(cellKey);
        if (cell == null) {
            cell = new Array<>();
            grid.put(cellKey, cell);
        }
        cell.add(particle);
    }
    
    /**
     * Update the entire grid with current particle positions.
     * Call this once per frame after physics step.
     */
    public void update(Array<Particle> particles) {
        clear();
        for (Particle particle : particles) {
            insert(particle);
        }
    }
    
    /**
     * Query all particles within a given radius of a position.
     * Returns deduplicated results (particles may span multiple cells).
     * 
     * @param x World X coordinate
     * @param y World Y coordinate
     * @param radius Search radius
     * @return Array of nearby particles
     */
    public Array<Particle> queryRadius(float x, float y, float radius) {
        queryResults.clear();
        nearbySet.clear();
        
        // Calculate cell range to check
        int minCellX = (int) Math.floor((x - radius) / cellSize);
        int maxCellX = (int) Math.floor((x + radius) / cellSize);
        int minCellY = (int) Math.floor((y - radius) / cellSize);
        int maxCellY = (int) Math.floor((y + radius) / cellSize);
        
        // Check all cells in range
        for (int cellY = minCellY; cellY <= maxCellY; cellY++) {
            for (int cellX = minCellX; cellX <= maxCellX; cellX++) {
                long cellKey = getCellKey(cellX, cellY);
                Array<Particle> cell = grid.get(cellKey);
                
                if (cell != null) {
                    for (Particle particle : cell) {
                        if (!particle.isActive()) continue;
                        
                        // Deduplicate (particle might be in multiple cells if on boundary)
                        if (nearbySet.contains(particle)) continue;
                        
                        // Distance check
                        Vector2 pos = particle.getPosition();
                        float dx = pos.x - x;
                        float dy = pos.y - y;
                        float distSq = dx * dx + dy * dy;
                        
                        if (distSq <= radius * radius) {
                            nearbySet.add(particle);
                            queryResults.add(particle);
                        }
                    }
                }
            }
        }
        
        return queryResults;
    }
    
    /**
     * Query all particles within radius of a given particle (excluding the particle itself).
     */
    public Array<Particle> queryNeighbors(Particle particle, float radius) {
        Vector2 pos = particle.getPosition();
        Array<Particle> results = queryRadius(pos.x, pos.y, radius);
        
        // Remove the query particle itself from results
        results.removeValue(particle, true);
        
        return results;
    }
    
    /**
     * Generate a unique cell key from cell coordinates.
     * Uses Cantor pairing function for spatial hashing.
     */
    private long getCellKey(float worldX, float worldY) {
        int cellX = (int) Math.floor(worldX / cellSize);
        int cellY = (int) Math.floor(worldY / cellSize);
        return getCellKey(cellX, cellY);
    }
    
    private long getCellKey(int cellX, int cellY) {
        // Cantor pairing function: maps 2D coordinates to unique 1D key
        // https://en.wikipedia.org/wiki/Pairing_function#Cantor_pairing_function
        long a = cellX >= 0 ? 2L * cellX : -2L * cellX - 1;
        long b = cellY >= 0 ? 2L * cellY : -2L * cellY - 1;
        long c = (a >= b) ? (a * a + a + b) : (a + b * b);
        return c;
    }
    
    /**
     * Get the cell size used by this grid.
     */
    public float getCellSize() {
        return cellSize;
    }
    
    /**
     * Get the number of occupied cells in the grid.
     */
    public int getOccupiedCellCount() {
        return grid.size();
    }
}
