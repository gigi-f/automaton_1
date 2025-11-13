package com.automaton.core.physics;

import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.IntMap;

/**
 * Incremental Union-Find (Disjoint Set Union) data structure for tracking
 * connected components of particles. Updates only when bonds form/break.
 * 
 * Uses path compression and union by rank for near-constant amortized time.
 * Each particle is assigned a component ID that can be queried efficiently.
 */
public class UnionFind {
    private final IntMap<Integer> parent;  // particle ID -> parent ID
    private final IntMap<Integer> rank;    // particle ID -> rank (tree depth estimate)
    private final IntMap<Integer> componentSize; // root ID -> size of component
    
    public UnionFind() {
        this.parent = new IntMap<>();
        this.rank = new IntMap<>();
        this.componentSize = new IntMap<>();
    }
    
    /**
     * Add a particle to the union-find structure (as its own component).
     */
    public void addParticle(int particleId) {
        if (!parent.containsKey(particleId)) {
            parent.put(particleId, particleId);
            rank.put(particleId, 0);
            componentSize.put(particleId, 1);
        }
    }
    
    /**
     * Remove a particle from the union-find structure.
     * Note: This doesn't rebuild the tree, so component sizes may be inaccurate
     * until a full rebuild.
     */
    public void removeParticle(int particleId) {
        int root = find(particleId);
        parent.remove(particleId);
        rank.remove(particleId);
        
        // Decrease component size if this was tracking a valid root
        if (componentSize.containsKey(root)) {
            int size = componentSize.get(root, 1);
            componentSize.put(root, size - 1);
        }
    }
    
    /**
     * Find the root (component ID) of a particle with path compression.
     * This is the representative element of the connected component.
     */
    public int find(int particleId) {
        if (!parent.containsKey(particleId)) {
            addParticle(particleId);
            return particleId;
        }
        
        int p = parent.get(particleId);
        if (p != particleId) {
            // Path compression: make all nodes point directly to root
            int root = find(p);
            parent.put(particleId, root);
            return root;
        }
        return particleId;
    }
    
    /**
     * Union two particles' components when a bond forms.
     * Uses union by rank to keep tree shallow.
     * Returns true if they were in different components (union happened).
     */
    public boolean union(int particleA, int particleB) {
        int rootA = find(particleA);
        int rootB = find(particleB);
        
        if (rootA == rootB) {
            return false; // Already in same component
        }
        
        // Union by rank: attach smaller tree under root of larger tree
        int rankA = rank.get(rootA, 0);
        int rankB = rank.get(rootB, 0);
        
        if (rankA < rankB) {
            parent.put(rootA, rootB);
            int sizeA = componentSize.get(rootA, 1);
            int sizeB = componentSize.get(rootB, 1);
            componentSize.put(rootB, sizeA + sizeB);
            componentSize.remove(rootA);
        } else if (rankA > rankB) {
            parent.put(rootB, rootA);
            int sizeA = componentSize.get(rootA, 1);
            int sizeB = componentSize.get(rootB, 1);
            componentSize.put(rootA, sizeA + sizeB);
            componentSize.remove(rootB);
        } else {
            parent.put(rootB, rootA);
            rank.put(rootA, rankA + 1);
            int sizeA = componentSize.get(rootA, 1);
            int sizeB = componentSize.get(rootB, 1);
            componentSize.put(rootA, sizeA + sizeB);
            componentSize.remove(rootB);
        }
        
        return true;
    }
    
    /**
     * Check if two particles are in the same connected component.
     */
    public boolean connected(int particleA, int particleB) {
        return find(particleA) == find(particleB);
    }
    
    /**
     * Get the size of the component containing this particle.
     */
    public int getComponentSize(int particleId) {
        int root = find(particleId);
        return componentSize.get(root, 1);
    }
    
    /**
     * Clear all data and rebuild from scratch.
     * Should be called when many bonds break at once to avoid stale data.
     */
    public void clear() {
        parent.clear();
        rank.clear();
        componentSize.clear();
    }
    
    /**
     * Rebuild the union-find structure from current particles and bonds.
     * More efficient than incremental updates after many changes.
     */
    public void rebuild(Array<Particle> particles, Array<Bond> bonds) {
        clear();
        
        // Add all particles as separate components
        for (Particle particle : particles) {
            if (particle.isActive()) {
                addParticle(particle.hashCode());
            }
        }
        
        // Union particles connected by bonds
        for (Bond bond : bonds) {
            if (bond.isActive()) {
                union(bond.getParticleA().hashCode(), bond.getParticleB().hashCode());
            }
        }
    }
    
    /**
     * Get the number of distinct components.
     */
    public int getComponentCount() {
        return componentSize.size;
    }
}
