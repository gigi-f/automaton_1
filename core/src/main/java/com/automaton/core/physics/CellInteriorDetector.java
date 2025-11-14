package com.automaton.core.physics;

import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.IntMap;
import com.badlogic.gdx.utils.ObjectSet;

/**
 * Detects when corpses enter the interior space of molecular structures (cells).
 * Uses polygon containment tests on the convex hull of bonded particle groups.
 */
public class CellInteriorDetector {
    private final Array<Vector2> tmpHull = new Array<>();
    private final IntMap<Array<Particle>> componentParticles = new IntMap<>();
    
    /**
     * Process all corpses and check if they're inside any molecular structure.
     * If a corpse is inside, distribute its energy to the molecule and mark for removal.
     * 
     * @param particles All particles (living and dead)
     * @param corpsesToRemove Output: corpses that were consumed
     */
    public void processCorpseConsumption(Array<Particle> particles, Array<Particle> corpsesToRemove) {
        corpsesToRemove.clear();
        componentParticles.clear();
        
        // Group living particles by component ID (molecules)
        for (Particle particle : particles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            
            int componentId = particle.getComponentId();
            Array<Particle> group = componentParticles.get(componentId);
            if (group == null) {
                group = new Array<>();
                componentParticles.put(componentId, group);
            }
            group.add(particle);
        }
        
        // For each component with 3+ particles (can form an interior), check corpses
        for (IntMap.Entry<Array<Particle>> entry : componentParticles) {
            Array<Particle> moleculeParticles = entry.value;
            
            // Need at least 3 particles to form a closed interior
            if (moleculeParticles.size < 3) continue;
            
            // Build convex hull of this molecule
            buildConvexHull(moleculeParticles, tmpHull);
            
            // Check each corpse against this hull
            for (Particle corpse : particles) {
                if (!corpse.isActive() || corpse.isAlive()) continue;
                if (corpsesToRemove.contains(corpse, true)) continue;
                
                Vector2 corpsePos = corpse.getPosition();
                
                // Test if corpse is inside this molecule's hull
                if (isPointInPolygon(corpsePos, tmpHull)) {
                    // Consume the corpse: distribute energy to all particles in this molecule
                    float energyPerParticle = corpse.getEnergy() / moleculeParticles.size;
                    for (Particle moleculeParticle : moleculeParticles) {
                        moleculeParticle.addEnergy(energyPerParticle);
                    }
                    
                    // Mark corpse for removal
                    corpsesToRemove.add(corpse);
                }
            }
        }
    }
    
    /**
     * Build a convex hull from particle positions using Graham scan algorithm.
     */
    private void buildConvexHull(Array<Particle> particles, Array<Vector2> outHull) {
        outHull.clear();
        if (particles.size < 3) return;
        
        // For simplicity with small particle counts, use a bounding polygon approach:
        // Create a polygon from particle positions sorted by angle around centroid
        
        // Calculate centroid
        float cx = 0, cy = 0;
        for (Particle p : particles) {
            Vector2 pos = p.getPosition();
            cx += pos.x;
            cy += pos.y;
        }
        cx /= particles.size;
        cy /= particles.size;
        
        // Sort particles by angle from centroid
        Array<ParticleAngle> sortedParticles = new Array<>();
        for (Particle p : particles) {
            Vector2 pos = p.getPosition();
            float angle = (float) Math.atan2(pos.y - cy, pos.x - cx);
            sortedParticles.add(new ParticleAngle(p, angle));
        }
        sortedParticles.sort();
        
        // Build hull from sorted particles
        for (ParticleAngle pa : sortedParticles) {
            outHull.add(pa.particle.getPosition().cpy());
        }
    }
    
    /**
     * Test if a point is inside a polygon using ray casting algorithm.
     */
    private boolean isPointInPolygon(Vector2 point, Array<Vector2> polygon) {
        if (polygon.size < 3) return false;
        
        int intersections = 0;
        for (int i = 0; i < polygon.size; i++) {
            Vector2 v1 = polygon.get(i);
            Vector2 v2 = polygon.get((i + 1) % polygon.size);
            
            // Ray casting: count intersections with edges
            if (rayIntersectsSegment(point, v1, v2)) {
                intersections++;
            }
        }
        
        // Odd number of intersections = inside
        return (intersections % 2) == 1;
    }
    
    /**
     * Check if a horizontal ray from point to the right intersects line segment v1-v2.
     */
    private boolean rayIntersectsSegment(Vector2 point, Vector2 v1, Vector2 v2) {
        // Edge is horizontal, no intersection with horizontal ray
        if (v1.y == v2.y) return false;
        
        // Point is outside vertical bounds of edge
        if (point.y < Math.min(v1.y, v2.y) || point.y > Math.max(v1.y, v2.y)) {
            return false;
        }
        
        // Calculate x coordinate where edge crosses point.y
        float xIntersect = v1.x + (point.y - v1.y) * (v2.x - v1.x) / (v2.y - v1.y);
        
        // Ray goes to the right, so only count if intersection is to the right of point
        return xIntersect > point.x;
    }
    
    /**
     * Helper class for sorting particles by angle.
     */
    private static class ParticleAngle implements Comparable<ParticleAngle> {
        final Particle particle;
        final float angle;
        
        ParticleAngle(Particle particle, float angle) {
            this.particle = particle;
            this.angle = angle;
        }
        
        @Override
        public int compareTo(ParticleAngle other) {
            return Float.compare(this.angle, other.angle);
        }
    }
}
