package com.automaton.core.physics;

import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.CircleShape;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.World;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Disposable;
import com.badlogic.gdx.utils.IntFloatMap;
import com.badlogic.gdx.utils.IntIntMap;
import com.badlogic.gdx.utils.IntMap;
import com.badlogic.gdx.utils.LongMap;
import com.badlogic.gdx.utils.ObjectFloatMap;
import com.badlogic.gdx.utils.ObjectMap;
import com.badlogic.gdx.utils.ObjectSet;
import java.util.Arrays;

/**
 * Wrapper around the Box2D {@link World} managing particle lifecycle, pooling and
 * stepping. Bonds/springs are handled elsewhere but are expected to act on the
 * bodies exposed by this class.
 */
public final class PhysicsWorld implements Disposable {
    private static final float DEFAULT_LINEAR_DAMPING = 0.05f;  // Minimal damping for endless motion
    private static final float DEFAULT_ANGULAR_DAMPING = 0.05f;
    private static final float DEFAULT_PARTICLE_DENSITY = 1f;
    private static final float DEFAULT_PARTICLE_FRICTION = 0.0f;  // Zero friction
    private static final float DEFAULT_PARTICLE_RESTITUTION = 1.0f;  // Perfectly elastic collisions
    private static final int MAX_MITOSIS_DIVISIONS_PER_CHECK = 2;
    private static final float MIN_MITOSIS_SPLIT_BAND = 0.75f;
    private static final float MIN_MITOSIS_PUSH_FORCE = 400f;
    private static final float MAX_MITOSIS_PUSH_FORCE = 2500f;
    private static final float MITOSIS_PUSH_FORCE_SCALE = 6f;
    private static final float MAX_ORGANELLE_ENERGY_RATE = 50f;
    private static final float BOND_RESTORATION_RATE = 300f;
    private static final float BOND_REJUVENATION_RATE = 60f;
    private static final int MAX_ANCHOR_CONNECTIONS = 3;
    private static final float ANCHOR_STIFFNESS = 70f;
    private static final float ANCHOR_DAMPING = 5f;
    private static final float ANCHOR_BREAK_FORCE = 5000f;
    private static final float ANCHOR_REST_SLACK = 1.08f;
    private static final float ANCHOR_REST_ADJUST_RATE = 6f;
    private static final float DEFAULT_ANGULAR_SPACING_STRENGTH = 40f;
    private static final float DEFAULT_ANGULAR_SPACING_DAMPING = 2.5f;
    private static final float DEFAULT_ANGULAR_SPACING_TORQUE_LIMIT = 600f;
    private static final float BOUNDARY_TARGET_ANGLE_DEG = 130f;
    private static final float BOUNDARY_TARGET_COS = MathUtils.cosDeg(BOUNDARY_TARGET_ANGLE_DEG);
    private static final float DEFAULT_AREA_TARGET_SCALE = 1.15f;
    private static final float DEFAULT_AREA_PRESSURE_STRENGTH = 120f;

    private final World world;
    private final ParticlePool particlePool;
    private final BondPool bondPool;
    private final Array<Particle> activeParticles;
    private final Array<Bond> activeBonds;
    private final SpatialHashGrid spatialGrid;
    private final UnionFind unionFind;
    private final BondProperties bondProperties;
    private final EnergyField energyField;
    private final Array<MovingEnergyField> movingFields;
    private final CellInteriorDetector cellInteriorDetector;
    private final Array<Particle> corpsesToRemove;
    private final ObjectMap<CellStructure, Array<Particle>> organellesByCellCache;
    private final Array<Array<Particle>> organelleListPool;
    private final Array<Bond> anchorBondBuffer;
    private final Array<Particle> anchorBoundaryBuffer;
    private final ObjectSet<Particle> anchorTargetSet;
    private final float[] anchorDistanceScratch;
    private final ObjectMap<Particle, Array<Particle>> spacingAdjacencyCache;
    private final Array<Array<Particle>> spacingAdjacencyPool;
    private float reactionViolenceMultiplier;
    private float energyDecayRate;
    private float bondEnergyCost;
    private float foodSpawnRate;
    private float foodSpawnAccumulator;
    private float replicationEnergyThreshold;
    private float replicationCheckInterval;
    private float replicationCheckAccumulator;
    private float mutationRate;
    private float environmentalEnergyRate;
    private float chemotaxisStrength;
    private int mitosisThreshold;  // Maximum molecule size before division
    private float mitosisCheckInterval;
    private float mitosisCheckAccumulator;
    private float mitosisFastCheckInterval;
    private float huntingRange;  // How far molecules look for food
    private float huntingForce;  // Force strength toward food
    private float suctionRange;  // How far molecules pull food
    private float suctionForce;  // Base suction force strength
    private float organelleEnergyRate;  // Energy generation per organelle per second
    private float organelleReplicationInterval;
    private float angularSpacingStrength;
    private float angularSpacingDamping;
    private float angularSpacingTorqueLimit;
    private float areaTargetScale;
    private float areaPressureStrength;
    private boolean componentsNeedRebuild;
    private final ObjectFloatMap<Particle> organelleReplicationTimers;
    private final Array<Particle> organelleBuffer;
    
    // Mitosis event tracking
    private static class MitosisEvent {
        java.util.Set<Particle> particles;
        float lockTimeRemaining;
        float flashTimeRemaining;
        
        MitosisEvent(java.util.Set<Particle> particles, float lockDuration, float flashDuration) {
            this.particles = particles;
            this.lockTimeRemaining = lockDuration;
            this.flashTimeRemaining = flashDuration;
        }
    }

    private static class BondBucket {
        final int cellX;
        final int cellY;
        final Array<Bond> bonds = new Array<>();
        BondBucket(int cellX, int cellY) {
            this.cellX = cellX;
            this.cellY = cellY;
        }
    }

    private static class DirectedEdge {
        final Particle from;
        final Particle to;
        final float angle;
        boolean visited;

        DirectedEdge(Particle from, Particle to) {
            this.from = from;
            this.to = to;
            Vector2 fromPos = from.getPosition();
            Vector2 toPos = to.getPosition();
            this.angle = MathUtils.atan2(toPos.y - fromPos.y, toPos.x - fromPos.x);
            this.visited = false;
        }
    }

    private static class ClosestBoundaryInfo {
        Particle particleA;
        Particle particleB;
        final Vector2 closestPoint = new Vector2();
        float distanceSq = Float.MAX_VALUE;

        void reset() {
            particleA = null;
            particleB = null;
            distanceSq = Float.MAX_VALUE;
            closestPoint.setZero();
        }
    }

    private static class CellStructure {
        final int componentId;
        final Array<Vector2> vertices = new Array<>();
        final Array<Particle> boundaryParticles = new Array<>();
        final Array<Particle> memberParticles = new Array<>();
        final ObjectSet<Particle> memberSet = new ObjectSet<>();
        float minX = Float.MAX_VALUE;
        float maxX = -Float.MAX_VALUE;
        float minY = Float.MAX_VALUE;
        float maxY = -Float.MAX_VALUE;
        float area;

        CellStructure(int componentId) {
            this.componentId = componentId;
        }

        void addVertex(Particle particle) {
            boundaryParticles.add(particle);
            Vector2 position = particle.getPosition();
            vertices.add(position.cpy());
            registerBounds(position);
        }

        void registerBounds(Vector2 pos) {
            minX = Math.min(minX, pos.x);
            maxX = Math.max(maxX, pos.x);
            minY = Math.min(minY, pos.y);
            maxY = Math.max(maxY, pos.y);
        }

        boolean contains(Vector2 point) {
            if (vertices.size < 3) return false;
            if (point.x < minX || point.x > maxX || point.y < minY || point.y > maxY) {
                return false;
            }
            int intersections = 0;
            for (int i = 0; i < vertices.size; i++) {
                Vector2 v1 = vertices.get(i);
                Vector2 v2 = vertices.get((i + 1) % vertices.size);
                if (rayIntersectsSegment(point, v1, v2)) {
                    intersections++;
                }
            }
            return (intersections % 2) == 1;
        }

        boolean containsParticle(Particle particle) {
            return memberSet.contains(particle);
        }

        void finalizeMembers(Array<Particle> moleculeParticles) {
            memberParticles.clear();
            memberSet.clear();
            ObjectSet<Particle> boundarySet = new ObjectSet<>(boundaryParticles.size);
            for (Particle boundary : boundaryParticles) {
                memberParticles.add(boundary);
                memberSet.add(boundary);
                boundarySet.add(boundary);
            }
            for (Particle particle : moleculeParticles) {
                if (boundarySet.contains(particle)) continue;
                if (!particle.isActive() || !particle.isAlive()) continue;
                if (contains(particle.getPosition())) {
                    memberParticles.add(particle);
                    memberSet.add(particle);
                }
            }
        }

        Vector2 closestPoint(Vector2 point, Vector2 out) {
            float closestDist2 = Float.MAX_VALUE;
            Vector2 best = null;
            for (int i = 0; i < vertices.size; i++) {
                Vector2 v1 = vertices.get(i);
                Vector2 v2 = vertices.get((i + 1) % vertices.size);
                Vector2 candidate = getClosestPointOnSegment(point, v1, v2);
                float dx = point.x - candidate.x;
                float dy = point.y - candidate.y;
                float dist2 = dx * dx + dy * dy;
                if (dist2 < closestDist2) {
                    closestDist2 = dist2;
                    best = candidate;
                }
            }
            if (best == null) {
                out.set(point);
            } else {
                out.set(best);
            }
            return out;
        }

        Vector2 getCentroid(Vector2 out) {
            if (vertices.size == 0) {
                return out.set(0f, 0f);
            }
            if (Math.abs(area) < 0.0001f) {
                return out.set(vertices.first());
            }
            float cx = 0f;
            float cy = 0f;
            for (int i = 0; i < vertices.size; i++) {
                Vector2 current = vertices.get(i);
                Vector2 next = vertices.get((i + 1) % vertices.size);
                float cross = current.x * next.y - next.x * current.y;
                cx += (current.x + next.x) * cross;
                cy += (current.y + next.y) * cross;
            }
            float factor = 1f / (6f * area);
            out.set(cx * factor, cy * factor);
            return out;
        }
    }
    private final Array<MitosisEvent> activeMitosisEvents;

    public PhysicsWorld(Vector2 gravity, int initialParticleCapacity) {
        this.reactionViolenceMultiplier = 0.4f;  // Default: 40% violence (reduced from 1.0)
        this.energyDecayRate = 0.1f;  // Default: 0.1 energy/second decay
        this.bondEnergyCost = 0.5f;  // Default: 0.5 energy/second per bond
        this.foodSpawnRate = 2f;  // Default: 2 food particles per second
        this.foodSpawnAccumulator = 0f;
        this.replicationEnergyThreshold = 80f;  // Default: need 80+ energy to replicate
        this.replicationCheckInterval = 2f;  // Default: check every 2 seconds
        this.replicationCheckAccumulator = 0f;
        this.mutationRate = 0.1f;  // Default: 10% chance of mutation during replication
        this.environmentalEnergyRate = 5f;  // Default: 5 energy/sec from field
        this.chemotaxisStrength = 50f;  // Default: 50 force units for chemotaxis
        this.mitosisThreshold = 25;  // Default: molecules with 25+ particles undergo division
    this.mitosisCheckInterval = 3f;  // Default: check every 3 seconds
    this.mitosisFastCheckInterval = 1f;  // Fast-pass cadence to keep mitosis responsive
        this.mitosisCheckAccumulator = 0f;
        this.huntingRange = 15f;  // Default: look for food within 15 units
        this.huntingForce = 40f;  // Default: 40 force units toward food
        this.suctionRange = 8f;  // Default: pull food within 8 units
        this.suctionForce = 60f;  // Default: 60 force units for suction
    this.organelleEnergyRate = 8f;  // Default: 8 energy/sec per organelle
    this.organelleReplicationInterval = 30f; // Default: replicate every 30 seconds
    this.angularSpacingStrength = DEFAULT_ANGULAR_SPACING_STRENGTH;
    this.angularSpacingDamping = DEFAULT_ANGULAR_SPACING_DAMPING;
    this.angularSpacingTorqueLimit = DEFAULT_ANGULAR_SPACING_TORQUE_LIMIT;
    this.areaTargetScale = DEFAULT_AREA_TARGET_SCALE;
    this.areaPressureStrength = DEFAULT_AREA_PRESSURE_STRENGTH;
    this.world = new World(gravity, true);
        this.particlePool = new ParticlePool(initialParticleCapacity, Integer.MAX_VALUE);
        this.bondPool = new BondPool(initialParticleCapacity * 2, Integer.MAX_VALUE);
        
        // Initialize energy field (100x56 world, 5 unit cells, 0.3 strength)
        this.energyField = new EnergyField(100f, 56f, 5f, 0.3f);
        
        // Initialize moving energy fields (one of each type)
        this.movingFields = new Array<>();
        for (MovingEnergyField.FieldType type : MovingEnergyField.FieldType.values()) {
            this.movingFields.add(new MovingEnergyField(type, 100f, 56f));
        }
        
        this.activeParticles = new Array<>(false, initialParticleCapacity);
        this.activeBonds = new Array<>(false, initialParticleCapacity * 2);
        // Cell size = 2.5 units (typical particle spacing in demo)
        this.spatialGrid = new SpatialHashGrid(2.5f);
        this.unionFind = new UnionFind();
        this.bondProperties = new BondProperties();
        this.cellInteriorDetector = new CellInteriorDetector();
        this.corpsesToRemove = new Array<>();
        this.activeMitosisEvents = new Array<>();
        this.componentsNeedRebuild = false;
        this.organelleReplicationTimers = new ObjectFloatMap<>();
        this.organelleBuffer = new Array<>();
        this.organellesByCellCache = new ObjectMap<>();
        this.organelleListPool = new Array<>();
        this.anchorBondBuffer = new Array<>();
        this.anchorBoundaryBuffer = new Array<>(MAX_ANCHOR_CONNECTIONS);
        this.anchorTargetSet = new ObjectSet<>();
        this.anchorDistanceScratch = new float[MAX_ANCHOR_CONNECTIONS];
        this.spacingAdjacencyCache = new ObjectMap<>();
        this.spacingAdjacencyPool = new Array<>();
    }

    private IntMap<Array<CellStructure>> detectCellsByComponent() {
        IntMap<Array<CellStructure>> cellsByComponent = new IntMap<>();

        IntMap<Array<Particle>> moleculeGroups = new IntMap<>();
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            if (particle.isOrganelle()) continue;
            int componentId = particle.getComponentId();
            Array<Particle> group = moleculeGroups.get(componentId);
            if (group == null) {
                group = new Array<>();
                moleculeGroups.put(componentId, group);
            }
            group.add(particle);
        }

        for (IntMap.Entry<Array<Particle>> entry : moleculeGroups) {
            Array<CellStructure> cells = buildCellsForMolecule(entry.value, entry.key);
            if (cells.size > 0) {
                cellsByComponent.put(entry.key, cells);
            }
        }

        return cellsByComponent;
    }

    private Array<Particle> collectActiveOrganelles() {
        organelleBuffer.clear();
        for (Particle particle : activeParticles) {
            if (particle.isActive() && particle.isOrganelle()) {
                organelleBuffer.add(particle);
            }
        }
        return organelleBuffer;
    }

    private ObjectMap<CellStructure, Array<Particle>> buildOrganellesByCellMap(
            ObjectMap<Particle, CellStructure> organelleCells) {
        // Return pooled arrays to the scratch pool for reuse
        for (ObjectMap.Entry<CellStructure, Array<Particle>> entry : organellesByCellCache.entries()) {
            entry.value.clear();
            organelleListPool.add(entry.value);
        }
        organellesByCellCache.clear();

        if (organelleCells == null || organelleCells.size == 0) {
            return organellesByCellCache;
        }

        for (ObjectMap.Entry<Particle, CellStructure> entry : organelleCells.entries()) {
            CellStructure cell = entry.value;
            if (cell == null) {
                continue;
            }
            Array<Particle> list = organellesByCellCache.get(cell);
            if (list == null) {
                list = organelleListPool.size > 0 ? organelleListPool.pop() : new Array<>();
                list.clear();
                organellesByCellCache.put(cell, list);
            }
            list.add(entry.key);
        }

        return organellesByCellCache;
    }

    private Array<CellStructure> buildCellsForMolecule(Array<Particle> moleculeParticles, int componentId) {
        Array<CellStructure> cells = new Array<>();
        if (moleculeParticles.size < 3) {
            return cells;
        }

        ObjectSet<Particle> moleculeSet = new ObjectSet<>(moleculeParticles.size);
        for (Particle particle : moleculeParticles) {
            moleculeSet.add(particle);
        }
        ObjectMap<Particle, Array<DirectedEdge>> adjacency = new ObjectMap<>();
        Array<DirectedEdge> allEdges = new Array<>();

        for (Bond bond : activeBonds) {
            if (!bond.isActive()) continue;
            Particle a = bond.getParticleA();
            Particle b = bond.getParticleB();
            if (!moleculeSet.contains(a) || !moleculeSet.contains(b)) continue;
            if (a.isOrganelle() || b.isOrganelle()) continue;
            DirectedEdge edgeAB = addDirectedEdge(a, b, adjacency);
            DirectedEdge edgeBA = addDirectedEdge(b, a, adjacency);
            if (edgeAB != null) allEdges.add(edgeAB);
            if (edgeBA != null) allEdges.add(edgeBA);
        }

        for (ObjectMap.Entry<Particle, Array<DirectedEdge>> entry : adjacency.entries()) {
            entry.value.sort((e1, e2) -> Float.compare(e1.angle, e2.angle));
        }

        for (DirectedEdge edge : allEdges) {
            if (edge.visited) continue;
            Array<Particle> polygonParticles = new Array<>();
            DirectedEdge current = edge;
            boolean valid = true;
            while (true) {
                current.visited = true;
                polygonParticles.add(current.from);
                DirectedEdge next = getNextEdge(current, adjacency);
                if (next == null) {
                    valid = false;
                    break;
                }
                current = next;
                if (current == edge) {
                    break;
                }
            }

            if (!valid || polygonParticles.size < 3) {
                continue;
            }

            CellStructure cell = new CellStructure(componentId);
            for (Particle particle : polygonParticles) {
                cell.addVertex(particle);
            }

            cell.area = computePolygonArea(cell.vertices);
            if (Math.abs(cell.area) < 0.01f) {
                continue;
            }
            if (cell.area < 0f) {
                // Reverse winding to keep cells CCW
                cell.area = -cell.area;
                cell.vertices.reverse();
                cell.boundaryParticles.reverse();
                // Recompute bounds after reversing
                cell.minX = Float.MAX_VALUE;
                cell.maxX = -Float.MAX_VALUE;
                cell.minY = Float.MAX_VALUE;
                cell.maxY = -Float.MAX_VALUE;
                for (Vector2 vertex : cell.vertices) {
                    cell.registerBounds(vertex);
                }
            }

            cell.finalizeMembers(moleculeParticles);
            if (cell.memberParticles.size >= 3) {
                cells.add(cell);
            }
        }

        return cells;
    }

    private DirectedEdge addDirectedEdge(Particle from, Particle to, ObjectMap<Particle, Array<DirectedEdge>> adjacency) {
        Array<DirectedEdge> edges = adjacency.get(from);
        if (edges == null) {
            edges = new Array<>();
            adjacency.put(from, edges);
        }

        // Prevent duplicate edges between the same pair
        for (DirectedEdge existing : edges) {
            if (existing.to == to) {
                return null;
            }
        }

        DirectedEdge edge = new DirectedEdge(from, to);
        edges.add(edge);
        return edge;
    }

    private DirectedEdge getNextEdge(DirectedEdge current, ObjectMap<Particle, Array<DirectedEdge>> adjacency) {
        Array<DirectedEdge> edgesFromTarget = adjacency.get(current.to);
        if (edgesFromTarget == null || edgesFromTarget.size < 2) {
            return null;
        }

        int reverseIndex = -1;
        for (int i = 0; i < edgesFromTarget.size; i++) {
            if (edgesFromTarget.get(i).to == current.from) {
                reverseIndex = i;
                break;
            }
        }

        if (reverseIndex == -1) {
            return null;
        }

        int nextIndex = (reverseIndex - 1 + edgesFromTarget.size) % edgesFromTarget.size;
        return edgesFromTarget.get(nextIndex);
    }

    private float computePolygonArea(Array<Vector2> vertices) {
        if (vertices.size < 3) return 0f;
        float area = 0f;
        for (int i = 0; i < vertices.size; i++) {
            Vector2 current = vertices.get(i);
            Vector2 next = vertices.get((i + 1) % vertices.size);
            area += current.x * next.y - next.x * current.y;
        }
        return area * 0.5f;
    }

    private ObjectMap<Particle, CellStructure> mapOrganellesToCells(Array<Particle> organelles,
                                                                    IntMap<Array<CellStructure>> cellsByComponent) {
        ObjectMap<Particle, CellStructure> mapping = new ObjectMap<>();
        if (cellsByComponent == null || organelles.size == 0) return mapping;

        for (Particle organelle : organelles) {
            Vector2 pos = organelle.getPosition();
            for (IntMap.Entry<Array<CellStructure>> entry : cellsByComponent) {
                Array<CellStructure> cells = entry.value;
                for (CellStructure cell : cells) {
                    if (cell.containsParticle(organelle) || cell.contains(pos)) {
                        mapping.put(organelle, cell);
                        break;
                    }
                }
                if (mapping.containsKey(organelle)) {
                    break;
                }
            }
        }

        return mapping;
    }
    
    public BondProperties getBondProperties() {
        return bondProperties;
    }
    
    public void setReactionViolenceMultiplier(float multiplier) {
        this.reactionViolenceMultiplier = Math.max(0f, multiplier);
    }
    
    public float getReactionViolenceMultiplier() {
        return reactionViolenceMultiplier;
    }
    
    public void setEnergyDecayRate(float rate) {
        this.energyDecayRate = Math.max(0f, rate);
    }
    
    public float getEnergyDecayRate() {
        return energyDecayRate;
    }
    
    public void setBondEnergyCost(float cost) {
        this.bondEnergyCost = Math.max(0f, cost);
    }
    
    public float getBondEnergyCost() {
        return bondEnergyCost;
    }
    
    public void setFoodSpawnRate(float rate) {
        this.foodSpawnRate = Math.max(0f, rate);
    }
    
    public float getFoodSpawnRate() {
        return foodSpawnRate;
    }
    
    public void setReplicationEnergyThreshold(float threshold) {
        this.replicationEnergyThreshold = Math.max(0f, Math.min(100f, threshold));
    }
    
    public float getReplicationEnergyThreshold() {
        return replicationEnergyThreshold;
    }
    
    public void setMutationRate(float rate) {
        this.mutationRate = Math.max(0f, Math.min(1f, rate));
    }
    
    public float getMutationRate() {
        return mutationRate;
    }
    
    public EnergyField getEnergyField() {
        return energyField;
    }
    
    public void setEnvironmentalEnergyRate(float rate) {
        this.environmentalEnergyRate = Math.max(0f, rate);
    }
    
    public float getEnvironmentalEnergyRate() {
        return environmentalEnergyRate;
    }
    
    public void setChemotaxisStrength(float strength) {
        this.chemotaxisStrength = Math.max(0f, strength);
    }
    
    public float getChemotaxisStrength() {
        return chemotaxisStrength;
    }
    
    public void setMitosisThreshold(int threshold) {
        this.mitosisThreshold = Math.max(3, threshold); // Minimum 3 for valid molecule
    }
    
    public int getMitosisThreshold() {
        return mitosisThreshold;
    }
    
    public void setHuntingRange(float range) {
        this.huntingRange = Math.max(0f, range);
    }
    
    public float getHuntingRange() {
        return huntingRange;
    }
    
    public void setHuntingForce(float force) {
        this.huntingForce = Math.max(0f, force);
    }
    
    public float getHuntingForce() {
        return huntingForce;
    }
    
    public void setSuctionRange(float range) {
        this.suctionRange = Math.max(0f, range);
    }
    
    public float getSuctionRange() {
        return suctionRange;
    }
    
    public void setSuctionForce(float force) {
        this.suctionForce = Math.max(0f, force);
    }
    
    public float getSuctionForce() {
        return suctionForce;
    }
    
    public void setOrganelleEnergyRate(float rate) {
        this.organelleEnergyRate = Math.max(0f, rate);
    }
    
    public float getOrganelleEnergyRate() {
        return organelleEnergyRate;
    }

    public void setOrganelleReplicationInterval(float intervalSeconds) {
        this.organelleReplicationInterval = Math.max(0f, intervalSeconds);
    }

    public float getOrganelleReplicationInterval() {
        return organelleReplicationInterval;
    }

    public void setAngularSpacingStrength(float strength) {
        this.angularSpacingStrength = Math.max(0f, strength);
    }

    public float getAngularSpacingStrength() {
        return angularSpacingStrength;
    }

    public void setAngularSpacingDamping(float damping) {
        this.angularSpacingDamping = Math.max(0f, damping);
    }

    public float getAngularSpacingDamping() {
        return angularSpacingDamping;
    }

    public void setAngularSpacingTorqueLimit(float torqueLimit) {
        this.angularSpacingTorqueLimit = Math.max(0f, torqueLimit);
    }

    public float getAngularSpacingTorqueLimit() {
        return angularSpacingTorqueLimit;
    }

    public void setAreaTargetScale(float scale) {
        this.areaTargetScale = Math.max(0.1f, scale);
    }

    public float getAreaTargetScale() {
        return areaTargetScale;
    }

    public void setAreaPressureStrength(float strength) {
        this.areaPressureStrength = Math.max(0f, strength);
    }

    public float getAreaPressureStrength() {
        return areaPressureStrength;
    }
    
    public Array<MovingEnergyField> getMovingFields() {
        return movingFields;
    }
    
    public void setWorldSize(float worldWidth, float worldHeight) {
        for (MovingEnergyField field : movingFields) {
            field.setWorldSize(worldWidth, worldHeight);
        }
    }

    public World getWorld() {
        return world;
    }

    public Array<Particle> getActiveParticles() {
        return activeParticles;
    }

    public Array<Bond> getActiveBonds() {
        return activeBonds;
    }

    public SpatialHashGrid getSpatialGrid() {
        return spatialGrid;
    }

    public UnionFind getUnionFind() {
        return unionFind;
    }

    /**
     * Query particles within a radius of a position using spatial hashing.
     * Efficient O(k) operation for finding nearby particles.
     */
    public Array<Particle> queryParticlesInRadius(float x, float y, float radius) {
        return spatialGrid.queryRadius(x, y, radius);
    }

    /**
     * Query neighbors of a particle within a given radius (excluding the particle itself).
     */
    public Array<Particle> queryNeighbors(Particle particle, float radius) {
        return spatialGrid.queryNeighbors(particle, radius);
    }

    public Array<Particle> queryRadius(float x, float y, float radius) {
        return spatialGrid.queryRadius(x, y, radius);
    }

    /**
     * Check if two particles are in the same connected component.
     */
    public boolean areConnected(Particle a, Particle b) {
        return unionFind.connected(a.hashCode(), b.hashCode());
    }

    /**
     * Get the size of the connected component containing this particle.
     */
    public int getComponentSize(Particle particle) {
        return unionFind.getComponentSize(particle.hashCode());
    }

    public Particle spawnParticle(ParticleType type, Vector2 position, float visualRadius, float mass) {
        float collisionRadius = visualRadius * 0.6f;
        CircleShape shape = new CircleShape();
        shape.setRadius(collisionRadius);

        BodyDef bodyDef = new BodyDef();
        bodyDef.type = BodyDef.BodyType.DynamicBody;
        bodyDef.position.set(position);
        bodyDef.linearDamping = DEFAULT_LINEAR_DAMPING;
        bodyDef.angularDamping = DEFAULT_ANGULAR_DAMPING;

        Body body = world.createBody(bodyDef);

        FixtureDef fixtureDef = new FixtureDef();
        fixtureDef.shape = shape;
        float area = MathUtils.PI * collisionRadius * collisionRadius;
        float density = area > MathUtils.FLOAT_ROUNDING_ERROR ? mass / area : DEFAULT_PARTICLE_DENSITY;
        fixtureDef.density = density;
        fixtureDef.friction = DEFAULT_PARTICLE_FRICTION;
        fixtureDef.restitution = DEFAULT_PARTICLE_RESTITUTION;

        body.createFixture(fixtureDef);
        shape.dispose();

        Particle particle = particlePool.obtain();
        // Initialize with random energy between 50-100
        float initialEnergy = 50f + MathUtils.random(50f);
        particle.init(body, type, collisionRadius, visualRadius, initialEnergy);
        activeParticles.add(particle);
        
        // Add to union-find as its own component
        unionFind.addParticle(particle.hashCode());
        particle.setComponentId(particle.hashCode());
        
        return particle;
    }

    public void destroyParticle(Particle particle) {
        if (particle == null || !particle.isActive()) {
            return;
        }
        
        // Remove from union-find
        unionFind.removeParticle(particle.hashCode());
        
        particle.deactivate(world);
        activeParticles.removeValue(particle, true);
        particlePool.free(particle);
        componentsNeedRebuild = true;
        organelleReplicationTimers.remove(particle, 0f);
    }

    public Bond createBond(Particle a, Particle b, BondType type, float stiffness, 
                           float damping, float breakForceThreshold) {
        if (a == null || b == null || !a.isActive() || !b.isActive()) {
            return null;
        }

        if (a.isOrganelle() || b.isOrganelle()) {
            return null; // Organelles never form structural bonds
        }
        
        // Get multipliers based on particle types
        float[] multipliers = bondProperties.getMultipliers(a.getType(), b.getType());
        float lengthMult = multipliers[0];
        float stiffnessMult = multipliers[1];
        float breakMult = multipliers[2];
        
        Vector2 posA = a.getPosition();
        Vector2 posB = b.getPosition();
        float baseRestLength = posA.dst(posB);
        
        // Apply multipliers and add 10% extra length for spacing
        float restLength = baseRestLength * lengthMult * 1.1f;
        float finalStiffness = stiffness * stiffnessMult;
        float finalBreakForce = breakForceThreshold * breakMult;
        
        Bond bond = bondPool.obtain();
        bond.init(a, b, type, restLength, finalStiffness, damping, finalBreakForce);
        activeBonds.add(bond);
        
        // Union components when bond forms
        if (unionFind.union(a.hashCode(), b.hashCode())) {
            // Update component IDs
            int newComponentId = unionFind.find(a.hashCode());
            a.setComponentId(newComponentId);
            b.setComponentId(newComponentId);
        }
        
        return bond;
    }

    public void destroyBond(Bond bond) {
        destroyBond(bond, true, false);
    }
    
    /**
     * Destroy a bond, optionally applying a violent reaction force.
     * The strength of the reaction is proportional to the bond's break force threshold.
     */
    public void destroyBond(Bond bond, boolean applyViolentReaction) {
        destroyBond(bond, applyViolentReaction, false);
    }

    public void destroyBond(Bond bond, boolean applyViolentReaction, boolean forceDestroy) {
        if (bond == null || !bond.isActive()) {
            return;
        }

        // Skip destruction while mitosis protection is active unless forced
        if (!forceDestroy && isBondProtectedByMitosis(bond)) {
            return;
        }
        
        // Apply violent reaction before deactivating
        if (applyViolentReaction && bond.getBondType() != BondType.ANCHOR) {
            applyBondBreakReaction(bond);
        }
        
        bond.deactivate();
        activeBonds.removeValue(bond, true);
        bondPool.free(bond);
        componentsNeedRebuild = true;
        
        // Note: Union-Find doesn't support efficient split operations.
        // For accurate component tracking after many bond breaks, call rebuildUnionFind()
    }
    
    /**
     * Apply violent reaction force when a bond breaks.
     * Stronger bonds create more violent reactions (stronger repulsion).
     * Force is modulated by the reactionViolenceMultiplier slider.
     */
    private void applyBondBreakReaction(Bond bond) {
        Particle a = bond.getParticleA();
        Particle b = bond.getParticleB();
        
        // Reaction force scales with bond strength (break force threshold)
        // Base: 200, Range: 60-600, Scale: 2-12x multiplier
        float breakForce = bond.getBreakForceThreshold();
        float reactionMultiplier = breakForce / 50f;  // 60->1.2x, 200->4x, 600->12x
        float baseRepulsion = 300f;
        float repulsionStrength = baseRepulsion * reactionMultiplier * reactionViolenceMultiplier;
        
        // Calculate direction from A to B
        Vector2 posA = a.getPosition();
        Vector2 posB = b.getPosition();
        Vector2 direction = new Vector2(posB).sub(posA);
        float distance = direction.len();
        
        if (distance > 0.001f) {
            direction.nor();
            
            // Apply explosive force pushing particles apart
            Vector2 forceA = new Vector2(direction).scl(-repulsionStrength);
            Vector2 forceB = new Vector2(direction).scl(repulsionStrength);
            
            a.getBody().applyForceToCenter(forceA, true);
            b.getBody().applyForceToCenter(forceB, true);
        }
    }

    public void step(float delta, int velocityIterations, int positionIterations) {
        // Update moving energy fields
        for (MovingEnergyField field : movingFields) {
            field.update(delta);
        }
        
        // Spawn food particles at configured rate
        spawnFoodParticles(delta);
        
    // Process energy absorption from food particles
    processEnergyAbsorption(delta);
        
    // Apply energy decay and bond costs
    applyEnergyMechanics(delta);
        
        // Apply chemotaxis forces (gradient following)
        applyChemotaxis();
        
        // Apply hunting behavior (weak molecules seek food)
        applyHuntingBehavior();
        
        // Apply food suction (molecules pull nearby food toward them)
        applyFoodSuction();
        
        // Apply crowding pressure (repulsion in dense regions)
        applyCrowdingPressure();
        
        // Apply intramolecular spacing (prevent clumping within molecules)
        applyIntramolecularSpacing();
        
        // Age particles and bonds
        updateAging(delta);
        
        // Check for replication opportunities
        processReplication(delta);
        
        // Check for oversized molecules and trigger mitosis
        processMitosis(delta);
        
        // Remove particles that ran out of energy
        removeDeadParticles();
        
        // Eject or consume dead particles from living molecules
        processDeadParticlesInMolecules();
        
        // Process corpse consumption by molecular structures
        processCellInteriorConsumption();
        
    // Detect closed cells once per step for organelle logic
    IntMap<Array<CellStructure>> cellsByComponent = detectCellsByComponent();

    // Process organelle absorption (GREEN particles become trapped energy generators)
    processOrganelleAbsorption(cellsByComponent);

    Array<Particle> organelles = collectActiveOrganelles();
    ObjectMap<Particle, CellStructure> organelleCells =
        (organelles.size == 0) ? null : mapOrganellesToCells(organelles, cellsByComponent);
    ObjectMap<CellStructure, Array<Particle>> organellesByCell =
        buildOrganellesByCellMap(organelleCells);

    // Process organelle containment, boundary pushing, and inter-organelle bonding
    processOrganelleContainment(cellsByComponent, organelles, organelleCells);

    // Break organelle bonds when cells separate
    pruneOrganelleBonds(cellsByComponent, organelleCells);

    // Handle organelle self-replication timer and spawning
    processOrganelleReplication(delta, cellsByComponent, organelles, organelleCells);

    // Apply organelle energy generation per detected cell
    applyOrganelleEnergyGeneration(delta, cellsByComponent, organelleCells, organellesByCell);
    maintainOrganelleAnchorBonds(delta, cellsByComponent, organelleCells, organellesByCell);
    applyBoundaryAngularSpacing(cellsByComponent);
    applyCellAreaPressure(cellsByComponent);
        
        // Check for crossing bonds within same molecule and break them
        detectAndBreakCrossingBonds();
        
        // Apply rotational shear forces to break peripheral bonds
        applyRotationalShear();
        
        // Apply bond constraints before Box2D step (with organelle strengthening)
        for (int i = activeBonds.size - 1; i >= 0; i--) {
            Bond bond = activeBonds.get(i);
            if (bond.applyConstraint(activeParticles)) {
                destroyBond(bond);
            }
        }
        
        world.step(delta, velocityIterations, positionIterations);
        
        // Update spatial grid after physics step
        spatialGrid.update(activeParticles);
    }
    
    /**
     * Process metabolic byproducts from active bonds.
     * Certain bond combinations create energy-rich particles as byproducts.
     * This creates a metabolism-like system where bonds produce consumable energy.
     */
    private void spawnFoodParticles(float delta) {
        if (foodSpawnRate <= 0) return;
        
        foodSpawnAccumulator += delta;
        float spawnInterval = 1f / foodSpawnRate;  // Seconds between spawns
        
        while (foodSpawnAccumulator >= spawnInterval) {
            foodSpawnAccumulator -= spawnInterval;
            
            // Find productive bonds (bonds that generate energy byproducts)
            Array<Bond> productiveBonds = new Array<>();
            for (Bond bond : activeBonds) {
                if (!bond.isActive()) continue;
                if (bond.getBondType() == BondType.ANCHOR) continue;
                if (isProductiveBond(bond)) {
                    productiveBonds.add(bond);
                }
            }
            
            if (productiveBonds.size == 0) continue;
            
            // Pick a random productive bond
            Bond selectedBond = productiveBonds.get(MathUtils.random(productiveBonds.size - 1));
            
            // Spawn energy-rich particle near the bond midpoint
            Particle a = selectedBond.getParticleA();
            Particle b = selectedBond.getParticleB();
            Vector2 midpoint = new Vector2(a.getPosition()).add(b.getPosition()).scl(0.5f);
            
            // Add some random offset
            float offsetX = MathUtils.random(-2f, 2f);
            float offsetY = MathUtils.random(-2f, 2f);
            midpoint.add(offsetX, offsetY);
            
            // Determine particle type based on bond types
            ParticleType byproductType = getByproductType(a.getType(), b.getType());
            
            // Create energy-rich particle
            Particle byproduct = spawnParticle(byproductType, midpoint, 0.5f, 1f);
            byproduct.setEnergy(60f);  // Rich in energy
            byproduct.setEnergyRich(true);  // Mark as consumable
            
            // Give it velocity away from bond (ejection)
            Vector2 direction = new Vector2(midpoint).sub(a.getPosition()).nor();
            float vx = direction.x * MathUtils.random(8f, 15f) + MathUtils.random(-3f, 3f);
            float vy = direction.y * MathUtils.random(8f, 15f) + MathUtils.random(-3f, 3f);
            byproduct.getBody().setLinearVelocity(vx, vy);
        }
    }
    
    /**
     * Determine if a bond produces energy byproducts.
     * Productive bonds: different types, especially logic states + gates.
     */
    private boolean isProductiveBond(Bond bond) {
        ParticleType typeA = bond.getParticleA().getType();
        ParticleType typeB = bond.getParticleB().getType();
        
        // Same type bonds don't produce byproducts
        if (typeA == typeB) return false;
        
        // Logic states bonded to gates are productive
        if (typeA.isLogicState() && typeB.isGate()) return true;
        if (typeB.isLogicState() && typeA.isGate()) return true;
        
        // Different logic states are productive
        if (typeA.isLogicState() && typeB.isLogicState()) return true;
        
        // Different gates together are moderately productive
        if (typeA.isGate() && typeB.isGate()) {
            return MathUtils.randomBoolean(0.3f); // 30% chance
        }
        
        return false;
    }
    
    /**
     * Determine the particle type of the metabolic byproduct based on parent types.
     * Creates interesting combinations - e.g., TRUE+AND -> FALSE, NOT+OR -> TRUE
     */
    private ParticleType getByproductType(ParticleType typeA, ParticleType typeB) {
        // TRUE + FALSE -> Random gate
        if ((typeA == ParticleType.TRUE && typeB == ParticleType.FALSE) ||
            (typeA == ParticleType.FALSE && typeB == ParticleType.TRUE)) {
            ParticleType[] gates = {ParticleType.AND, ParticleType.OR, ParticleType.XOR};
            return gates[MathUtils.random(gates.length - 1)];
        }
        
        // TRUE + Gate -> FALSE (consumption/inhibition)
        if (typeA == ParticleType.TRUE && typeB.isGate()) return ParticleType.FALSE;
        if (typeB == ParticleType.TRUE && typeA.isGate()) return ParticleType.FALSE;
        
        // FALSE + Gate -> TRUE (activation)
        if (typeA == ParticleType.FALSE && typeB.isGate()) return ParticleType.TRUE;
        if (typeB == ParticleType.FALSE && typeA.isGate()) return ParticleType.TRUE;
        
        // Gate + Gate -> Logic state (random)
        if (typeA.isGate() && typeB.isGate()) {
            return MathUtils.randomBoolean() ? ParticleType.TRUE : ParticleType.FALSE;
        }
        
        // Default: opposite of first type if logic state
        if (typeA.isLogicState()) {
            return typeA == ParticleType.TRUE ? ParticleType.FALSE : ParticleType.TRUE;
        }
        
        return ParticleType.TRUE;
    }
    
    /**
     * Process energy absorption when particles contact energy-rich particles.
     * Energy-rich particles transfer energy and are consumed when depleted.
     */
    private void processEnergyAbsorption(float delta) {
        float absorptionRadius = 1.5f;  // Distance for energy transfer
        float transferRate = 30f;  // Energy units per second
        
        Array<Particle> foodToRemove = new Array<>();
        
        for (Particle food : activeParticles) {
            if (!food.isActive() || !food.isEnergyRich()) continue;
            if (food.getEnergy() <= 0) {
                foodToRemove.add(food);
                continue;
            }
            
            // Find nearby particles that can absorb energy
            Array<Particle> nearbyParticles = spatialGrid.queryRadius(
                food.getPosition().x, food.getPosition().y, absorptionRadius);
            
            for (Particle consumer : nearbyParticles) {
                if (!consumer.isActive() || consumer == food) continue;
                if (consumer.isEnergyRich()) continue; // Energy-rich particles don't consume each other
                
                // Check if close enough for energy transfer
                float distSq = food.getPosition().dst2(consumer.getPosition());
                if (distSq < absorptionRadius * absorptionRadius) {
                    // Transfer energy from food to consumer
                    float transferAmount = transferRate * delta;
                    float actualTransfer = Math.min(transferAmount, food.getEnergy());
                    
                    food.consumeEnergy(actualTransfer);
                    consumer.addEnergy(actualTransfer);
                    
                    // Mark as depleted (will be cleaned up by removeDeadParticles)
                    if (food.getEnergy() <= 0) {
                        break;
                    }
                }
            }
        }
        
        // Note: Depleted food particles (energy = 0) are removed by removeDeadParticles()
    }
    
    /**
     * Apply energy decay and bond energy costs to all particles.
     */
    private void applyEnergyMechanics(float delta) {
        // Count bonds per particle
        int[] bondCount = new int[activeParticles.size];
        for (Bond bond : activeBonds) {
            if (!bond.isActive()) continue;
            if (bond.getBondType() == BondType.ANCHOR) continue;
            
            for (int i = 0; i < activeParticles.size; i++) {
                Particle p = activeParticles.get(i);
                if (p == bond.getParticleA() || p == bond.getParticleB()) {
                    bondCount[i]++;
                }
            }
        }
        
        // Apply energy decay, bond costs, and environmental absorption
        for (int i = 0; i < activeParticles.size; i++) {
            Particle particle = activeParticles.get(i);
            if (!particle.isActive()) continue;
            if (particle.isOrganelle()) continue;
            
            // Base energy decay
            float decay = energyDecayRate * delta;
            
            // Metabolic scaling: larger molecules have higher energy costs
            // Get molecule size for this particle
            int moleculeSize = getComponentSize(particle);
            
            // Scaling factor: starts at 1.0 for small molecules, increases for larger ones
            // Formula: 1.0 + (size - 10) * 0.05, capped at 3x cost
            // Examples: size 10 = 1.0x, size 20 = 1.5x, size 30 = 2.0x, size 40+ = 3.0x
            float scalingFactor = Math.min(3.0f, 1.0f + Math.max(0f, (moleculeSize - 10) * 0.05f));
            
            // Additional cost per bond with metabolic scaling
            float bondCost = bondEnergyCost * bondCount[i] * scalingFactor * delta;
            
            // Environmental energy absorption from moving fields
            float environmentalGain = 0f;
            Vector2 particlePos = particle.getPosition();
            ParticleType particleType = particle.getType();
            
            for (MovingEnergyField field : movingFields) {
                if (!field.isActive()) continue;
                
                // Get field energy at particle position
                float fieldEnergy = field.getEnergyAt(particlePos.x, particlePos.y);
                
                // Get type-specific affinity (only positive affinity provides energy)
                float affinity = field.getType().getAffinityFor(particleType);
                if (affinity > 0f) {
                    // Energy field disruption: large molecules absorb less efficiently
                    // Penalty formula: 1.0 / (1.0 + size * 0.02)
                    // Examples: size 10 = 0.83x, size 20 = 0.71x, size 30 = 0.63x
                    float absorptionPenalty = 1.0f / (1.0f + moleculeSize * 0.02f);
                    environmentalGain += fieldEnergy * affinity * environmentalEnergyRate * absorptionPenalty * delta;
                }
            }
            
            // Net energy change
            particle.consumeEnergy(decay + bondCost);
            particle.addEnergy(environmentalGain);
        }
        
    }
    
    /**
     * Organelles generate energy for nearby particles in the same molecule.
     * Acts like photosynthesis - passive energy production that scales with organelle count.
     */
    private void applyOrganelleEnergyGeneration(float delta,
                                                IntMap<Array<CellStructure>> cellsByComponent,
                                                ObjectMap<Particle, CellStructure> organelleCells,
                                                ObjectMap<CellStructure, Array<Particle>> organellesByCell) {
        if (organelleEnergyRate <= 0f || cellsByComponent == null || cellsByComponent.size == 0) return;
        if (organelleCells == null || organelleCells.size == 0) return;
        if (organellesByCell == null || organellesByCell.size == 0) return;

        for (ObjectMap.Entry<CellStructure, Array<Particle>> entry : organellesByCell.entries()) {
            CellStructure cell = entry.key;
            Array<Particle> cellOrganelles = entry.value;
            int organelleCount = cellOrganelles.size;
            if (cell.memberParticles.size == 0 || organelleCount == 0) continue;

            float count = organelleCount;
            float generationMultiplier = count * (1.0f - 0.05f * Math.min(count, 10f));
            float totalEnergyGeneration = organelleEnergyRate * generationMultiplier * delta;
            float energyPerParticle = totalEnergyGeneration / cell.memberParticles.size;

            for (Particle member : cell.memberParticles) {
                if (!member.isActive() || !member.isAlive()) continue;
                member.addEnergy(energyPerParticle);
            }

            reinforceCellBoundaryBonds(cell, organelleCount, delta);
            adjustCellBondStretch(cell, organelleCount, delta);
            applyOrganelleInflationForces(cell, cellOrganelles, delta);
        }
    }

    private void reinforceCellBoundaryBonds(CellStructure cell, int organelleCount, float delta) {
        if (cell == null || organelleCount <= 0) {
            return;
        }
        if (organelleEnergyRate <= 0f || cell.boundaryParticles.size < 3) {
            return;
        }

        float energyRatio = MAX_ORGANELLE_ENERGY_RATE <= 0f ? 0f :
            Math.min(1f, organelleEnergyRate / MAX_ORGANELLE_ENERGY_RATE);
        if (energyRatio <= 0f) {
            return;
        }

        float reinforcementFactor = organelleCount * energyRatio;
        float breakForceGain = reinforcementFactor * BOND_RESTORATION_RATE * delta;
        float ageReduction = reinforcementFactor * BOND_REJUVENATION_RATE * delta;

        for (int i = 0; i < cell.boundaryParticles.size; i++) {
            Particle a = cell.boundaryParticles.get(i);
            Particle b = cell.boundaryParticles.get((i + 1) % cell.boundaryParticles.size);
            Bond bond = findActiveBondBetween(a, b);
            if (bond == null) {
                continue;
            }
            bond.increaseBreakForceThreshold(breakForceGain);
            bond.rejuvenate(ageReduction);
        }
    }

    private void adjustCellBondStretch(CellStructure cell, int organelleCount, float delta) {
        if (cell == null || cell.boundaryParticles.size < 2) {
            return;
        }

        final float STRETCH_PER_ORGANELLE = 0.04f;
        final float MAX_STRETCH = 0.6f;
        final float REST_ADJUST_RATE = 2.5f;
        final float BREAK_STRETCH_RATIO = 0.6f;

        float stretchMultiplier = 1f + Math.min(MAX_STRETCH, organelleCount * STRETCH_PER_ORGANELLE);

        for (int i = 0; i < cell.boundaryParticles.size; i++) {
            Particle a = cell.boundaryParticles.get(i);
            Particle b = cell.boundaryParticles.get((i + 1) % cell.boundaryParticles.size);
            Bond bond = findActiveBondBetween(a, b);
            if (bond == null) {
                continue;
            }

            float initialRest = bond.getInitialRestLength() > 0f ? bond.getInitialRestLength() : bond.getRestLength();
            float targetRest = initialRest * stretchMultiplier;
            float currentRest = bond.getRestLength();
            float restDelta = targetRest - currentRest;
            if (Math.abs(restDelta) > 0.0001f) {
                float maxStep = initialRest * REST_ADJUST_RATE * delta;
                restDelta = MathUtils.clamp(restDelta, -maxStep, maxStep);
                bond.setRestLength(currentRest + restDelta);
            }

            float initialBreak = bond.getInitialBreakForceThreshold();
            if (initialBreak <= 0f) {
                continue;
            }
            float targetBreak = initialBreak * (1f + Math.min(MAX_STRETCH, organelleCount * STRETCH_PER_ORGANELLE * BREAK_STRETCH_RATIO));
            float currentBreak = bond.getBreakForceThreshold();
            float breakDelta = targetBreak - currentBreak;
            if (Math.abs(breakDelta) > 0.01f) {
                float maxBreakStep = initialBreak * 3f * delta;
                breakDelta = MathUtils.clamp(breakDelta, -maxBreakStep, maxBreakStep);
                bond.setBreakForceThreshold(currentBreak + breakDelta);
            }
        }
    }

    private void applyOrganelleInflationForces(CellStructure cell, Array<Particle> organelles, float delta) {
        if (cell == null || organelles == null || organelles.size == 0) {
            return;
        }
        if (cell.boundaryParticles.size < 2) {
            return;
        }

        final float PUSH_FORCE = 180f;
        final float MAX_EFFECT_DISTANCE = 3.2f;
        ClosestBoundaryInfo boundaryInfo = new ClosestBoundaryInfo();

        for (Particle organelle : organelles) {
            if (organelle == null || !organelle.isActive()) {
                continue;
            }

            Vector2 orgPos = organelle.getPosition();
            boundaryInfo = findClosestBoundaryInfo(cell, orgPos, boundaryInfo);
            if (boundaryInfo.distanceSq == Float.MAX_VALUE) {
                continue;
            }

            float dist = (float) Math.sqrt(boundaryInfo.distanceSq);
            if (dist > MAX_EFFECT_DISTANCE) {
                continue;
            }

            Vector2 pushDir = new Vector2(boundaryInfo.closestPoint).sub(orgPos);
            float len = pushDir.len();
            if (len < 0.0001f) {
                continue;
            }
            pushDir.scl(1f / len);

            float falloff = 1f - MathUtils.clamp(dist / MAX_EFFECT_DISTANCE, 0f, 1f);
            float forceMag = PUSH_FORCE * falloff * delta;
            float fx = pushDir.x * forceMag;
            float fy = pushDir.y * forceMag;

            if (boundaryInfo.particleA != null) {
                boundaryInfo.particleA.getBody().applyForceToCenter(fx * 0.5f, fy * 0.5f, true);
            }
            if (boundaryInfo.particleB != null) {
                boundaryInfo.particleB.getBody().applyForceToCenter(fx * 0.5f, fy * 0.5f, true);
            }

            organelle.getBody().applyForceToCenter(-fx, -fy, true);
        }
    }

    private void applyBoundaryAngularSpacing(IntMap<Array<CellStructure>> cellsByComponent) {
        if (cellsByComponent == null || cellsByComponent.size == 0) {
            return;
        }

    Vector2 prevPos = new Vector2();
    Vector2 nextPos = new Vector2();
    Vector2 dir = new Vector2();
    Vector2 relVel = new Vector2();
    Vector2 centroid = new Vector2();
    Vector2 centerPush = new Vector2();

        for (IntMap.Entry<Array<CellStructure>> entry : cellsByComponent) {
            for (CellStructure cell : entry.value) {
                int boundaryCount = cell.boundaryParticles.size;
                if (boundaryCount < 3) {
                    continue;
                }
                cell.getCentroid(centroid);

                for (int i = 0; i < boundaryCount; i++) {
                    Particle curr = cell.boundaryParticles.get(i);
                    Particle prev = cell.boundaryParticles.get((i - 1 + boundaryCount) % boundaryCount);
                    Particle next = cell.boundaryParticles.get((i + 1) % boundaryCount);
                    if (!curr.isActive() || !curr.isAlive() || !prev.isActive() || !next.isActive()) {
                        continue;
                    }

                    prevPos.set(prev.getPosition()).sub(curr.getPosition());
                    nextPos.set(next.getPosition()).sub(curr.getPosition());
                    float lenPrev = prevPos.len();
                    float lenNext = nextPos.len();
                    if (lenPrev < 0.15f || lenNext < 0.15f) {
                        continue;
                    }
                    prevPos.scl(1f / lenPrev);
                    nextPos.scl(1f / lenNext);

                    float cosAngle = prevPos.dot(nextPos);
                    if (cosAngle <= BOUNDARY_TARGET_COS) {
                        continue; // already wide enough
                    }

                    float angleFactor = MathUtils.clamp((cosAngle - BOUNDARY_TARGET_COS) / (1f - BOUNDARY_TARGET_COS), 0f, 1f);
                    dir.set(next.getPosition()).sub(prev.getPosition());
                    float chordLength = dir.len();
                    if (chordLength < 0.25f) {
                        continue;
                    }
                    dir.scl(1f / chordLength);

                    float forceMag = angularSpacingStrength * angleFactor;
                    relVel.set(next.getBody().getLinearVelocity()).sub(prev.getBody().getLinearVelocity());
                    float damping = angularSpacingDamping * relVel.dot(dir);
                    float finalForce = Math.max(0f, forceMag - damping);
                    finalForce = Math.min(finalForce, angularSpacingTorqueLimit);

                    float forceX = dir.x * finalForce;
                    float forceY = dir.y * finalForce;
                    prev.getBody().applyForceToCenter(-forceX, -forceY, true);
                    next.getBody().applyForceToCenter(forceX, forceY, true);

                    centerPush.set(curr.getPosition()).sub(centroid);
                    float centerLen = centerPush.len();
                    if (centerLen > 0.0001f) {
                        centerPush.scl(1f / centerLen);
                        curr.getBody().applyForceToCenter(centerPush.x * finalForce * 0.5f, centerPush.y * finalForce * 0.5f, true);
                    }
                }
            }
        }
    }

    private void applyCellAreaPressure(IntMap<Array<CellStructure>> cellsByComponent) {
        if (cellsByComponent == null || cellsByComponent.size == 0) {
            return;
        }

        Vector2 centroid = new Vector2();
        Vector2 direction = new Vector2();

        for (IntMap.Entry<Array<CellStructure>> entry : cellsByComponent) {
            for (CellStructure cell : entry.value) {
                int boundaryCount = cell.boundaryParticles.size;
                if (boundaryCount < 3) {
                    continue;
                }

                cell.getCentroid(centroid);
                float totalRadius = 0f;
                int activeCount = 0;
                for (Particle boundary : cell.boundaryParticles) {
                    if (!boundary.isActive() || !boundary.isAlive()) {
                        continue;
                    }
                    totalRadius += centroid.dst(boundary.getPosition());
                    activeCount++;
                }
                if (activeCount == 0) {
                    continue;
                }

                float avgRadius = totalRadius / activeCount;
                if (avgRadius < 0.001f) {
                    continue;
                }
                float targetRadius = avgRadius * areaTargetScale;

                for (Particle boundary : cell.boundaryParticles) {
                    if (!boundary.isActive() || !boundary.isAlive()) {
                        continue;
                    }
                    direction.set(boundary.getPosition()).sub(centroid);
                    float dist = direction.len();
                    if (dist < 0.0001f) {
                        continue;
                    }
                    if (dist >= targetRadius) {
                        continue;
                    }
                    float deficit = targetRadius - dist;
                    float forceMag = areaPressureStrength * (deficit / targetRadius);
                    direction.scl(forceMag / dist);
                    boundary.getBody().applyForceToCenter(direction.x, direction.y, true);
                }
            }
        }
    }

    private void maintainOrganelleAnchorBonds(float delta,
                                              IntMap<Array<CellStructure>> cellsByComponent,
                                              ObjectMap<Particle, CellStructure> organelleCells,
                                              ObjectMap<CellStructure, Array<Particle>> organellesByCell) {
        if (cellsByComponent == null || cellsByComponent.size == 0) {
            removeAllAnchorBonds();
            return;
        }
        if (organelleCells == null || organelleCells.size == 0 || organellesByCell == null || organellesByCell.size == 0) {
            removeAllAnchorBonds();
            return;
        }

        for (ObjectMap.Entry<CellStructure, Array<Particle>> entry : organellesByCell.entries()) {
            CellStructure cell = entry.key;
            Array<Particle> cellOrganelles = entry.value;
            if (cell == null || cellOrganelles == null || cellOrganelles.size == 0) {
                continue;
            }
            maintainAnchorsForCell(cell, cellOrganelles, delta);
        }
    }

    private void maintainAnchorsForCell(CellStructure cell, Array<Particle> organelles, float delta) {
        if (cell.boundaryParticles.size < 3) {
            for (Particle organelle : organelles) {
                removeAllAnchorBonds(organelle);
            }
            return;
        }

        for (Particle organelle : organelles) {
            if (organelle == null || !organelle.isActive()) {
                removeAllAnchorBonds(organelle);
                continue;
            }
            selectClosestBoundaryParticles(cell, organelle, anchorBoundaryBuffer);
            ensureAnchorsForOrganelle(organelle, anchorBoundaryBuffer, delta);
        }
    }

    private void selectClosestBoundaryParticles(CellStructure cell, Particle organelle, Array<Particle> out) {
        out.clear();
        Arrays.fill(anchorDistanceScratch, Float.MAX_VALUE);
        if (cell == null || organelle == null) {
            return;
        }

        Vector2 orgPos = organelle.getPosition();
        for (Particle boundary : cell.boundaryParticles) {
            if (boundary == null || !boundary.isActive() || !boundary.isAlive()) {
                continue;
            }
            Vector2 boundaryPos = boundary.getPosition();
            float dx = boundaryPos.x - orgPos.x;
            float dy = boundaryPos.y - orgPos.y;
            float distSq = dx * dx + dy * dy;

            if (out.size < MAX_ANCHOR_CONNECTIONS) {
                out.add(boundary);
                anchorDistanceScratch[out.size - 1] = distSq;
            } else {
                int replaceIndex = 0;
                float farthestDist = anchorDistanceScratch[0];
                for (int i = 1; i < MAX_ANCHOR_CONNECTIONS; i++) {
                    if (anchorDistanceScratch[i] > farthestDist) {
                        farthestDist = anchorDistanceScratch[i];
                        replaceIndex = i;
                    }
                }
                if (distSq < farthestDist) {
                    out.set(replaceIndex, boundary);
                    anchorDistanceScratch[replaceIndex] = distSq;
                }
            }
        }
    }

    private void ensureAnchorsForOrganelle(Particle organelle, Array<Particle> targetBoundaries, float delta) {
        collectAnchorBondsForOrganelle(organelle, anchorBondBuffer);
        anchorTargetSet.clear();
        if (targetBoundaries != null) {
            for (Particle boundary : targetBoundaries) {
                if (boundary != null) {
                    anchorTargetSet.add(boundary);
                }
            }
        }

        if (targetBoundaries != null) {
            for (Particle boundary : targetBoundaries) {
                if (boundary == null || !boundary.isActive() || !boundary.isAlive()) {
                    continue;
                }
                Bond anchor = findAnchorBondInBuffer(organelle, boundary, anchorBondBuffer);
                if (anchor == null) {
                    createOrganelleAnchorBond(organelle, boundary);
                } else {
                    retuneAnchorBond(anchor, delta);
                }
            }
        }

        for (Bond anchor : anchorBondBuffer) {
            Particle boundary = anchor.getParticleA() == organelle ? anchor.getParticleB() : anchor.getParticleA();
            if (boundary == null || !boundary.isActive() || !boundary.isAlive()) {
                destroyBond(anchor, false, true);
                continue;
            }
            if (anchorTargetSet.size == 0 || !anchorTargetSet.contains(boundary)) {
                destroyBond(anchor, false, true);
            }
        }
    }

    private void collectAnchorBondsForOrganelle(Particle organelle, Array<Bond> out) {
        out.clear();
        if (organelle == null) {
            return;
        }
        for (Bond bond : activeBonds) {
            if (!bond.isActive() || bond.getBondType() != BondType.ANCHOR) {
                continue;
            }
            if (bond.getParticleA() == organelle || bond.getParticleB() == organelle) {
                out.add(bond);
            }
        }
    }

    private void removeAllAnchorBonds(Particle organelle) {
        if (organelle == null) {
            return;
        }
        collectAnchorBondsForOrganelle(organelle, anchorBondBuffer);
        for (Bond anchor : anchorBondBuffer) {
            destroyBond(anchor, false, true);
        }
        anchorBondBuffer.clear();
    }

    private void removeAllAnchorBonds() {
        for (int i = activeBonds.size - 1; i >= 0; i--) {
            Bond bond = activeBonds.get(i);
            if (bond.isActive() && bond.getBondType() == BondType.ANCHOR) {
                destroyBond(bond, false, true);
            }
        }
    }

    private Bond findAnchorBondInBuffer(Particle organelle, Particle boundary, Array<Bond> buffer) {
        if (buffer == null) {
            return null;
        }
        for (Bond bond : buffer) {
            if ((bond.getParticleA() == organelle && bond.getParticleB() == boundary) ||
                (bond.getParticleB() == organelle && bond.getParticleA() == boundary)) {
                return bond;
            }
        }
        return null;
    }

    private void retuneAnchorBond(Bond bond, float delta) {
        Particle a = bond.getParticleA();
        Particle b = bond.getParticleB();
        if (a == null || b == null) {
            return;
        }
        float desiredRest = a.getPosition().dst(b.getPosition()) * ANCHOR_REST_SLACK;
        float currentRest = bond.getRestLength();
        float restDelta = desiredRest - currentRest;
        if (Math.abs(restDelta) < 0.0001f) {
            return;
        }
        float maxStep = Math.max(0.001f, desiredRest * ANCHOR_REST_ADJUST_RATE * delta);
        restDelta = MathUtils.clamp(restDelta, -maxStep, maxStep);
        bond.setRestLength(currentRest + restDelta);
    }

    private Bond createOrganelleAnchorBond(Particle organelle, Particle boundary) {
        if (organelle == null || boundary == null) {
            return null;
        }
        Vector2 orgPos = organelle.getPosition();
        Vector2 boundaryPos = boundary.getPosition();
        float restLength = orgPos.dst(boundaryPos) * ANCHOR_REST_SLACK;
        Bond bond = bondPool.obtain();
        bond.init(organelle, boundary, BondType.ANCHOR, restLength, ANCHOR_STIFFNESS, ANCHOR_DAMPING, ANCHOR_BREAK_FORCE);
        bond.setUnbreakable(true);
        activeBonds.add(bond);

        if (unionFind.union(organelle.hashCode(), boundary.hashCode())) {
            int newComponentId = unionFind.find(organelle.hashCode());
            organelle.setComponentId(newComponentId);
            boundary.setComponentId(newComponentId);
        }

        return bond;
    }
    
    /**
     * Apply chemotaxis forces - particles follow energy gradients.
     * TRUE particles move toward high energy (positive chemotaxis).
     * FALSE particles move toward low energy (negative chemotaxis).
     */
    private void applyChemotaxis() {
        if (chemotaxisStrength <= 0f) return;
        
        float[] gradient = new float[2];
        
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            if (particle.isOrganelle()) continue;
            
            Vector2 particlePos = particle.getPosition();
            ParticleType particleType = particle.getType();
            
            // Accumulate forces from all active moving fields
            float totalForceX = 0f;
            float totalForceY = 0f;
            
            for (MovingEnergyField field : movingFields) {
                if (!field.isActive()) continue;
                
                // Get gradient direction toward this field
                field.getGradientDirection(particlePos.x, particlePos.y, gradient);
                
                // Get type-specific affinity (positive=attracted, negative=repelled)
                float affinity = field.getType().getAffinityFor(particleType);
                
                // Get field energy at particle position
                float fieldEnergy = field.getEnergyAt(particlePos.x, particlePos.y);
                
                // Apply force based on affinity and field strength
                float baseForce = chemotaxisStrength * affinity * fieldEnergy;
                
                // Energy field disruption: large molecules experience stronger pulling forces
                // This creates osmotic-like pressure that can tear molecules apart
                int moleculeSize = getComponentSize(particle);
                if (moleculeSize > 15) {
                    // Disruption multiplier increases with size
                    float disruptionMultiplier = 1.0f + (moleculeSize - 15) * 0.05f;
                    baseForce *= disruptionMultiplier;
                }
                
                totalForceX += gradient[0] * baseForce;
                totalForceY += gradient[1] * baseForce;
            }
            
            // Apply accumulated force
            particle.getBody().applyForceToCenter(totalForceX, totalForceY, true);
        }
    }
    
    /**
     * Apply hunting behavior - molecules with weak bonds actively seek food particles.
     * This creates predator-like behavior where energy-depleted structures pursue sustenance.
     */
    private void applyHuntingBehavior() {
        if (huntingRange <= 0f || huntingForce <= 0f) return; // Hunting disabled
        
        final float WEAK_THRESHOLD = 0.6f; // Consider bonds weak if < 60% of base strength
        
        // Group particles by molecule to calculate average bond strength
        com.badlogic.gdx.utils.IntMap<Array<Particle>> moleculeGroups = new com.badlogic.gdx.utils.IntMap<>();
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            if (particle.isOrganelle()) continue;
            
            int componentId = particle.getComponentId();
            Array<Particle> group = moleculeGroups.get(componentId);
            if (group == null) {
                group = new Array<>();
                moleculeGroups.put(componentId, group);
            }
            group.add(particle);
        }

        // Precompute bond strength aggregates per component (avoids O(M * B) scans)
        IntFloatMap componentBondForceSum = new IntFloatMap();
        IntIntMap componentBondCounts = new IntIntMap();
        for (Bond bond : activeBonds) {
            if (!bond.isActive()) continue;
            Particle a = bond.getParticleA();
            Particle b = bond.getParticleB();
            int componentA = a.getComponentId();
            if (componentA != b.getComponentId()) continue; // Only consider intra-molecule bonds
            float sum = componentBondForceSum.get(componentA, 0f) + bond.getEffectiveBreakForce();
            componentBondForceSum.put(componentA, sum);
            componentBondCounts.put(componentA, componentBondCounts.get(componentA, 0) + 1);
        }
        
        // For each molecule, check if it needs to hunt
        for (com.badlogic.gdx.utils.IntMap.Entry<Array<Particle>> entry : moleculeGroups) {
            Array<Particle> molecule = entry.value;
            int componentId = entry.key;
            
            if (molecule.size < 2) continue; // Solo particles don't hunt cooperatively
            
            // Calculate average energy level of the molecule
            float avgEnergy = 0f;
            for (Particle p : molecule) {
                avgEnergy += p.getEnergy();
            }
            avgEnergy /= molecule.size;
            
            // Check average bond strength for this molecule
            int bondCount = componentBondCounts.get(componentId, 0);
            if (bondCount == 0) continue;
            float avgBondStrength = componentBondForceSum.get(componentId, 0f) / bondCount;
            
            // Determine if molecule is "hungry" (weak bonds or low energy)
            float normalizedStrength = avgBondStrength / 200f; // 200 is base break force
            boolean isHungry = normalizedStrength < WEAK_THRESHOLD || avgEnergy < 40f;
            
            if (!isHungry) continue;
            
            // Calculate molecule center of mass
            float cx = 0, cy = 0;
            for (Particle p : molecule) {
                Vector2 pos = p.getPosition();
                cx += pos.x;
                cy += pos.y;
            }
            cx /= molecule.size;
            cy /= molecule.size;
            
            // Find nearest food particle (energy-rich particle)
            Particle nearestFood = null;
            float nearestDistSq = huntingRange * huntingRange;
            
            for (Particle food : activeParticles) {
                if (!food.isActive() || !food.isEnergyRich()) continue;
                
                Vector2 foodPos = food.getPosition();
                float dx = foodPos.x - cx;
                float dy = foodPos.y - cy;
                float distSq = dx * dx + dy * dy;
                
                if (distSq < nearestDistSq) {
                    nearestDistSq = distSq;
                    nearestFood = food;
                }
            }
            
            // If food found, all particles in molecule move toward it
            if (nearestFood != null) {
                Vector2 foodPos = nearestFood.getPosition();
                
                // Calculate hunting intensity based on how desperate the molecule is
                float desperation = 1.0f - normalizedStrength; // 0 = strong, 1 = very weak
                desperation = Math.max(desperation, (40f - avgEnergy) / 40f); // Also factor in energy
                desperation = Math.max(0f, Math.min(1f, desperation));
                
                for (Particle p : molecule) {
                    Vector2 pPos = p.getPosition();
                    float dx = foodPos.x - pPos.x;
                    float dy = foodPos.y - pPos.y;
                    float dist = (float) Math.sqrt(dx * dx + dy * dy);
                    
                    if (dist > 0.1f) {
                        // Apply hunting force
                        float forceX = (dx / dist) * huntingForce * desperation;
                        float forceY = (dy / dist) * huntingForce * desperation;
                        p.getBody().applyForceToCenter(forceX, forceY, true);
                    }
                }
            }
        }
    }
    
    /**
     * Apply food suction - molecules pull nearby energy-rich particles toward them.
     * This dramatically increases eating efficiency by actively drawing food into consumption range.
     * Larger molecules exert stronger suction (more "mouths" to feed).
     */
    private void applyFoodSuction() {
        if (suctionRange <= 0f || suctionForce <= 0f) return; // Suction disabled
        
        // Group living particles by molecule
        com.badlogic.gdx.utils.IntMap<Array<Particle>> moleculeGroups = new com.badlogic.gdx.utils.IntMap<>();
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            if (particle.isOrganelle()) continue; // Organelles don't participate in suction
            
            int componentId = particle.getComponentId();
            Array<Particle> group = moleculeGroups.get(componentId);
            if (group == null) {
                group = new Array<>();
                moleculeGroups.put(componentId, group);
            }
            group.add(particle);
        }
        
        // For each molecule, find nearby food and pull it in
        for (com.badlogic.gdx.utils.IntMap.Entry<Array<Particle>> entry : moleculeGroups) {
            Array<Particle> molecule = entry.value;
            
            if (molecule.size < 2) continue; // Solo particles don't create suction
            
            // Calculate molecule center of mass
            float cx = 0, cy = 0;
            for (Particle p : molecule) {
                Vector2 pos = p.getPosition();
                cx += pos.x;
                cy += pos.y;
            }
            cx /= molecule.size;
            cy /= molecule.size;
            
            // Suction strength scales with molecule size (more particles = stronger pull)
            // Formula: log-based scaling to prevent extreme forces
            float sizeMultiplier = 1.0f + (float) Math.log(molecule.size) * 0.3f;
            
            // Find all food particles within suction range (both living energy-rich and corpses)
            Array<Particle> nearbyCandidates = queryRadius(cx, cy, suctionRange);
            for (Particle food : nearbyCandidates) {
                if (!food.isActive()) continue;
                
                // Pull in energy-rich particles (living food) and corpses (dead particles)
                boolean isFood = food.isEnergyRich() && food.isAlive();
                boolean isCorpse = !food.isAlive();
                
                if (!isFood && !isCorpse) continue;
                
                // Don't pull particles from this molecule
                if (molecule.contains(food, true)) continue;
                
                Vector2 foodPos = food.getPosition();
                float dx = foodPos.x - cx;
                float dy = foodPos.y - cy;
                float distSq = dx * dx + dy * dy;
                float dist = (float) Math.sqrt(distSq);
                
                if (dist > suctionRange || dist < 0.5f) continue; // Too far or too close
                
                // Calculate suction force using inverse-square-like falloff
                // Stronger when food is closer, scales with molecule size
                float distanceFactor = 1.0f - (dist / suctionRange); // 1.0 at center, 0.0 at edge
                float suctionForceMagnitude = suctionForce * sizeMultiplier * distanceFactor * distanceFactor;
                
                // Corpses are pulled more strongly (2x) since they're the primary food source
                if (isCorpse) {
                    suctionForceMagnitude *= 2.0f;
                }
                
                // Apply force toward molecule center
                float forceX = -(dx / dist) * suctionForceMagnitude;
                float forceY = -(dy / dist) * suctionForceMagnitude;
                food.getBody().applyForceToCenter(forceX, forceY, true);
            }
        }
    }
    
    /**
     * Apply crowding pressure - particles in dense regions experience repulsive forces.
     * This simulates cytoskeletal stress and membrane tension in cells.
     * Helps break up overly dense molecular structures.
     */
    private void applyCrowdingPressure() {
        // Crowding radius - distance at which particles start to feel repulsion
        final float CROWDING_RADIUS = 2.5f;
        final float CROWDING_STRENGTH = 30f; // Force magnitude
        
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            if (particle.isOrganelle()) continue;
            
            Vector2 particlePos = particle.getPosition();
            
            // Query nearby particles
            Array<Particle> neighbors = queryNeighbors(particle, CROWDING_RADIUS);
            
            if (neighbors.size < 3) continue; // Not crowded enough
            
            // Calculate crowding density (more neighbors = stronger pressure)
            float crowdingFactor = Math.min(1.0f, neighbors.size / 8.0f); // Cap at 8 neighbors
            
            // Calculate net repulsion direction
            float repulsionX = 0f;
            float repulsionY = 0f;
            
            for (Particle neighbor : neighbors) {
                if (!neighbor.isAlive()) continue; // Don't interact with corpses
                if (neighbor.isOrganelle()) continue;
                
                Vector2 neighborPos = neighbor.getPosition();
                float dx = particlePos.x - neighborPos.x;
                float dy = particlePos.y - neighborPos.y;
                float distSq = dx * dx + dy * dy;
                
                if (distSq < 0.01f) continue; // Too close, avoid division by zero
                
                float dist = (float) Math.sqrt(distSq);
                
                // Inverse square law for repulsion (stronger when very close)
                float repulsionMag = 1.0f / (distSq + 0.1f);
                
                // Normalized direction away from neighbor
                repulsionX += (dx / dist) * repulsionMag;
                repulsionY += (dy / dist) * repulsionMag;
            }
            
            // Apply crowding pressure force
            float forceX = repulsionX * CROWDING_STRENGTH * crowdingFactor;
            float forceY = repulsionY * CROWDING_STRENGTH * crowdingFactor;
            
            particle.getBody().applyForceToCenter(forceX, forceY, true);
        }
    }
    
    /**
     * Apply intramolecular spacing forces - particles within the same molecule 
     * repel each other slightly to prevent clumping and encourage surface area expansion.
     * This creates more open, spread-out molecular structures without breaking bonds.
     */
    private void applyIntramolecularSpacing() {
        final float SPACING_RADIUS = 4.2f; // Particles within this distance repel
        final float SPACING_STRENGTH = 38f; // Base repulsion force
        
        // Group particles by molecule
        com.badlogic.gdx.utils.IntMap<Array<Particle>> moleculeGroups = new com.badlogic.gdx.utils.IntMap<>();
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            if (particle.isOrganelle()) continue;
            
            int componentId = particle.getComponentId();
            Array<Particle> group = moleculeGroups.get(componentId);
            if (group == null) {
                group = new Array<>();
                moleculeGroups.put(componentId, group);
            }
            group.add(particle);
        }
        
        ObjectMap<Particle, Array<Particle>> adjacency = buildSpacingAdjacencyCache();

        // For each molecule, apply spacing forces between its particles
        for (com.badlogic.gdx.utils.IntMap.Entry<Array<Particle>> entry : moleculeGroups) {
            Array<Particle> molecule = entry.value;
            
            if (molecule.size < 2) continue; // Solo particles don't need spacing
            
            // Apply repulsion between all pairs within the molecule
            for (int i = 0; i < molecule.size; i++) {
                Particle pA = molecule.get(i);
                if (pA.isOrganelle()) continue;
                Vector2 posA = pA.getPosition();
                
                for (int j = i + 1; j < molecule.size; j++) {
                    Particle pB = molecule.get(j);
                    if (pB.isOrganelle()) continue;
                    Vector2 posB = pB.getPosition();
                    
                    float dx = posA.x - posB.x;
                    float dy = posA.y - posB.y;
                    float distSq = dx * dx + dy * dy;
                    float dist = (float) Math.sqrt(distSq);
                    
                    // Only apply spacing if within range
                    if (dist < SPACING_RADIUS && dist > 0.1f) {
                        // Spacing force falls off with distance (stronger when closer)
                        float spacingFactor = (SPACING_RADIUS - dist) / SPACING_RADIUS;
                        
                        boolean directlyBonded = areDirectNeighbors(pA, pB, adjacency);
                        float forceMult = directlyBonded ? 0.35f : 1.0f;
                        float force = SPACING_STRENGTH * spacingFactor * forceMult;
                        
                        // Apply equal and opposite forces
                        float forceX = (dx / dist) * force;
                        float forceY = (dy / dist) * force;
                        
                        pA.getBody().applyForceToCenter(forceX, forceY, true);
                        pB.getBody().applyForceToCenter(-forceX, -forceY, true);
                    }
                }
            }
        }
    }

    private ObjectMap<Particle, Array<Particle>> buildSpacingAdjacencyCache() {
        if (spacingAdjacencyCache.size > 0) {
            for (ObjectMap.Entry<Particle, Array<Particle>> entry : spacingAdjacencyCache.entries()) {
                entry.value.clear();
                spacingAdjacencyPool.add(entry.value);
            }
            spacingAdjacencyCache.clear();
        }

        for (Bond bond : activeBonds) {
            if (!bond.isActive()) continue;
            if (bond.getBondType() == BondType.ANCHOR) continue;
            addAdjacencyNeighbor(bond.getParticleA(), bond.getParticleB());
            addAdjacencyNeighbor(bond.getParticleB(), bond.getParticleA());
        }

        return spacingAdjacencyCache;
    }

    private void addAdjacencyNeighbor(Particle particle, Particle neighbor) {
        if (particle == null || neighbor == null) {
            return;
        }
        Array<Particle> neighbors = spacingAdjacencyCache.get(particle);
        if (neighbors == null) {
            neighbors = spacingAdjacencyPool.size > 0 ? spacingAdjacencyPool.pop() : new Array<>();
            spacingAdjacencyCache.put(particle, neighbors);
        }
        neighbors.add(neighbor);
    }

    private boolean areDirectNeighbors(Particle a, Particle b, ObjectMap<Particle, Array<Particle>> adjacencyCache) {
        if (a == null || b == null || adjacencyCache == null) {
            return false;
        }
        Array<Particle> neighbors = adjacencyCache.get(a);
        if (neighbors == null) {
            return false;
        }
        return neighbors.contains(b, true);
    }
    
    /**
     * Update the age of all active particles and bonds.
     * Aging causes bonds to become more fragile over time.
     */
    private void updateAging(float delta) {
        // Age particles
        for (Particle particle : activeParticles) {
            if (particle.isActive()) {
                particle.updateAge(delta);
            }
        }
        
        // Age bonds
        for (Bond bond : activeBonds) {
            if (bond.isActive()) {
                bond.updateAge(delta);
            }
        }
        
        // Update mitosis events and remove expired ones
        for (int i = activeMitosisEvents.size - 1; i >= 0; i--) {
            MitosisEvent event = activeMitosisEvents.get(i);
            event.lockTimeRemaining -= delta;
            event.flashTimeRemaining -= delta;
            
            // Remove event when both timers expire
            if (event.lockTimeRemaining <= 0f && event.flashTimeRemaining <= 0f) {
                activeMitosisEvents.removeIndex(i);
            }
        }
    }
    
    /**
     * Process molecular replication.
     * Detect simple bonded patterns (pairs, triangles) and replicate them
     * if conditions are met (high energy, space available).
     */
    private void processReplication(float delta) {
        replicationCheckAccumulator += delta;
        
        if (replicationCheckAccumulator < replicationCheckInterval) {
            return;  // Not time to check yet
        }
        
        replicationCheckAccumulator = 0f;
        
        // Find replicable molecules (simple pairs with high energy)
        for (int i = 0; i < activeBonds.size; i++) {
            Bond bond = activeBonds.get(i);
            if (!bond.isActive()) continue;
            
            Particle a = bond.getParticleA();
            Particle b = bond.getParticleB();
            
            // Check if both particles have sufficient energy
            if (a.getEnergy() < replicationEnergyThreshold || 
                b.getEnergy() < replicationEnergyThreshold) {
                continue;
            }
            
            // Check if this is a simple pair (each particle has only 1 bond)
            int bondsA = countBonds(a);
            int bondsB = countBonds(b);
            
            if (bondsA != 1 || bondsB != 1) {
                continue;  // Not a simple pair
            }
            
            // Find space for replication (nearby but not too close)
            Vector2 midpoint = new Vector2(a.getPosition()).add(b.getPosition()).scl(0.5f);
            Vector2 replicaPos = findReplicationSpace(midpoint, 4f, 8f);
            
            if (replicaPos == null) {
                continue;  // No space available
            }
            
            // Replicate the pair!
            replicatePair(a, b, replicaPos, bond);
            
            // Consume significant energy from parents
            a.consumeEnergy(40f);
            b.consumeEnergy(40f);
            
            break;  // Only one replication per check interval
        }
    }
    
    /**
     * Count how many bonds a particle has.
     */
    private int countBonds(Particle particle) {
        int count = 0;
        for (int i = 0; i < activeBonds.size; i++) {
            Bond bond = activeBonds.get(i);
            if (!bond.isActive()) continue;
            if (bond.getParticleA() == particle || bond.getParticleB() == particle) {
                count++;
            }
        }
        return count;
    }
    
    /**
     * Find empty space for replication near a position.
     * Returns null if no suitable space found.
     */
    private Vector2 findReplicationSpace(Vector2 center, float minDist, float maxDist) {
        // Try 8 directions around the center
        float[] angles = {0, 45, 90, 135, 180, 225, 270, 315};
        
        for (float angle : angles) {
            float rad = (float) Math.toRadians(angle);
            float distance = minDist + (maxDist - minDist) * 0.5f;
            
            float x = center.x + (float) Math.cos(rad) * distance;
            float y = center.y + (float) Math.sin(rad) * distance;
            
            // Check if position is clear (no particles within 3 units)
            boolean isClear = true;
            for (Particle p : activeParticles) {
                if (!p.isActive()) continue;
                float dx = p.getPosition().x - x;
                float dy = p.getPosition().y - y;
                if (dx * dx + dy * dy < 9f) {  // 3 units radius
                    isClear = false;
                    break;
                }
            }
            
            if (isClear) {
                return new Vector2(x, y);
            }
        }
        
        return null;  // No space found
    }
    
    /**
     * Create a replica of a bonded pair at the specified position.
     */
    private void replicatePair(Particle parentA, Particle parentB, Vector2 replicaCenter, Bond parentBond) {
        // Calculate offset for the two child particles
        Vector2 direction = new Vector2(parentB.getPosition()).sub(parentA.getPosition());
        float bondLength = direction.len();
        direction.nor();
        
        Vector2 posA = new Vector2(replicaCenter).sub(direction.scl(bondLength * 0.5f));
        Vector2 posB = new Vector2(replicaCenter).add(direction.scl(bondLength * 0.5f));
        
        // Determine child types (with possible mutation)
        ParticleType typeA = shouldMutate() ? getRandomParticleType() : parentA.getType();
        ParticleType typeB = shouldMutate() ? getRandomParticleType() : parentB.getType();
        
        // Spawn replica particles (possibly mutated)
        Particle childA = spawnParticle(typeA, posA, 0.5f, 1f);
        Particle childB = spawnParticle(typeB, posB, 0.5f, 1f);
        
        // Give children moderate starting energy
        childA.setEnergy(60f);
        childB.setEnergy(60f);
        
        // Create bond between children (copy parent bond properties)
        createBond(childA, childB, BondType.ELASTIC, 
                   50f, 2f, parentBond.getBreakForceThreshold());
        
        // Give children small random velocity (ejection from parent)
        Vector2 perpendicular = new Vector2(-direction.y, direction.x);
        float vx = perpendicular.x * 5f;
        float vy = perpendicular.y * 5f;
        childA.getBody().setLinearVelocity(vx, vy);
        childB.getBody().setLinearVelocity(vx, vy);
    }
    
    /**
     * Check if mutation should occur based on mutation rate.
     */
    private boolean shouldMutate() {
        return Math.random() < mutationRate;
    }
    
    /**
     * Get a random particle type for mutation.
     */
    private ParticleType getRandomParticleType() {
        ParticleType[] types = ParticleType.values();
        return types[(int) (Math.random() * types.length)];
    }
    
    /**
     * Process mitosis (cell division) for oversized molecules.
     * When a molecule exceeds the size threshold, find the best place to divide it.
     */
    private void processMitosis(float delta) {
        if (mitosisThreshold <= 0) return;
        
        mitosisCheckAccumulator += delta;
        float targetInterval = Math.min(mitosisCheckInterval, mitosisFastCheckInterval);
        if (targetInterval <= 0f) {
            targetInterval = 1f;
        }
        if (mitosisCheckAccumulator < targetInterval) return;
        mitosisCheckAccumulator -= targetInterval;
        if (mitosisCheckAccumulator < 0f) {
            mitosisCheckAccumulator = 0f;
        }

        // Ensure component IDs reflect current connectivity before grouping molecules
        if (componentsNeedRebuild) {
            rebuildUnionFind();
        }
        
        // Group particles by component ID to identify molecules
        com.badlogic.gdx.utils.IntMap<Array<Particle>> componentGroups = new com.badlogic.gdx.utils.IntMap<>();
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            
            int componentId = particle.getComponentId();
            Array<Particle> group = componentGroups.get(componentId);
            if (group == null) {
                group = new Array<>();
                componentGroups.put(componentId, group);
            }
            group.add(particle);
        }
        
        // Check each molecule for size threshold or metabolic stress
        int divisionsThisFrame = 0;
        for (com.badlogic.gdx.utils.IntMap.Entry<Array<Particle>> entry : componentGroups) {
            Array<Particle> molecule = entry.value;
            
            // Skip single particles or molecules too small to divide
            if (molecule.size < 3) continue;
            
            // Trigger 1: Size threshold exceeded
            boolean oversized = molecule.size >= mitosisThreshold;
            
            // Trigger 2: Metabolic stress - average energy is critically low and molecule is large
            float avgEnergy = 0f;
            for (Particle p : molecule) {
                avgEnergy += p.getEnergy();
            }
            avgEnergy /= molecule.size;
            boolean metabolicStress = molecule.size > 15 && avgEnergy < 30f;
            
            if (oversized || metabolicStress) {
                divideMolecule(molecule);
                divisionsThisFrame++;
                if (divisionsThisFrame >= MAX_MITOSIS_DIVISIONS_PER_CHECK) {
                    break;
                }
            }
        }
    }
    
    /**
     * Divide a molecule by breaking bonds along its narrowest cross-section.
     * Strategy: Find the bond whose removal would best split the molecule into two equal halves.
     */
    private void divideMolecule(Array<Particle> moleculeParticles) {
        if (moleculeParticles.size < 3) return;

        Vector2 centerOfMass = calculateCenterOfMass(moleculeParticles);
        float maxRadius = calculateMaxRadius(moleculeParticles, centerOfMass);

        java.util.Set<Particle> moleculeSet = new java.util.HashSet<>(moleculeParticles.size);
        for (Particle particle : moleculeParticles) {
            moleculeSet.add(particle);
        }

        Array<Bond> moleculeBonds = collectMoleculeBonds(moleculeSet);
        if (moleculeBonds.size == 0) return;

        Vector2 primaryAxis = findPrimaryAxis(moleculeParticles, centerOfMass);
        float splittingBand = Math.max(MIN_MITOSIS_SPLIT_BAND, maxRadius * 0.2f);

        Array<Bond> bondsToBreak = new Array<>();
        for (Bond bond : moleculeBonds) {
            if (!bond.isActive()) continue;
            if (crossesCenterBand(bond, centerOfMass, primaryAxis, splittingBand)) {
                bondsToBreak.add(bond);
            }
        }

        if (bondsToBreak.size == 0) {
            Bond fallback = findClosestBondToPoint(moleculeBonds, centerOfMass);
            if (fallback != null) {
                bondsToBreak.add(fallback);
            }
        }

        if (bondsToBreak.size == 0) {
            return;
        }

        float releasedBondEnergy = 0f;
        for (Bond bond : bondsToBreak) {
            releasedBondEnergy += bond.getBreakForceThreshold();
            destroyBond(bond, false, true);
        }

        Array<Array<Particle>> daughterCells = extractMoleculeComponents(moleculeParticles, moleculeBonds, moleculeSet);
        if (daughterCells.size < 2) {
            return;
        }

        Array<Particle> daughterCell1 = null;
        Array<Particle> daughterCell2 = null;
        for (Array<Particle> cell : daughterCells) {
            if (daughterCell1 == null || cell.size > daughterCell1.size) {
                daughterCell2 = daughterCell1;
                daughterCell1 = cell;
            } else if (daughterCell2 == null || cell.size > daughterCell2.size) {
                daughterCell2 = cell;
            }
        }

        if (daughterCell1 == null || daughterCell2 == null) {
            return;
        }

        for (Array<Particle> cell : daughterCells) {
            if (cell == daughterCell1 || cell == daughterCell2) continue;
            if (daughterCell1.size <= daughterCell2.size) {
                daughterCell1.addAll(cell);
            } else {
                daughterCell2.addAll(cell);
            }
        }

        float energyPerCell = releasedBondEnergy * 0.5f;
        redistributeBondEnergy(daughterCell1, energyPerCell);
        redistributeBondEnergy(daughterCell2, energyPerCell);

        Vector2 center1 = calculateCenterOfMass(daughterCell1);
        Vector2 center2 = calculateCenterOfMass(daughterCell2);
        float pushForce = MathUtils.clamp(
            releasedBondEnergy * MITOSIS_PUSH_FORCE_SCALE,
            MIN_MITOSIS_PUSH_FORCE,
            MAX_MITOSIS_PUSH_FORCE
        );
        applyEqualOpposingPush(center1, center2, daughterCell1, daughterCell2, pushForce, primaryAxis);

        registerMitosisEvent(daughterCell1, daughterCell2);
    }

    private Array<Bond> collectMoleculeBonds(java.util.Set<Particle> moleculeSet) {
        Array<Bond> bonds = new Array<>();
        for (Bond bond : activeBonds) {
            if (!bond.isActive()) continue;
            if (moleculeSet.contains(bond.getParticleA()) && moleculeSet.contains(bond.getParticleB())) {
                bonds.add(bond);
            }
        }
        return bonds;
    }

    private Vector2 calculateCenterOfMass(Array<Particle> particles) {
        Vector2 center = new Vector2();
        if (particles.size == 0) {
            return center;
        }
        for (Particle particle : particles) {
            center.add(particle.getPosition());
        }
        center.scl(1f / particles.size);
        return center;
    }

    private float calculateMaxRadius(Array<Particle> particles, Vector2 center) {
        float maxRadius = 0f;
        for (Particle particle : particles) {
            Vector2 pos = particle.getPosition();
            float dx = pos.x - center.x;
            float dy = pos.y - center.y;
            float dist = (float) Math.sqrt(dx * dx + dy * dy);
            if (dist > maxRadius) {
                maxRadius = dist;
            }
        }
        return maxRadius;
    }

    private Vector2 findPrimaryAxis(Array<Particle> particles, Vector2 center) {
        Vector2 axis = new Vector2(1f, 0f);
        float maxDistSq = 0f;
        for (Particle particle : particles) {
            Vector2 pos = particle.getPosition();
            float dx = pos.x - center.x;
            float dy = pos.y - center.y;
            float distSq = dx * dx + dy * dy;
            if (distSq > maxDistSq) {
                maxDistSq = distSq;
                axis.set(dx, dy);
            }
        }
        if (axis.isZero(0.0001f)) {
            axis.set(1f, 0f);
        } else {
            axis.nor();
        }
        return axis;
    }

    private boolean crossesCenterBand(Bond bond, Vector2 center, Vector2 normal, float bandWidth) {
        Vector2 posA = bond.getParticleA().getPosition();
        Vector2 posB = bond.getParticleB().getPosition();
        float relAx = posA.x - center.x;
        float relAy = posA.y - center.y;
        float relBx = posB.x - center.x;
        float relBy = posB.y - center.y;
        float projA = relAx * normal.x + relAy * normal.y;
        float projB = relBx * normal.x + relBy * normal.y;
        if (Math.abs(projA) < 0.0001f) projA = 0f;
        if (Math.abs(projB) < 0.0001f) projB = 0f;

        if ((projA > 0f && projB > 0f) || (projA < 0f && projB < 0f)) {
            return false;
        }

        float midpointProjection = Math.abs((projA + projB) * 0.5f);
        return midpointProjection <= bandWidth;
    }

    private Bond findClosestBondToPoint(Array<Bond> bonds, Vector2 point) {
        Bond closest = null;
        float minDist = Float.MAX_VALUE;
        for (Bond bond : bonds) {
            if (!bond.isActive()) continue;
            Vector2 posA = bond.getParticleA().getPosition();
            Vector2 posB = bond.getParticleB().getPosition();
            float midX = (posA.x + posB.x) * 0.5f;
            float midY = (posA.y + posB.y) * 0.5f;
            float dist = Vector2.dst(midX, midY, point.x, point.y);
            if (dist < minDist) {
                minDist = dist;
                closest = bond;
            }
        }
        return closest;
    }

    private Array<Array<Particle>> extractMoleculeComponents(
        Array<Particle> moleculeParticles,
        Array<Bond> moleculeBonds,
        java.util.Set<Particle> moleculeSet
    ) {
        Array<Array<Particle>> components = new Array<>();
        java.util.Set<Particle> visited = new java.util.HashSet<>();
        Array<Particle> queue = new Array<>();

        for (Particle start : moleculeParticles) {
            if (!moleculeSet.contains(start) || visited.contains(start)) continue;
            queue.clear();
            queue.add(start);
            visited.add(start);

            Array<Particle> component = new Array<>();
            while (queue.size > 0) {
                Particle current = queue.pop();
                component.add(current);
                for (Bond bond : moleculeBonds) {
                    if (!bond.isActive()) continue;
                    Particle neighbor = null;
                    if (bond.getParticleA() == current) {
                        neighbor = bond.getParticleB();
                    } else if (bond.getParticleB() == current) {
                        neighbor = bond.getParticleA();
                    }

                    if (neighbor != null && moleculeSet.contains(neighbor) && !visited.contains(neighbor)) {
                        visited.add(neighbor);
                        queue.add(neighbor);
                    }
                }
            }

            if (component.size > 0) {
                components.add(component);
            }
        }

        return components;
    }

    private void redistributeBondEnergy(Array<Particle> particles, float energyShare) {
        if (particles.size == 0 || energyShare <= 0f) {
            return;
        }

        java.util.Set<Particle> particleSet = new java.util.HashSet<>(particles.size);
        for (Particle particle : particles) {
            particleSet.add(particle);
        }

        Array<Bond> recipientBonds = new Array<>();
        for (Bond bond : activeBonds) {
            if (!bond.isActive()) continue;
            if (particleSet.contains(bond.getParticleA()) && particleSet.contains(bond.getParticleB())) {
                recipientBonds.add(bond);
            }
        }

        if (recipientBonds.size == 0) {
            float perParticleEnergy = energyShare / particles.size;
            for (Particle particle : particles) {
                particle.addEnergy(perParticleEnergy);
            }
            return;
        }

        float energyPerBond = energyShare / recipientBonds.size;
        for (Bond bond : recipientBonds) {
            bond.increaseBreakForceThreshold(energyPerBond);
        }
    }

    private void applyEqualOpposingPush(
        Vector2 center1,
        Vector2 center2,
        Array<Particle> cell1,
        Array<Particle> cell2,
        float forceMagnitude,
        Vector2 fallbackDirection
    ) {
        if (cell1.size == 0 || cell2.size == 0) {
            return;
        }

        Vector2 direction = new Vector2(center2).sub(center1);
        if (direction.isZero(0.0001f)) {
            if (fallbackDirection != null) {
                direction.set(fallbackDirection);
            } else {
                direction.set(1f, 0f);
            }
        }

        if (!direction.isZero(0.0001f)) {
            direction.nor();
        } else {
            direction.set(1f, 0f);
        }

        float perParticleForce1 = forceMagnitude / cell1.size;
        float perParticleForce2 = forceMagnitude / cell2.size;

        for (Particle particle : cell1) {
            Body body = particle.getBody();
            if (body != null) {
                body.applyForceToCenter(-direction.x * perParticleForce1, -direction.y * perParticleForce1, true);
            }
        }

        for (Particle particle : cell2) {
            Body body = particle.getBody();
            if (body != null) {
                body.applyForceToCenter(direction.x * perParticleForce2, direction.y * perParticleForce2, true);
            }
        }
    }

    private void registerMitosisEvent(Array<Particle> daughterCell1, Array<Particle> daughterCell2) {
        float lockDuration = 3.5f;
        float flashDuration = 0.8f;
        java.util.Set<Particle> mitosisParticles = new java.util.HashSet<>();
        for (Particle particle : daughterCell1) {
            mitosisParticles.add(particle);
        }
        for (Particle particle : daughterCell2) {
            mitosisParticles.add(particle);
        }
        activeMitosisEvents.add(new MitosisEvent(mitosisParticles, lockDuration, flashDuration));
    }
    
    /**
     * Check if a particle is currently locked from mitosis (can't form bonds).
     */
    public boolean isParticleMitosisLocked(Particle particle) {
        for (MitosisEvent event : activeMitosisEvents) {
            if (event.lockTimeRemaining > 0f && event.particles.contains(particle)) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * Check if a particle is currently flashing from mitosis (visual effect).
     */
    public boolean isParticleMitosisFlashing(Particle particle) {
        for (MitosisEvent event : activeMitosisEvents) {
            if (event.flashTimeRemaining > 0f && event.particles.contains(particle)) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * Get the flash intensity for a particle (0.0 to 1.0).
     */
    public float getParticleMitosisFlashIntensity(Particle particle) {
        float maxFlash = 0f;
        for (MitosisEvent event : activeMitosisEvents) {
            if (event.flashTimeRemaining > 0f && event.particles.contains(particle)) {
                float intensity = event.flashTimeRemaining / 0.8f; // Normalize by flash duration
                maxFlash = Math.max(maxFlash, intensity);
            }
        }
        return maxFlash;
    }

    /**
     * Determine if a bond should be protected from breaking while mitosis cooldown is active.
     */
    private boolean isBondProtectedByMitosis(Bond bond) {
        Particle a = bond.getParticleA();
        Particle b = bond.getParticleB();
        for (MitosisEvent event : activeMitosisEvents) {
            if (event.lockTimeRemaining > 0f &&
                event.particles.contains(a) &&
                event.particles.contains(b)) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * Remove particles that have zero energy.
     */
    private void removeDeadParticles() {
        Array<Particle> particlesToKill = new Array<>();
        
        for (Particle particle : activeParticles) {
            if (particle.isOrganelle()) continue;
            if (particle.isActive() && particle.getEnergy() <= 0f && particle.isAlive()) {
                particlesToKill.add(particle);
            }
        }
        
        for (Particle particle : particlesToKill) {
            // Remove all bonds connected to this particle first
            Array<Bond> bondsToRemove = new Array<>();
            for (Bond bond : activeBonds) {
                if (bond.isActive() && 
                    (bond.getParticleA() == particle || bond.getParticleB() == particle)) {
                    bondsToRemove.add(bond);
                }
            }
            
            for (Bond bond : bondsToRemove) {
                destroyBond(bond, false, true);  // No violent reaction on energy death
            }
            
            // Convert to corpse instead of destroying
            particle.kill(); // Sets isAlive=false, isEnergyRich=true
            
            // Reduce velocity to simulate "death"
            particle.getBody().setLinearVelocity(
                particle.getBody().getLinearVelocity().scl(0.3f)
            );
        }
    }
    
    /**
     * Process dead particles that are still bonded within living molecules.
     * Dead particles (corpses) should be ejected or consumed by the living structure.
     */
    private void processDeadParticlesInMolecules() {
        Array<Particle> deadParticlesToHandle = new Array<>();
        
        // Find all dead particles that still have bonds
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || particle.isAlive()) continue;
            
            // Check if this dead particle has any bonds
            boolean hasBonds = false;
            for (Bond bond : activeBonds) {
                if (!bond.isActive()) continue;
                if (bond.getParticleA() == particle || bond.getParticleB() == particle) {
                    hasBonds = true;
                    break;
                }
            }
            
            if (hasBonds) {
                deadParticlesToHandle.add(particle);
            }
        }
        
        // Process each dead particle
        for (Particle deadParticle : deadParticlesToHandle) {
            // Find all living particles bonded to this dead one
            Array<Particle> livingNeighbors = new Array<>();
            Array<Bond> bondsToBreak = new Array<>();
            
            for (Bond bond : activeBonds) {
                if (!bond.isActive()) continue;
                
                Particle other = null;
                if (bond.getParticleA() == deadParticle) {
                    other = bond.getParticleB();
                } else if (bond.getParticleB() == deadParticle) {
                    other = bond.getParticleA();
                }
                
                if (other != null) {
                    bondsToBreak.add(bond);
                    if (other.isAlive()) {
                        livingNeighbors.add(other);
                    }
                }
            }
            
            // If there are living neighbors, they consume the dead particle's energy
            if (livingNeighbors.size > 0) {
                float energyPerNeighbor = deadParticle.getEnergy() / livingNeighbors.size;
                for (Particle living : livingNeighbors) {
                    living.addEnergy(energyPerNeighbor);
                }
            }
            
            // Break all bonds to the dead particle (eject it)
            for (Bond bond : bondsToBreak) {
                destroyBond(bond, false, true); // Non-violent break (quiet ejection)
            }
            
            // Apply small outward force to eject the corpse
            if (livingNeighbors.size > 0) {
                Vector2 deadPos = deadParticle.getPosition();
                
                // Calculate center of living neighbors
                float cx = 0, cy = 0;
                for (Particle living : livingNeighbors) {
                    Vector2 pos = living.getPosition();
                    cx += pos.x;
                    cy += pos.y;
                }
                cx /= livingNeighbors.size;
                cy /= livingNeighbors.size;
                
                // Push corpse away from center (gentle ejection)
                float dx = deadPos.x - cx;
                float dy = deadPos.y - cy;
                float dist = (float) Math.sqrt(dx * dx + dy * dy);
                if (dist > 0.1f) {
                    float forceX = (dx / dist) * 6f; // Reduced from 20 to 6 (70% reduction)
                    float forceY = (dy / dist) * 6f;
                    deadParticle.getBody().applyForceToCenter(forceX, forceY, true);
                }
            }
        }
    }
    
    /**
     * Process corpses that enter the interior space of molecular structures.
     * When a corpse (dead particle) is detected inside a "cell" (molecule with 3+ bonded particles),
     * its energy is distributed to all living particles in that molecule and the corpse is removed.
     */
    private void processCellInteriorConsumption() {
        // Detect which corpses are inside molecular structures
        cellInteriorDetector.processCorpseConsumption(activeParticles, corpsesToRemove);
        
        // Remove consumed corpses
        for (Particle corpse : corpsesToRemove) {
            destroyParticle(corpse);
        }
    }
    
    /**
     * Process organelle absorption - GREEN (TRUE) particles entering molecular interiors
     * become trapped organelles that strengthen bonds and generate energy.
     * This simulates cellular organelles like chloroplasts.
     */
    private void processOrganelleAbsorption(IntMap<Array<CellStructure>> cellsByComponent) {
        if (cellsByComponent == null || cellsByComponent.size == 0) return;

        Array<Particle> candidateOrganelles = new Array<>();
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            if (particle.isOrganelle()) continue;
            if (particle.getType() != ParticleType.TRUE) continue;
            candidateOrganelles.add(particle);
        }

        if (candidateOrganelles.size == 0) return;

        for (Particle candidate : candidateOrganelles) {
            Vector2 candidatePos = candidate.getPosition();
            CellStructure containingCell = null;
            for (IntMap.Entry<Array<CellStructure>> entry : cellsByComponent) {
                Array<CellStructure> cells = entry.value;
                for (CellStructure cell : cells) {
                    if (cell.containsParticle(candidate)) continue;
                    if (cell.contains(candidatePos)) {
                        containingCell = cell;
                        break;
                    }
                }
                if (containingCell != null) break;
            }

            if (containingCell == null) continue;
            if (candidate.getComponentId() == containingCell.componentId) continue;

            candidate.setOrganelle(true);
            candidate.setEnergy(100f);
            candidate.setComponentId(containingCell.componentId);
            candidate.setCollisionsEnabled(false);
            organelleReplicationTimers.put(candidate, 0f);

            Body body = candidate.getBody();
            Vector2 vel = body.getLinearVelocity();
            body.setLinearVelocity(vel.x * 0.3f, vel.y * 0.3f);
            body.setAngularVelocity(0f);
        }
    }
    
    /**
     * Process organelle containment and boundary interactions.
     * Organelles collide with intact membrane bonds and otherwise drift toward their host cell center.
     */
    private void processOrganelleContainment(IntMap<Array<CellStructure>> cellsByComponent,
                                             Array<Particle> organelles,
                                             ObjectMap<Particle, CellStructure> organelleCells) {
        final float MEMBRANE_COLLISION_DISTANCE = 1.2f;
        final float INTERIOR_PULL_DISTANCE = 2.5f;
        final float INTERIOR_PULL_FORCE = 140f;
        final float OUTSIDE_RESCUE_FORCE = 260f;
        if (cellsByComponent == null || cellsByComponent.size == 0) return;
        if (organelles == null || organelles.size == 0) return;
        if (organelleCells == null || organelleCells.size == 0) return;

        Vector2 centroid = new Vector2();
        Vector2 pullDirection = new Vector2();
        ClosestBoundaryInfo boundaryInfo = new ClosestBoundaryInfo();
        for (Particle organelle : organelles) {
            CellStructure cell = organelleCells.get(organelle);
            if (cell == null) continue;

            Vector2 orgPos = organelle.getPosition();
            boolean inside = cell.contains(orgPos);

            boundaryInfo = findClosestBoundaryInfo(cell, orgPos, boundaryInfo);
            if (boundaryInfo.distanceSq == Float.MAX_VALUE) {
                continue;
            }

            float dist = (float) Math.sqrt(boundaryInfo.distanceSq);
            boolean membraneIntact = false;
            if (boundaryInfo.particleA != null && boundaryInfo.particleB != null) {
                Bond boundaryBond = findActiveBondBetween(boundaryInfo.particleA, boundaryInfo.particleB);
                membraneIntact = boundaryBond != null;
                if (membraneIntact && dist < MEMBRANE_COLLISION_DISTANCE && dist > 0.0001f) {
                    enforceOrganelleMembraneCollision(organelle,
                        boundaryInfo.particleA,
                        boundaryInfo.particleB,
                        boundaryInfo.closestPoint,
                        inside);
                }
            }

            if (inside) {
                if (dist < INTERIOR_PULL_DISTANCE) {
                    cell.getCentroid(centroid);
                    pullDirection.set(centroid).sub(orgPos);
                    float pullLen = pullDirection.len();
                    if (pullLen > 0.0001f) {
                        pullDirection.scl(INTERIOR_PULL_FORCE / pullLen);
                        organelle.getBody().applyForceToCenter(pullDirection, true);
                    }
                }
            } else {
                cell.getCentroid(centroid);
                pullDirection.set(centroid).sub(orgPos);
                float pullLen = pullDirection.len();
                if (pullLen > 0.0001f) {
                    pullDirection.scl(OUTSIDE_RESCUE_FORCE / pullLen);
                    organelle.getBody().applyForceToCenter(pullDirection, true);
                }
            }
        }
    }

    private void processOrganelleReplication(float delta,
                                             IntMap<Array<CellStructure>> cellsByComponent,
                                             Array<Particle> organelles,
                                             ObjectMap<Particle, CellStructure> organelleCells) {
        if (organelleReplicationInterval <= 0f) return;
        if (cellsByComponent == null || cellsByComponent.size == 0) return;
        if (organelles == null || organelles.size == 0) return;
        if (organelleCells == null || organelleCells.size == 0) return;

        for (Particle organelle : organelles) {
            CellStructure cell = organelleCells.get(organelle);
            if (cell == null) continue;

            float timer = organelleReplicationTimers.get(organelle, 0f) + delta;
            if (timer >= organelleReplicationInterval) {
                timer -= organelleReplicationInterval;
                replicateOrganelle(organelle, cell, organelleCells);
            }
            organelleReplicationTimers.put(organelle, timer);
        }
    }

    private void replicateOrganelle(Particle parent,
                                    CellStructure cell,
                                    ObjectMap<Particle, CellStructure> organelleCells) {
        if (cell == null) return;

        Vector2 spawnPos = new Vector2();
        cell.getCentroid(spawnPos);
        float jitterScale = 0.5f;
        spawnPos.add(MathUtils.random(-jitterScale, jitterScale), MathUtils.random(-jitterScale, jitterScale));
        if (!cell.contains(spawnPos)) {
            cell.closestPoint(spawnPos, spawnPos);
        }

        ParticleType spawnType = parent != null ? parent.getType() : ParticleType.TRUE;
        float visualRadius = parent != null ? parent.getVisualRadius() : 0.4f;
        Particle newOrganelle = spawnParticle(spawnType, spawnPos, visualRadius, 1f);
        if (newOrganelle == null) {
            return;
        }

        newOrganelle.setOrganelle(true);
        newOrganelle.setEnergy(100f);
        newOrganelle.setComponentId(cell.componentId);
    newOrganelle.setCollisionsEnabled(false);
        newOrganelle.getBody().setLinearVelocity(0f, 0f);
        newOrganelle.getBody().setAngularVelocity(0f);
        organelleReplicationTimers.put(newOrganelle, 0f);
        if (organelleCells != null) {
            organelleCells.put(newOrganelle, cell);
        }
    }

    private Bond findActiveBondBetween(Particle a, Particle b) {
        if (a == null || b == null) {
            return null;
        }
        for (Bond bond : activeBonds) {
            if (!bond.isActive()) continue;
            Particle bondA = bond.getParticleA();
            Particle bondB = bond.getParticleB();
            if ((bondA == a && bondB == b) || (bondA == b && bondB == a)) {
                return bond;
            }
        }
        return null;
    }

    private void enforceOrganelleMembraneCollision(Particle organelle,
                                                   Particle boundaryA,
                                                   Particle boundaryB,
                                                   Vector2 closestPoint,
                                                   boolean inside) {
        final float COLLISION_FORCE = 420f;
        final float COLLISION_THICKNESS = 1.0f;
        final float BOUNCE_MULTIPLIER = 1.4f;
        if (organelle == null || closestPoint == null) {
            return;
        }

        Vector2 orgPos = organelle.getPosition();
        Vector2 inwardNormal = new Vector2(orgPos).sub(closestPoint);
        float distance = inwardNormal.len();
        if (distance < 0.0001f) {
            inwardNormal.set(0f, 1f);
            distance = 0.0001f;
        } else {
            inwardNormal.scl(1f / distance);
        }

        Body body = organelle.getBody();
        Vector2 velocity = body.getLinearVelocity();
        float normalVelocity = velocity.dot(inwardNormal);

        if (normalVelocity < 0f) {
            float impulseMag = -normalVelocity * body.getMass() * BOUNCE_MULTIPLIER;
            Vector2 impulse = new Vector2(inwardNormal).scl(impulseMag);
            body.applyLinearImpulse(impulse, body.getWorldCenter(), true);
        }

        float penetration = Math.max(0f, COLLISION_THICKNESS - distance);
        if (penetration > 0f) {
            Vector2 push = new Vector2(inwardNormal).scl(COLLISION_FORCE * penetration);
            body.applyForceToCenter(push, true);

            if (boundaryA != null) {
                boundaryA.getBody().applyForceToCenter(-push.x * 0.5f, -push.y * 0.5f, true);
            }
            if (boundaryB != null) {
                boundaryB.getBody().applyForceToCenter(-push.x * 0.5f, -push.y * 0.5f, true);
            }
        }

        if (!inside) {
            // If the organelle slipped slightly outside, immediately nudge it back using the same force.
            body.applyForceToCenter(new Vector2(inwardNormal).scl(COLLISION_FORCE), true);
        }
    }
    
    /**
     * Get the closest point on a line segment to a given point.
     */
    private static Vector2 getClosestPointOnSegment(Vector2 point, Vector2 segStart, Vector2 segEnd) {
        float dx = segEnd.x - segStart.x;
        float dy = segEnd.y - segStart.y;
        float lengthSq = dx * dx + dy * dy;
        
        if (lengthSq < 0.0001f) {
            // Segment is essentially a point
            return new Vector2(segStart);
        }
        
        // Project point onto line segment
        float t = ((point.x - segStart.x) * dx + (point.y - segStart.y) * dy) / lengthSq;
        t = Math.max(0f, Math.min(1f, t)); // Clamp to segment
        
        return new Vector2(segStart.x + t * dx, segStart.y + t * dy);
    }

    private ClosestBoundaryInfo findClosestBoundaryInfo(CellStructure cell, Vector2 position, ClosestBoundaryInfo result) {
        if (result == null) {
            result = new ClosestBoundaryInfo();
        } else {
            result.reset();
        }

        if (cell == null || position == null || cell.boundaryParticles.size < 2) {
            return result;
        }

        for (int i = 0; i < cell.boundaryParticles.size; i++) {
            Particle a = cell.boundaryParticles.get(i);
            Particle b = cell.boundaryParticles.get((i + 1) % cell.boundaryParticles.size);
            Vector2 candidate = getClosestPointOnSegment(position, a.getPosition(), b.getPosition());
            float dx = position.x - candidate.x;
            float dy = position.y - candidate.y;
            float distSq = dx * dx + dy * dy;
            if (distSq < result.distanceSq) {
                result.distanceSq = distSq;
                result.closestPoint.set(candidate);
                result.particleA = a;
                result.particleB = b;
            }
        }

        return result;
    }
    
    /**
     * Break organelle bonds when their host cells separate.
     * If two organelles are no longer in the same component, their bond should break.
     */
    private void pruneOrganelleBonds(IntMap<Array<CellStructure>> cellsByComponent,
                                     ObjectMap<Particle, CellStructure> organelleCells) {
        Array<Bond> bondsToBreak = new Array<>();
        if (cellsByComponent == null || cellsByComponent.size == 0) {
            removeAllAnchorBonds();
            return;
        }
        
        for (Bond bond : activeBonds) {
            if (!bond.isActive()) continue;
            
            Particle a = bond.getParticleA();
            Particle b = bond.getParticleB();

            if (bond.getBondType() == BondType.ANCHOR) {
                Particle organelle = a.isOrganelle() ? a : (b.isOrganelle() ? b : null);
                Particle boundary = organelle == a ? b : a;
                CellStructure cell = organelle != null && organelleCells != null ? organelleCells.get(organelle) : null;
                if (organelle == null || boundary == null || cell == null || !cell.memberSet.contains(boundary)) {
                    bondsToBreak.add(bond);
                }
                continue;
            }
            
            // Check if this is an organelle-organelle bond
            if (a.isOrganelle() && b.isOrganelle()) {
                CellStructure cellA = organelleCells != null ? organelleCells.get(a) : null;
                CellStructure cellB = organelleCells != null ? organelleCells.get(b) : null;
                if (cellA == null || cellB == null || cellA != cellB) {
                    bondsToBreak.add(bond);
                }
            }
            
            // Also break bonds between organelles and non-organelles
            // Organelles should NEVER bond with regular particles
            if ((a.isOrganelle() && !b.isOrganelle()) || (!a.isOrganelle() && b.isOrganelle())) {
                bondsToBreak.add(bond);
            }
        }
        
        for (Bond bond : bondsToBreak) {
            destroyBond(bond, false, true); // Non-violent break
        }
    }
    
    /**
     * Check if a horizontal ray from point intersects the line segment.
     */
    private static boolean rayIntersectsSegment(Vector2 point, Vector2 segA, Vector2 segB) {
        if (segA.y > segB.y) {
            Vector2 temp = segA;
            segA = segB;
            segB = temp;
        }
        
        if (point.y < segA.y || point.y >= segB.y) return false;
        if (point.x >= Math.max(segA.x, segB.x)) return false;
        if (point.x < Math.min(segA.x, segB.x)) return true;
        
        float xIntersection = segA.x + (point.y - segA.y) / (segB.y - segA.y) * (segB.x - segA.x);
        return point.x < xIntersection;
    }

    /**
     * Detect bonds that cross within the same molecule and break them with repulsive force.
     */
    private void detectAndBreakCrossingBonds() {
        Array<Bond> bondsToBreak = new Array<>();

        if (activeBonds.size == 0) return;

        float cellSize = spatialGrid.getCellSize();
        LongMap<BondBucket> bucketMap = new LongMap<>();
        Array<BondBucket> bucketList = new Array<>();

        // Bucket bonds by the cell containing their midpoint
        for (Bond bond : activeBonds) {
            if (!bond.isActive()) continue;
            if (bond.getBondType() == BondType.ANCHOR) continue;
            Particle a = bond.getParticleA();
            Particle b = bond.getParticleB();
            if (a.getComponentId() != b.getComponentId()) continue; // Skip inter-component bonds

            Vector2 posA = a.getPosition();
            Vector2 posB = b.getPosition();
            float midX = (posA.x + posB.x) * 0.5f;
            float midY = (posA.y + posB.y) * 0.5f;
            int cellX = (int) Math.floor(midX / cellSize);
            int cellY = (int) Math.floor(midY / cellSize);
            long key = packCellKey(cellX, cellY);

            BondBucket bucket = bucketMap.get(key);
            if (bucket == null) {
                bucket = new BondBucket(cellX, cellY);
                bucketMap.put(key, bucket);
                bucketList.add(bucket);
            }
            bucket.bonds.add(bond);
        }

        // Compare bonds within each bucket and neighboring buckets
        for (BondBucket bucket : bucketList) {
            checkBucketForCrossings(bucket, bucket, bondsToBreak, true);

            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;
                    int neighborX = bucket.cellX + dx;
                    int neighborY = bucket.cellY + dy;
                    if (!shouldProcessBucketPair(bucket.cellX, bucket.cellY, neighborX, neighborY)) continue;
                    long neighborKey = packCellKey(neighborX, neighborY);
                    BondBucket neighbor = bucketMap.get(neighborKey);
                    if (neighbor == null) continue;
                    checkBucketForCrossings(bucket, neighbor, bondsToBreak, false);
                }
            }
        }

        // Break all crossing bonds (violent reactions applied automatically)
        for (Bond bond : bondsToBreak) {
            if (bond.isActive()) {
                destroyBond(bond, true);
            }
        }
    }
    
    private void checkBucketForCrossings(BondBucket bucketA, BondBucket bucketB, Array<Bond> bondsToBreak, boolean sameBucket) {
        Array<Bond> bondsA = bucketA.bonds;
        Array<Bond> bondsB = bucketB.bonds;
        if (sameBucket) {
            for (int i = 0; i < bondsA.size; i++) {
                Bond bondA = bondsA.get(i);
                if (!bondA.isActive()) continue;
                if (bondA.getBondType() == BondType.ANCHOR) continue;
                for (int j = i + 1; j < bondsA.size; j++) {
                    Bond bondB = bondsA.get(j);
                    if (!bondB.isActive()) continue;
                    if (bondB.getBondType() == BondType.ANCHOR) continue;
                    if (!areBondsInSameComponent(bondA, bondB)) continue;
                    if (bondA.intersects(bondB)) {
                        bondsToBreak.add(bondA);
                        bondsToBreak.add(bondB);
                    }
                }
            }
        } else {
            for (int i = 0; i < bondsA.size; i++) {
                Bond bondA = bondsA.get(i);
                if (!bondA.isActive()) continue;
                if (bondA.getBondType() == BondType.ANCHOR) continue;
                for (int j = 0; j < bondsB.size; j++) {
                    Bond bondB = bondsB.get(j);
                    if (!bondB.isActive()) continue;
                    if (bondB.getBondType() == BondType.ANCHOR) continue;
                    if (!areBondsInSameComponent(bondA, bondB)) continue;
                    if (bondA.intersects(bondB)) {
                        bondsToBreak.add(bondA);
                        bondsToBreak.add(bondB);
                    }
                }
            }
        }
    }

    private boolean areBondsInSameComponent(Bond a, Bond b) {
        return a.getParticleA().getComponentId() == b.getParticleA().getComponentId();
    }

    private boolean shouldProcessBucketPair(int cellAX, int cellAY, int cellBX, int cellBY) {
        if (cellBX > cellAX) return true;
        if (cellBX == cellAX && cellBY > cellAY) return true;
        return false;
    }

    private long packCellKey(int cellX, int cellY) {
        return (((long) cellX) << 32) ^ (cellY & 0xffffffffL);
    }

    /**
     * Apply rotational shear forces to break peripheral bonds in fast-spinning molecules.
     * When angular velocity exceeds threshold, centrifugal forces tear molecules apart.
     */
    private void applyRotationalShear() {
        final float ROTATION_THRESHOLD = 2.0f; // rad/s - angular velocity threshold
        final float CENTRIFUGAL_FORCE = 100f; // Force multiplier
        
        // Group particles by component to analyze molecular rotation
        com.badlogic.gdx.utils.IntMap<Array<Particle>> componentGroups = new com.badlogic.gdx.utils.IntMap<>();
        for (Particle particle : activeParticles) {
            if (!particle.isActive() || !particle.isAlive()) continue;
            
            int componentId = particle.getComponentId();
            Array<Particle> group = componentGroups.get(componentId);
            if (group == null) {
                group = new Array<>();
                componentGroups.put(componentId, group);
            }
            group.add(particle);
        }
        
        // Check each molecule for excessive rotation
        for (com.badlogic.gdx.utils.IntMap.Entry<Array<Particle>> entry : componentGroups) {
            Array<Particle> molecule = entry.value;
            
            if (molecule.size < 3) continue; // Too small to experience significant shear
            
            // Calculate average angular velocity
            float avgAngularVel = 0f;
            for (Particle p : molecule) {
                avgAngularVel += Math.abs(p.getBody().getAngularVelocity());
            }
            avgAngularVel /= molecule.size;
            
            if (avgAngularVel < ROTATION_THRESHOLD) continue;
            
            // Calculate center of mass
            float cx = 0, cy = 0;
            for (Particle p : molecule) {
                Vector2 pos = p.getPosition();
                cx += pos.x;
                cy += pos.y;
            }
            cx /= molecule.size;
            cy /= molecule.size;
            
            // Apply centrifugal force to peripheral particles
            // Find and break bonds on particles far from center
            Array<Bond> peripheralBonds = new Array<>();
            
            for (Bond bond : activeBonds) {
                if (!bond.isActive()) continue;
                if (bond.getBondType() == BondType.ANCHOR) continue;
                
                Particle a = bond.getParticleA();
                Particle b = bond.getParticleB();
                
                // Check if both particles are in this molecule
                if (!molecule.contains(a, true) || !molecule.contains(b, true)) continue;
                
                // Calculate distance from center for both particles
                Vector2 posA = a.getPosition();
                Vector2 posB = b.getPosition();
                float distA = (float) Math.sqrt((posA.x - cx) * (posA.x - cx) + (posA.y - cy) * (posA.y - cy));
                float distB = (float) Math.sqrt((posB.x - cx) * (posB.x - cx) + (posB.y - cy) * (posB.y - cy));
                
                // If at least one particle is peripheral (far from center)
                float maxDist = Math.max(distA, distB);
                if (maxDist > 3.0f) { // Peripheral threshold
                    // Apply outward centrifugal force
                    float forceX = (posA.x - cx) * CENTRIFUGAL_FORCE * avgAngularVel;
                    float forceY = (posA.y - cy) * CENTRIFUGAL_FORCE * avgAngularVel;
                    a.getBody().applyForceToCenter(forceX, forceY, true);
                    
                    forceX = (posB.x - cx) * CENTRIFUGAL_FORCE * avgAngularVel;
                    forceY = (posB.y - cy) * CENTRIFUGAL_FORCE * avgAngularVel;
                    b.getBody().applyForceToCenter(forceX, forceY, true);
                    
                    // Mark bond for potential breaking if rotation is extreme
                    if (avgAngularVel > ROTATION_THRESHOLD * 2f) {
                        peripheralBonds.add(bond);
                    }
                }
            }
            
            // Break peripheral bonds under extreme rotation
            for (Bond bond : peripheralBonds) {
                if (bond.isActive() && Math.random() < 0.3f) { // 30% chance per check
                    destroyBond(bond, true); // Violent break
                }
            }
        }
    }

    public void clearAllBonds() {
        for (int i = activeBonds.size - 1; i >= 0; i--) {
            destroyBond(activeBonds.get(i), false, true);  // No violent reactions on cleanup
        }
    }

    public void clearAllParticles() {
        for (int i = activeParticles.size - 1; i >= 0; i--) {
            destroyParticle(activeParticles.get(i));
        }
    }

    /**
     * Rebuild union-find from scratch based on current bonds.
     * Call this periodically or after many bond breaks to ensure accurate component tracking.
     */
    public void rebuildUnionFind() {
        unionFind.rebuild(activeParticles, activeBonds);
        
        // Update component IDs in particles
        for (Particle particle : activeParticles) {
            if (particle.isActive()) {
                int componentId = unionFind.find(particle.hashCode());
                particle.setComponentId(componentId);
            }
        }
        componentsNeedRebuild = false;
    }

    @Override
    public void dispose() {
        clearAllBonds();
        clearAllParticles();
        world.dispose();
    }
}
