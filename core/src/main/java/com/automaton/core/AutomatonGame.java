package com.automaton.core;

import com.automaton.core.physics.Bond;
import com.automaton.core.physics.BondType;
import com.automaton.core.physics.Particle;
import com.automaton.core.physics.ParticleType;
import com.automaton.core.physics.PhysicsWorld;
import com.automaton.core.physics.ReactionManager;
import com.automaton.core.render.BondRenderer;
import com.automaton.core.render.ParticleRenderer;
import com.badlogic.gdx.ApplicationAdapter;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.scenes.scene2d.Actor;
import com.badlogic.gdx.scenes.scene2d.Stage;
import com.badlogic.gdx.scenes.scene2d.ui.Label;
import com.badlogic.gdx.scenes.scene2d.ui.Skin;
import com.badlogic.gdx.scenes.scene2d.ui.Slider;
import com.badlogic.gdx.scenes.scene2d.ui.Table;
import com.badlogic.gdx.scenes.scene2d.utils.ChangeListener;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import java.util.Random;

public class AutomatonGame extends ApplicationAdapter {
    private static final float WORLD_WIDTH = 100f;  // Increased from 50f (4x area)
    private static final float WORLD_HEIGHT = 56f;  // Increased from 28f (4x area)
    private static final float TIME_STEP = 1f / 60f;
    private static final int VELOCITY_ITERATIONS = 6;
    private static final int POSITION_ITERATIONS = 2;

    private final Vector2 tmpVec = new Vector2();
    private final Random typeRandom = new Random(42L);  // For random type assignment

    private PhysicsWorld physicsWorld;
    private ReactionManager reactionManager;
    private ParticleRenderer particleRenderer;
    private BondRenderer bondRenderer;
    private OrthographicCamera camera;
    private CameraController cameraController;
    private float accumulator;
    private boolean enableReactions = true; // Enabled by default
    
    // UI
    private Stage uiStage;
    private Skin skin;
    private float yellowFriction = 0.0f;  // Friction for XOR gate particles (yellow colored)
    private boolean uiVisible = true;  // Toggle with 'H' key
    private float startingVelocityRange = 24f; // Initial velocity range for particles

    @Override
    public void create() {
        physicsWorld = new PhysicsWorld(new Vector2(0f, 0f), 1024);
        createWorldBoundaries();
        
        reactionManager = new ReactionManager(physicsWorld, 42L);
        
        // Configure reaction parameters - higher rates for visible interactions
        reactionManager.setBondFormationRadius(2.5f);  // Slightly tighter collision range
        reactionManager.setBondFormationProbability(0.15f);  // Increased from 0.02 for more reactions
        reactionManager.setBondBreakProbability(0.002f);  // Increased from 0.0005 for more dynamics
        
        particleRenderer = new ParticleRenderer();
        particleRenderer.setPhysicsWorld(physicsWorld);
        bondRenderer = new BondRenderer();
        camera = new OrthographicCamera(WORLD_WIDTH, WORLD_HEIGHT);
        camera.position.set(0f, 0f, 0f);
        camera.update();

        cameraController = new CameraController(camera);
        
        // Create UI
        createUI();
        
        // Set up input multiplexer to handle both UI and camera
        InputMultiplexer multiplexer = new InputMultiplexer();
        multiplexer.addProcessor(uiStage);
        multiplexer.addProcessor(cameraController);
        Gdx.input.setInputProcessor(multiplexer);

        spawnDemoMolecules();
    }
    
    private void createUI() {
        uiStage = new Stage(new ScreenViewport());
        
        // Create a simple skin programmatically
        skin = createSkin();
        
        Table table = new Table();
        table.setFillParent(true);
        table.top().left();
        table.pad(10);
        
        // XOR gate friction slider (XOR is yellow colored)
        Label frictionLabel = new Label("XOR Friction: 0.00", skin);
        Slider frictionSlider = new Slider(0f, 5f, 0.1f, false, skin);
        frictionSlider.setValue(yellowFriction);
        
        frictionSlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                yellowFriction = frictionSlider.getValue();
                frictionLabel.setText(String.format("XOR Friction: %.2f", yellowFriction));
                applyYellowFriction();
            }
        });
        
        table.add(frictionLabel).padBottom(5).row();
        table.add(frictionSlider).width(200).row();
        
        // Reaction Violence slider
        table.add(new Label("", skin)).padTop(15).row(); // Spacer
        Label violenceLabel = new Label("Reaction Violence: 1.00x", skin);
        Slider violenceSlider = new Slider(0f, 5f, 0.1f, false, skin);
        violenceSlider.setValue(physicsWorld.getReactionViolenceMultiplier());
        
        violenceSlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                float violence = violenceSlider.getValue();
                physicsWorld.setReactionViolenceMultiplier(violence);
                violenceLabel.setText(String.format("Reaction Violence: %.2fx", violence));
            }
        });
        
        table.add(violenceLabel).padBottom(5).row();
        table.add(violenceSlider).width(200).row();
        
        // Energy Decay Rate slider
        table.add(new Label("", skin)).padTop(15).row(); // Spacer
        Label energyDecayLabel = new Label("Energy Decay: 0.10/s", skin);
        Slider energyDecaySlider = new Slider(0f, 2f, 0.05f, false, skin);
        energyDecaySlider.setValue(physicsWorld.getEnergyDecayRate());
        
        energyDecaySlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                float rate = energyDecaySlider.getValue();
                physicsWorld.setEnergyDecayRate(rate);
                energyDecayLabel.setText(String.format("Energy Decay: %.2f/s", rate));
            }
        });
        
        table.add(energyDecayLabel).padBottom(5).row();
        table.add(energyDecaySlider).width(200).row();
        
        // Bond Energy Cost slider
        table.add(new Label("", skin)).padTop(15).row(); // Spacer
        Label bondCostLabel = new Label("Bond Cost: 0.50/s", skin);
        Slider bondCostSlider = new Slider(0f, 5f, 0.1f, false, skin);
        bondCostSlider.setValue(physicsWorld.getBondEnergyCost());
        
        bondCostSlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                float cost = bondCostSlider.getValue();
                physicsWorld.setBondEnergyCost(cost);
                bondCostLabel.setText(String.format("Bond Cost: %.2f/s", cost));
            }
        });
        
        table.add(bondCostLabel).padBottom(5).row();
        table.add(bondCostSlider).width(200).row();
        
        // Metabolism Rate slider (bond-generated food)
        table.add(new Label("", skin)).padTop(15).row(); // Spacer
        Label metabolismLabel = new Label("Metabolism: 2.00/s", skin);
        Slider metabolismSlider = new Slider(0f, 10f, 0.5f, false, skin);
        metabolismSlider.setValue(physicsWorld.getFoodSpawnRate());
        
        metabolismSlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                float rate = metabolismSlider.getValue();
                physicsWorld.setFoodSpawnRate(rate);
                metabolismLabel.setText(String.format("Metabolism: %.2f/s", rate));
            }
        });
        
        table.add(metabolismLabel).padBottom(5).row();
        table.add(metabolismSlider).width(200).row();
        
        uiStage.addActor(table);
    }
    
    /**
     * Create a simple UI skin programmatically without external files.
     */
    private Skin createSkin() {
        Skin skin = new Skin();
        
        // Create a simple 1x1 white texture for UI elements
        com.badlogic.gdx.graphics.Pixmap pixmap = new com.badlogic.gdx.graphics.Pixmap(1, 1, com.badlogic.gdx.graphics.Pixmap.Format.RGBA8888);
        pixmap.setColor(com.badlogic.gdx.graphics.Color.WHITE);
        pixmap.fill();
        skin.add("white", new com.badlogic.gdx.graphics.Texture(pixmap));
        pixmap.dispose();
        
        // Create BitmapFont
        com.badlogic.gdx.graphics.g2d.BitmapFont font = new com.badlogic.gdx.graphics.g2d.BitmapFont();
        font.getData().setScale(1.2f);
        skin.add("default", font);
        
        // Label style
        Label.LabelStyle labelStyle = new Label.LabelStyle();
        labelStyle.font = font;
        labelStyle.fontColor = com.badlogic.gdx.graphics.Color.WHITE;
        skin.add("default", labelStyle);
        
        // Slider style
        Slider.SliderStyle sliderStyle = new Slider.SliderStyle();
        com.badlogic.gdx.scenes.scene2d.utils.TextureRegionDrawable whiteDrawable = new com.badlogic.gdx.scenes.scene2d.utils.TextureRegionDrawable(
            new com.badlogic.gdx.graphics.g2d.TextureRegion(skin.get("white", com.badlogic.gdx.graphics.Texture.class))
        );
        
        // Background
        com.badlogic.gdx.scenes.scene2d.utils.Drawable background = whiteDrawable.tint(new com.badlogic.gdx.graphics.Color(0.3f, 0.3f, 0.3f, 1f));
        sliderStyle.background = background;
        
        // Knob
        com.badlogic.gdx.scenes.scene2d.utils.Drawable knob = whiteDrawable.tint(new com.badlogic.gdx.graphics.Color(1f, 0.9f, 0.2f, 1f));
        sliderStyle.knob = knob;
        sliderStyle.knob.setMinWidth(12);
        sliderStyle.knob.setMinHeight(20);
        
        skin.add("default-horizontal", sliderStyle);
        
        return skin;
    }
    
    /**
     * Apply friction setting to all XOR gate particles.
     */
    private void applyYellowFriction() {
        Array<Particle> particles = physicsWorld.getActiveParticles();
        for (Particle particle : particles) {
            if (particle.getType() == ParticleType.XOR) {
                particle.getBody().setLinearDamping(yellowFriction);
            }
        }
    }
    
    /**
     * Restart the simulation by clearing all particles and bonds, then spawning new molecules.
     */
    private void restartSimulation() {
        System.out.println("Restarting simulation...");
        
        // Clear all existing particles and bonds
        Array<Particle> particles = physicsWorld.getActiveParticles();
        Array<Bond> bonds = physicsWorld.getActiveBonds();
        
        // Copy to temporary arrays to avoid concurrent modification
        Array<Particle> particlesToRemove = new Array<>(particles);
        Array<Bond> bondsToRemove = new Array<>(bonds);
        
        for (Bond bond : bondsToRemove) {
            physicsWorld.destroyBond(bond, false);  // No violent reactions on restart
        }
        
        for (Particle particle : particlesToRemove) {
            physicsWorld.destroyParticle(particle);
        }
        
        // Reset accumulator
        accumulator = 0f;
        
        // Reseed random for variety
        typeRandom.setSeed(System.currentTimeMillis());
        
        // Spawn new demo molecules
        spawnDemoMolecules();
        
        System.out.println("Simulation restarted!");
    }

    @Override
    public void render() {
        // Restart simulation with 'R' key
        if (Gdx.input.isKeyJustPressed(Input.Keys.R)) {
            restartSimulation();
        }
        
        // Toggle UI visibility with 'H' key
        if (Gdx.input.isKeyJustPressed(Input.Keys.H)) {
            uiVisible = !uiVisible;
            System.out.println("UI " + (uiVisible ? "visible" : "hidden"));
        }
        
        float delta = Gdx.graphics.getDeltaTime();
        accumulator += delta;
        while (accumulator >= TIME_STEP) {
            physicsWorld.step(TIME_STEP, VELOCITY_ITERATIONS, POSITION_ITERATIONS);
            
            // Process reactions if enabled
            if (enableReactions) {
                reactionManager.processReactions();
            }
            
            accumulator -= TIME_STEP;
        }

        Gdx.gl.glClearColor(0f, 0f, 0f, 1f);
        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

        bondRenderer.render(physicsWorld.getActiveBonds(), camera);
        particleRenderer.render(physicsWorld.getActiveParticles(), camera);
        
        // Render UI on top if visible
        if (uiVisible) {
            uiStage.act(delta);
            uiStage.draw();
        }
    }
    
    @Override
    public void resize(int width, int height) {
        uiStage.getViewport().update(width, height, true);
    }

    @Override
    public void dispose() {
        bondRenderer.dispose();
        particleRenderer.dispose();
        physicsWorld.dispose();
        uiStage.dispose();
        skin.dispose();
    }
    
    /**
     * Get the BondProperties instance for UI slider control.
     * All bond multiplier fields are public and can be directly modified.
     * Example: getBondProperties().trueTrueLengthMult = 2.0f;
     */
    public com.automaton.core.physics.BondProperties getBondProperties() {
        return physicsWorld.getBondProperties();
    }

    private void spawnDemoMolecules() {
        // Spawn varied molecules (mols) with random initial velocities
        // Ensure minimum spacing between molecules
        Array<Vector2> occupiedPositions = new Array<>();
        float minSpacing = 5f; // Minimum distance between molecule centers
        
        // Singles (10 particles)
        for (int i = 0; i < 10; i++) {
            Vector2 pos = findFreePosition(occupiedPositions, minSpacing);
            spawnMolecule(1, pos.x, pos.y);
            occupiedPositions.add(pos);
        }
        
        // Pairs (8 molecules)
        for (int i = 0; i < 8; i++) {
            Vector2 pos = findFreePosition(occupiedPositions, minSpacing);
            spawnAngleMolecule(2, pos.x, pos.y);
            occupiedPositions.add(pos);
        }
        
        // Triplets (6 molecules) - varied angles
        for (int i = 0; i < 6; i++) {
            Vector2 pos = findFreePosition(occupiedPositions, minSpacing);
            spawnAngleMolecule(3, pos.x, pos.y);
            occupiedPositions.add(pos);
        }
        
        // Triangles (4 molecules)
        for (int i = 0; i < 4; i++) {
            Vector2 pos = findFreePosition(occupiedPositions, minSpacing);
            spawnTriangleMol(pos.x, pos.y);
            occupiedPositions.add(pos);
        }
        
        // Branched structures (3 molecules, 4-5 nodes)
        for (int i = 0; i < 3; i++) {
            Vector2 pos = findFreePosition(occupiedPositions, minSpacing);
            spawnBranchedMol(MathUtils.random(4, 6), pos.x, pos.y);
            occupiedPositions.add(pos);
        }
        
        // Large irregular mols (2 molecules, 8-10 nodes)
        for (int i = 0; i < 2; i++) {
            Vector2 pos = findFreePosition(occupiedPositions, minSpacing);
            spawnBranchedMol(MathUtils.random(8, 11), pos.x, pos.y);
            occupiedPositions.add(pos);
        }
    }

    /**
     * Find a position with minimum spacing from existing molecules.
     */
    private Vector2 findFreePosition(Array<Vector2> occupied, float minSpacing) {
        int maxAttempts = 50;
        float spawnRangeX = WORLD_WIDTH * 0.4f;  // Use 80% of world width
        float spawnRangeY = WORLD_HEIGHT * 0.4f;  // Use 80% of world height
        
        for (int attempt = 0; attempt < maxAttempts; attempt++) {
            float x = MathUtils.random(-spawnRangeX, spawnRangeX);
            float y = MathUtils.random(-spawnRangeY, spawnRangeY);
            
            boolean tooClose = false;
            for (Vector2 pos : occupied) {
                float distSq = (x - pos.x) * (x - pos.x) + (y - pos.y) * (y - pos.y);
                if (distSq < minSpacing * minSpacing) {
                    tooClose = true;
                    break;
                }
            }
            
            if (!tooClose) {
                return new Vector2(x, y);
            }
        }
        // Fallback if can't find free space
        return new Vector2(MathUtils.random(-spawnRangeX, spawnRangeX), MathUtils.random(-spawnRangeY, spawnRangeY));
    }

    /**
     * Get a random particle type (logic states and gates).
     */
    private ParticleType randomParticleType() {
        ParticleType[] types = ParticleType.values();
        return types[typeRandom.nextInt(types.length)];
    }
    
    /**
     * Get a random logic state (TRUE or FALSE).
     */
    private ParticleType randomLogicState() {
        return typeRandom.nextBoolean() ? ParticleType.TRUE : ParticleType.FALSE;
    }
    
    /**
     * Get a random logic gate.
     */
    private ParticleType randomGate() {
        ParticleType[] gates = {
            ParticleType.AND, ParticleType.OR, ParticleType.NOT,
            ParticleType.NAND, ParticleType.NOR, ParticleType.XOR, ParticleType.XNOR
        };
        return gates[typeRandom.nextInt(gates.length)];
    }

    /**
     * Spawn a single particle (for singles).
     */
    private void spawnMolecule(int nodeCount, float centerX, float centerY) {
        if (nodeCount != 1) return;
        
        // Random velocity
        float vx = MathUtils.random(-startingVelocityRange, startingVelocityRange);
        float vy = MathUtils.random(-startingVelocityRange, startingVelocityRange);

        tmpVec.set(centerX, centerY);
        ParticleType type = randomParticleType();
        Particle particle = physicsWorld.spawnParticle(type, tmpVec, 0.4f, 1f);
        particle.getBody().setLinearVelocity(vx, vy);
        
        // Apply XOR gate friction if applicable (XOR is yellow)
        if (type == ParticleType.XOR) {
            particle.getBody().setLinearDamping(yellowFriction);
        }
    }

    /**
     * Spawn a molecule with angular/diagonal bonds (not straight lines).
     * Creates zigzag or angular structures.
     */
    private void spawnAngleMolecule(int nodeCount, float centerX, float centerY) {
        if (nodeCount < 2) return;
        
        float bondLength = 1.5f;  // Increased from 0.8f for better visibility
        float angleVariation = MathUtils.random(30f, 60f) * MathUtils.degreesToRadians;
        
        // Random velocity for whole molecule
        float vx = MathUtils.random(-6f, 6f);
        float vy = MathUtils.random(-6f, 6f);
        
        Particle[] nodes = new Particle[nodeCount];
        float currentAngle = MathUtils.random(0f, MathUtils.PI2);
        
        // First node at center
        tmpVec.set(centerX, centerY);
        ParticleType type0 = randomParticleType();
        nodes[0] = physicsWorld.spawnParticle(type0, tmpVec, 0.4f, 1f);
        nodes[0].getBody().setLinearVelocity(vx, vy);
        if (type0 == ParticleType.XOR) {
            nodes[0].getBody().setLinearDamping(yellowFriction);
        }
        
        // Place remaining nodes at angles
        float x = centerX;
        float y = centerY;
        
        for (int i = 1; i < nodeCount; i++) {
            // Add angle variation to create zigzag
            currentAngle += MathUtils.random(-angleVariation, angleVariation);
            x += MathUtils.cos(currentAngle) * bondLength;
            y += MathUtils.sin(currentAngle) * bondLength;
            
            tmpVec.set(x, y);
            ParticleType type = randomParticleType();
            nodes[i] = physicsWorld.spawnParticle(type, tmpVec, 0.4f, 1f);
            nodes[i].getBody().setLinearVelocity(vx, vy);
            if (type == ParticleType.XOR) {
                nodes[i].getBody().setLinearDamping(yellowFriction);
            }
        }
        
        // Bond adjacent nodes
        for (int i = 0; i < nodeCount - 1; i++) {
            physicsWorld.createBond(nodes[i], nodes[i + 1], 
                BondType.ELASTIC, 80f, 3f, 200f);
        }
    }

    /**
     * Spawn a branched molecule structure (tree-like).
     */
    private void spawnBranchedMol(int nodeCount, float centerX, float centerY) {
        if (nodeCount < 3) {
            spawnAngleMolecule(nodeCount, centerX, centerY);
            return;
        }
        
        float bondLength = 1.5f;  // Increased from 0.8f for better visibility
        
        // Random velocity for whole molecule
        float vx = MathUtils.random(-6f, 6f);
        float vy = MathUtils.random(-6f, 6f);
        
        Array<Particle> nodes = new Array<>();
        
        // Central node
        tmpVec.set(centerX, centerY);
        ParticleType centerType = randomParticleType();
        Particle center = physicsWorld.spawnParticle(centerType, tmpVec, 0.4f, 1f);
        center.getBody().setLinearVelocity(vx, vy);
        if (centerType == ParticleType.XOR) {
            center.getBody().setLinearDamping(yellowFriction);
        }
        nodes.add(center);
        
        // Create branches radiating from center at various angles
        int branches = Math.min(nodeCount - 1, MathUtils.random(2, 4));
        int nodesPerBranch = (nodeCount - 1) / branches;
        int remaining = nodeCount - 1;
        
        float baseAngle = MathUtils.random(0f, MathUtils.PI2);
        float angleStep = MathUtils.PI2 / branches;
        
        for (int b = 0; b < branches && remaining > 0; b++) {
            float branchAngle = baseAngle + b * angleStep + MathUtils.random(-0.3f, 0.3f);
            int branchNodes = Math.min(nodesPerBranch + (remaining > nodesPerBranch * (branches - b - 1) ? 1 : 0), remaining);
            
            Particle prev = center;
            float x = centerX;
            float y = centerY;
            
            for (int n = 0; n < branchNodes; n++) {
                // Add some angle jitter
                float angle = branchAngle + MathUtils.random(-0.2f, 0.2f);
                x += MathUtils.cos(angle) * bondLength;
                y += MathUtils.sin(angle) * bondLength;
                
                tmpVec.set(x, y);
                ParticleType nodeType = randomParticleType();
                Particle node = physicsWorld.spawnParticle(nodeType, tmpVec, 0.4f, 1f);
                node.getBody().setLinearVelocity(vx, vy);
                if (nodeType == ParticleType.XOR) {
                    node.getBody().setLinearDamping(yellowFriction);
                }
                nodes.add(node);
                
                // Bond to previous in branch
                physicsWorld.createBond(prev, node, BondType.ELASTIC, 80f, 3f, 200f);
                prev = node;
                remaining--;
            }
        }
    }

    /**
     * Spawn a triangular molecule (3 nodes in triangle formation).
     */
    private void spawnTriangleMol(float centerX, float centerY) {
        float radius = 1.0f;  // Increased from 0.6f for better spacing
        
        // Random velocity for whole molecule
        float vx = MathUtils.random(-6f, 6f);
        float vy = MathUtils.random(-6f, 6f);
        
        Particle[] nodes = new Particle[3];
        
        for (int i = 0; i < 3; i++) {
            float angle = i * MathUtils.PI2 / 3f;
            tmpVec.set(centerX + MathUtils.cos(angle) * radius, 
                      centerY + MathUtils.sin(angle) * radius);
            ParticleType type = randomParticleType();
            nodes[i] = physicsWorld.spawnParticle(type, tmpVec, 0.4f, 1f);
            nodes[i].getBody().setLinearVelocity(vx, vy);
            if (type == ParticleType.XOR) {
                nodes[i].getBody().setLinearDamping(yellowFriction);
            }
        }
        
        // Bond all three nodes
        physicsWorld.createBond(nodes[0], nodes[1], BondType.ELASTIC, 80f, 3f, 200f);
        physicsWorld.createBond(nodes[1], nodes[2], BondType.ELASTIC, 80f, 3f, 200f);
        physicsWorld.createBond(nodes[2], nodes[0], BondType.ELASTIC, 80f, 3f, 200f);
    }

    /**
     * Create Box2D static bodies as walls around the world boundaries.
     */
    private void createWorldBoundaries() {
        float halfWidth = WORLD_WIDTH / 2f;
        float halfHeight = WORLD_HEIGHT / 2f;
        float wallThickness = 1f;
        
        // Import Box2D classes for boundary creation
        com.badlogic.gdx.physics.box2d.World world = physicsWorld.getWorld();
        com.badlogic.gdx.physics.box2d.BodyDef bodyDef = new com.badlogic.gdx.physics.box2d.BodyDef();
        bodyDef.type = com.badlogic.gdx.physics.box2d.BodyDef.BodyType.StaticBody;
        
        // Bottom wall
        bodyDef.position.set(0, -halfHeight);
        com.badlogic.gdx.physics.box2d.Body bottomWall = world.createBody(bodyDef);
        com.badlogic.gdx.physics.box2d.PolygonShape bottomBox = new com.badlogic.gdx.physics.box2d.PolygonShape();
        bottomBox.setAsBox(halfWidth + wallThickness, wallThickness);
        bottomWall.createFixture(bottomBox, 0f);
        bottomBox.dispose();
        
        // Top wall
        bodyDef.position.set(0, halfHeight);
        com.badlogic.gdx.physics.box2d.Body topWall = world.createBody(bodyDef);
        com.badlogic.gdx.physics.box2d.PolygonShape topBox = new com.badlogic.gdx.physics.box2d.PolygonShape();
        topBox.setAsBox(halfWidth + wallThickness, wallThickness);
        topWall.createFixture(topBox, 0f);
        topBox.dispose();
        
        // Left wall
        bodyDef.position.set(-halfWidth, 0);
        com.badlogic.gdx.physics.box2d.Body leftWall = world.createBody(bodyDef);
        com.badlogic.gdx.physics.box2d.PolygonShape leftBox = new com.badlogic.gdx.physics.box2d.PolygonShape();
        leftBox.setAsBox(wallThickness, halfHeight + wallThickness);
        leftWall.createFixture(leftBox, 0f);
        leftBox.dispose();
        
        // Right wall
        bodyDef.position.set(halfWidth, 0);
        com.badlogic.gdx.physics.box2d.Body rightWall = world.createBody(bodyDef);
        com.badlogic.gdx.physics.box2d.PolygonShape rightBox = new com.badlogic.gdx.physics.box2d.PolygonShape();
        rightBox.setAsBox(wallThickness, halfHeight + wallThickness);
        rightWall.createFixture(rightBox, 0f);
        rightBox.dispose();
    }
}
