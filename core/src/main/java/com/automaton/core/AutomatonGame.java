package com.automaton.core;

import com.automaton.core.physics.Bond;
import com.automaton.core.physics.BondType;
import com.automaton.core.physics.Particle;
import com.automaton.core.physics.ParticleType;
import com.automaton.core.physics.PhysicsWorld;
import com.automaton.core.physics.ReactionManager;
import com.automaton.core.render.BondRenderer;
import com.automaton.core.render.MovingEnergyFieldRenderer;
import com.automaton.core.render.ParticleRenderer;
import com.badlogic.gdx.ApplicationAdapter;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.scenes.scene2d.Actor;
import com.badlogic.gdx.scenes.scene2d.InputEvent;
import com.badlogic.gdx.scenes.scene2d.Stage;
import com.badlogic.gdx.scenes.scene2d.Touchable;
import com.badlogic.gdx.scenes.scene2d.ui.CheckBox;
import com.badlogic.gdx.scenes.scene2d.ui.Dialog;
import com.badlogic.gdx.scenes.scene2d.ui.Image;
import com.badlogic.gdx.scenes.scene2d.ui.Label;
import com.badlogic.gdx.scenes.scene2d.ui.ScrollPane;
import com.badlogic.gdx.scenes.scene2d.ui.Skin;
import com.badlogic.gdx.scenes.scene2d.ui.Slider;
import com.badlogic.gdx.scenes.scene2d.ui.Table;
import com.badlogic.gdx.scenes.scene2d.ui.TextTooltip;
import com.badlogic.gdx.scenes.scene2d.ui.TextButton;
import com.badlogic.gdx.scenes.scene2d.utils.ChangeListener;
import com.badlogic.gdx.scenes.scene2d.utils.ClickListener;
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
    private MovingEnergyFieldRenderer movingEnergyFieldRenderer;
    private OrthographicCamera camera;
    private CameraController cameraController;
    private float accumulator;
    private float totalTime; // Total elapsed time for animations
    private boolean enableReactions = true; // Enabled by default
    
    // World boundaries (tracked for dynamic resizing)
    private com.badlogic.gdx.physics.box2d.Body bottomWall;
    private com.badlogic.gdx.physics.box2d.Body topWall;
    private com.badlogic.gdx.physics.box2d.Body leftWall;
    private com.badlogic.gdx.physics.box2d.Body rightWall;
    
    // UI
    private Stage uiStage;
    private Skin skin;
    private float yellowFriction = 0.0f;  // Friction for XOR gate particles (yellow colored)
    private boolean uiVisible = true;  // Toggle with 'H' key
    private float startingVelocityRange = 24f; // Initial velocity range for particles
    private boolean monochromeModeEnabled = false;
    private final Color monochromeColor = new Color(0.2f, 0.6f, 1f, 1f);
    
    // World configuration
    private int initialSpawnCount = 33; // Total initial particles (singles + molecules)
    private float worldSizeMultiplier = 1.0f; // Size multiplier for world dimensions

    @Override
    public void create() {
        physicsWorld = new PhysicsWorld(new Vector2(0f, 0f), 1024);
        updateWorldBoundaries();
        
        reactionManager = new ReactionManager(physicsWorld, 42L);
        
        // Configure reaction parameters - higher rates for visible interactions
        reactionManager.setBondFormationRadius(2.5f);  // Slightly tighter collision range
        reactionManager.setBondFormationProbability(0.15f);  // Increased from 0.02 for more reactions
        reactionManager.setBondBreakProbability(0.002f);  // Increased from 0.0005 for more dynamics
        
        particleRenderer = new ParticleRenderer();
        particleRenderer.setPhysicsWorld(physicsWorld);
        bondRenderer = new BondRenderer();
        movingEnergyFieldRenderer = new MovingEnergyFieldRenderer();
    updateMonochromeRenderers();
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
        
        // Root table that fills parent
        Table root = new Table();
        root.setFillParent(true);
        
        // Content table for all controls (will be scrollable)
        Table contentTable = new Table();
        contentTable.top().left();
        contentTable.pad(10);
        
        // Use contentTable for all UI elements
        Table table = contentTable;
        
        // World Configuration Section
        Label worldConfigHeader = new Label("=== WORLD CONFIG ===", skin);
        table.add(worldConfigHeader).padBottom(10).row();
        
        // Initial Spawn Count slider
        Label spawnCountLabel = new Label("Spawn Count: 33", skin);
        spawnCountLabel.setWrap(true);
        spawnCountLabel.addListener(createTooltip("Number of particles spawned at simulation start (press R to restart)", skin));
        Slider spawnCountSlider = new Slider(10f, 200f, 1f, false, skin);
        spawnCountSlider.setValue(initialSpawnCount);
        
        spawnCountSlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                initialSpawnCount = (int) spawnCountSlider.getValue();
                spawnCountLabel.setText(String.format("Spawn Count: %d", initialSpawnCount));
            }
        });
        
        table.add(spawnCountLabel).fillX().padBottom(5).row();
        table.add(spawnCountSlider).width(200).row();
        
        // World Size Multiplier slider
        table.add(new Label("", skin)).padTop(15).row(); // Spacer
        Label worldSizeLabel = new Label("World Size: 1.0x", skin);
        worldSizeLabel.setWrap(true);
        worldSizeLabel.addListener(createTooltip("Multiplier for the playing field dimensions (0.5x = quarter size, 2x = quadruple size)", skin));
        Slider worldSizeSlider = new Slider(0.5f, 2.0f, 0.1f, false, skin);
        worldSizeSlider.setValue(worldSizeMultiplier);
        
        worldSizeSlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                worldSizeMultiplier = worldSizeSlider.getValue();
                worldSizeLabel.setText(String.format("World Size: %.1fx", worldSizeMultiplier));
            }
        });
        
        table.add(worldSizeLabel).fillX().padBottom(5).row();
        table.add(worldSizeSlider).width(200).row();
        
        // Simulation Parameters Section
        table.add(new Label("", skin)).padTop(20).row(); // Spacer
        Label simConfigHeader = new Label("=== SIMULATION ===", skin);
        table.add(simConfigHeader).padBottom(10).row();
        
        // XOR gate friction slider (XOR is yellow colored)
        Label frictionLabel = new Label("XOR Friction: 0.00", skin);
        frictionLabel.setWrap(true);
        frictionLabel.addListener(createTooltip("Linear damping applied to XOR (yellow) particles. Higher values slow them down faster.", skin));
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
        
        table.add(frictionLabel).fillX().padBottom(5).row();
        table.add(frictionSlider).width(200).row();
        
        // Reaction Violence slider
        table.add(new Label("", skin)).padTop(15).row(); // Spacer
        Label violenceLabel = new Label("Reaction Violence: 1.00x", skin);
        violenceLabel.addListener(createTooltip("Intensity of explosive forces when incompatible bonds break. Higher = more violent reactions.", skin));
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
        energyDecayLabel.addListener(createTooltip("Rate at which all particles lose energy over time. Particles die when energy reaches zero.", skin));
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
        bondCostLabel.addListener(createTooltip("Energy cost per second for each bond maintained. Bonded particles consume energy to stay connected.", skin));
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
        metabolismLabel.addListener(createTooltip("Rate at which productive bonds generate energy-rich food particles as metabolic byproducts.", skin));
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
        
        // Activation Energy slider (minimum energy for bond formation)
        table.add(new Label("", skin)).padTop(15).row(); // Spacer
        Label activationLabel = new Label("Activation: 20", skin);
        activationLabel.addListener(createTooltip("Minimum energy threshold required for particles to form new bonds. Low-energy particles cannot bond.", skin));
        Slider activationSlider = new Slider(0f, 50f, 5f, false, skin);
        activationSlider.setValue(reactionManager.getActivationEnergy());
        
        activationSlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                float energy = activationSlider.getValue();
                reactionManager.setActivationEnergy(energy);
                activationLabel.setText(String.format("Activation: %.0f", energy));
            }
        });
        
        table.add(activationLabel).padBottom(5).row();
        table.add(activationSlider).width(200).row();
        
        // Replication Energy slider (minimum energy to replicate)
        table.add(new Label("", skin)).padTop(15).row(); // Spacer
        Label replicationLabel = new Label("Replication: 80", skin);
        replicationLabel.addListener(createTooltip("Energy threshold for particle replication. Particles with sufficient energy can duplicate themselves.", skin));
        Slider replicationSlider = new Slider(50f, 100f, 5f, false, skin);
        replicationSlider.setValue(physicsWorld.getReplicationEnergyThreshold());
        
        replicationSlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                float energy = replicationSlider.getValue();
                physicsWorld.setReplicationEnergyThreshold(energy);
                replicationLabel.setText(String.format("Replication: %.0f", energy));
            }
        });
        
        table.add(replicationLabel).padBottom(5).row();
        table.add(replicationSlider).width(200).row();
        
        // Mutation Rate slider (probability of mutation during replication)
        table.add(new Label("", skin)).padTop(15).row(); // Spacer
        Label mutationLabel = new Label("Mutation: 10%", skin);
        mutationLabel.addListener(createTooltip("Probability that a replicated particle will randomly change type (genetic mutation).", skin));
        Slider mutationSlider = new Slider(0f, 0.5f, 0.05f, false, skin);
        mutationSlider.setValue(physicsWorld.getMutationRate());
        
        mutationSlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                float rate = mutationSlider.getValue();
                physicsWorld.setMutationRate(rate);
                mutationLabel.setText(String.format("Mutation: %.0f%%", rate * 100));
            }
        });
        
        table.add(mutationLabel).padBottom(5).row();
        table.add(mutationSlider).width(200).row();
        
        // Environmental Energy slider (absorption rate from field)
        table.add(new Label("", skin)).padTop(15).row(); // Spacer
        Label envEnergyLabel = new Label("Field Energy: 5.0/s", skin);
        envEnergyLabel.addListener(createTooltip("Rate at which particles absorb energy from moving energy fields, scaled by type-specific affinity.", skin));
        Slider envEnergySlider = new Slider(0f, 20f, 1f, false, skin);
        envEnergySlider.setValue(physicsWorld.getEnvironmentalEnergyRate());
        
        envEnergySlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                float rate = envEnergySlider.getValue();
                physicsWorld.setEnvironmentalEnergyRate(rate);
                envEnergyLabel.setText(String.format("Field Energy: %.1f/s", rate));
            }
        });
        
        table.add(envEnergyLabel).padBottom(5).row();
        table.add(envEnergySlider).width(200).row();
        
        // Chemotaxis Strength slider (gradient following force)
        table.add(new Label("", skin)).padTop(15).row(); // Spacer
        Label chemotaxisLabel = new Label("Chemotaxis: 50", skin);
        chemotaxisLabel.addListener(createTooltip("Force strength for gradient-following behavior. Particles move toward energy fields they're attracted to.", skin));
        Slider chemotaxisSlider = new Slider(0f, 200f, 10f, false, skin);
        chemotaxisSlider.setValue(physicsWorld.getChemotaxisStrength());
        
        chemotaxisSlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                float strength = chemotaxisSlider.getValue();
                physicsWorld.setChemotaxisStrength(strength);
                chemotaxisLabel.setText(String.format("Chemotaxis: %.0f", strength));
            }
        });
        
        table.add(chemotaxisLabel).padBottom(5).row();
        table.add(chemotaxisSlider).width(200).row();
        
        // Mitosis Threshold slider (maximum molecule size before division)
        table.add(new Label("", skin)).padTop(15).row(); // Spacer
        Label mitosisLabel = new Label("Mitosis Size: 25", skin);
        mitosisLabel.addListener(createTooltip("Maximum number of particles in a molecule before it divides (mitosis). Higher values allow larger structures.", skin));
        Slider mitosisSlider = new Slider(10f, 50f, 5f, false, skin);
        mitosisSlider.setValue(physicsWorld.getMitosisThreshold());
        
        mitosisSlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                int threshold = (int) mitosisSlider.getValue();
                physicsWorld.setMitosisThreshold(threshold);
                mitosisLabel.setText(String.format("Mitosis Size: %d", threshold));
            }
        });
        
        table.add(mitosisLabel).padBottom(5).row();
        table.add(mitosisSlider).width(200).row();
        
        // Hunting Range slider
        table.add(new Label("", skin)).padTop(15).row(); // Spacer
        Label huntingRangeLabel = new Label("Hunting Range: 15", skin);
        huntingRangeLabel.addListener(createTooltip("Distance within which hungry molecules detect and chase food particles. Larger range = longer-distance hunting.", skin));
        Slider huntingRangeSlider = new Slider(0f, 30f, 5f, false, skin);
        huntingRangeSlider.setValue(physicsWorld.getHuntingRange());
        
        huntingRangeSlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                float range = huntingRangeSlider.getValue();
                physicsWorld.setHuntingRange(range);
                huntingRangeLabel.setText(String.format("Hunting Range: %.0f", range));
            }
        });
        
        table.add(huntingRangeLabel).padBottom(5).row();
        table.add(huntingRangeSlider).width(200).row();
        
        // Hunting Force slider
        Label huntingForceLabel = new Label("Hunting Force: 40", skin);
        huntingForceLabel.addListener(createTooltip("Strength of the force pulling hungry molecules toward food. Higher values = more aggressive hunting.", skin));
        Slider huntingForceSlider = new Slider(0f, 100f, 10f, false, skin);
        huntingForceSlider.setValue(physicsWorld.getHuntingForce());
        
        huntingForceSlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                float force = huntingForceSlider.getValue();
                physicsWorld.setHuntingForce(force);
                huntingForceLabel.setText(String.format("Hunting Force: %.0f", force));
            }
        });
        
        table.add(huntingForceLabel).padBottom(5).row();
        table.add(huntingForceSlider).width(200).row();
        
        // Suction Range slider
        table.add(new Label("", skin)).padTop(15).row(); // Spacer
        Label suctionRangeLabel = new Label("Suction Range: 8", skin);
        suctionRangeLabel.addListener(createTooltip("Distance within which molecules pull food particles toward them. Larger range = wider feeding area.", skin));
        Slider suctionRangeSlider = new Slider(0f, 20f, 2f, false, skin);
        suctionRangeSlider.setValue(physicsWorld.getSuctionRange());
        
        suctionRangeSlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                float range = suctionRangeSlider.getValue();
                physicsWorld.setSuctionRange(range);
                suctionRangeLabel.setText(String.format("Suction Range: %.0f", range));
            }
        });
        
        table.add(suctionRangeLabel).padBottom(5).row();
        table.add(suctionRangeSlider).width(200).row();
        
        // Suction Force slider
        Label suctionForceLabel = new Label("Suction Force: 60", skin);
        suctionForceLabel.addListener(createTooltip("Strength of the force pulling food toward molecules. Higher values = stronger vacuum effect.", skin));
        Slider suctionForceSlider = new Slider(0f, 150f, 10f, false, skin);
        suctionForceSlider.setValue(physicsWorld.getSuctionForce());
        
        suctionForceSlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                float force = suctionForceSlider.getValue();
                physicsWorld.setSuctionForce(force);
                suctionForceLabel.setText(String.format("Suction Force: %.0f", force));
            }
        });
        
        table.add(suctionForceLabel).padBottom(5).row();
        table.add(suctionForceSlider).width(200).row();
        
        // Organelle Energy Rate slider
        table.add(new Label("", skin)).padTop(15).row(); // Spacer
        Label organelleEnergyLabel = new Label("Organelle Energy: 8", skin);
        organelleEnergyLabel.addListener(createTooltip("Energy generated per organelle per second. Higher values allow molecules with organelles to sustain themselves indefinitely. Max (~50) can keep large molecules alive.", skin));
        Slider organelleEnergySlider = new Slider(0f, 50f, 2f, false, skin);
        organelleEnergySlider.setValue(physicsWorld.getOrganelleEnergyRate());
        
        organelleEnergySlider.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                float rate = organelleEnergySlider.getValue();
                physicsWorld.setOrganelleEnergyRate(rate);
                organelleEnergyLabel.setText(String.format("Organelle Energy: %.0f", rate));
            }
        });
        
        table.add(organelleEnergyLabel).padBottom(5).row();
        table.add(organelleEnergySlider).width(200).row();

        // Rendering controls
        table.add(new Label("", skin)).padTop(20).row();
        Label visualsHeader = new Label("=== VISUALS ===", skin);
        table.add(visualsHeader).padBottom(10).row();

        Table monochromeRow = new Table();
        final CheckBox monochromeCheckBox = new CheckBox(" Monochrome Mode", skin);
        monochromeCheckBox.setChecked(monochromeModeEnabled);
        monochromeCheckBox.addListener(new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                monochromeModeEnabled = monochromeCheckBox.isChecked();
                updateMonochromeRenderers();
            }
        });
        monochromeCheckBox.addListener(createTooltip("When enabled, all particles and bonds render with the chosen color.", skin));

        final Image monochromeColorSwatch = new Image(skin.newDrawable("white", Color.WHITE));
        monochromeColorSwatch.setColor(monochromeColor);
        monochromeColorSwatch.setSize(28f, 28f);
        monochromeColorSwatch.setTouchable(Touchable.enabled);
        monochromeColorSwatch.addListener(createTooltip("Click to choose the monochrome color (default blue).", skin));
        monochromeColorSwatch.addListener(new ClickListener() {
            @Override
            public void clicked(InputEvent event, float x, float y) {
                showMonochromeColorPicker(monochromeColorSwatch);
            }
        });

        monochromeRow.add(monochromeCheckBox).left();
        monochromeRow.add(monochromeColorSwatch).size(28f, 28f).padLeft(12f).left();
        table.add(monochromeRow).left().padBottom(5).row();

        Label monochromeHint = new Label("Color applies only when monochrome mode is enabled.", skin);
        monochromeHint.setWrap(true);
        table.add(monochromeHint).width(220f).left().row();
        
        // Create scroll pane for the content
        ScrollPane scrollPane = new ScrollPane(contentTable, skin);
        scrollPane.setFadeScrollBars(false);
        scrollPane.setScrollingDisabled(true, false); // Only vertical scrolling
        scrollPane.setOverscroll(false, false);
        
        // Add title label at top of side panel
        Label titleLabel = new Label("CONTROLS", skin);
        titleLabel.setAlignment(com.badlogic.gdx.utils.Align.center);
        
        // Side panel container (fixed width on right side)
        Table sidePanel = new Table();
        sidePanel.setBackground(skin.newDrawable("white", new com.badlogic.gdx.graphics.Color(0.15f, 0.15f, 0.15f, 0.95f)));
        sidePanel.top();
        sidePanel.add(titleLabel).pad(10).fillX().row();
        sidePanel.add(scrollPane).expand().fill();
        
        // Layout: side panel on the right with fixed width
        root.add().expand().fill(); // Empty space for simulation
        root.add(sidePanel).width(250).fillY();
        
        uiStage.addActor(root);
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

    // TextButton style
    TextButton.TextButtonStyle textButtonStyle = new TextButton.TextButtonStyle();
    textButtonStyle.up = whiteDrawable.tint(new com.badlogic.gdx.graphics.Color(0.25f, 0.25f, 0.25f, 1f));
    textButtonStyle.down = whiteDrawable.tint(new com.badlogic.gdx.graphics.Color(0.2f, 0.2f, 0.2f, 1f));
    textButtonStyle.checked = whiteDrawable.tint(new com.badlogic.gdx.graphics.Color(0.35f, 0.35f, 0.35f, 1f));
    textButtonStyle.font = font;
    textButtonStyle.fontColor = com.badlogic.gdx.graphics.Color.WHITE;
    skin.add("default", textButtonStyle);

    // CheckBox style
    CheckBox.CheckBoxStyle checkBoxStyle = new CheckBox.CheckBoxStyle();
    checkBoxStyle.checkboxOff = whiteDrawable.tint(new com.badlogic.gdx.graphics.Color(0.3f, 0.3f, 0.3f, 1f));
    checkBoxStyle.checkboxOn = whiteDrawable.tint(new com.badlogic.gdx.graphics.Color(0.1f, 0.6f, 0.2f, 1f));
    checkBoxStyle.font = font;
    checkBoxStyle.fontColor = com.badlogic.gdx.graphics.Color.WHITE;
    skin.add("default", checkBoxStyle);

    // Window/Dialog style
    com.badlogic.gdx.scenes.scene2d.ui.Window.WindowStyle windowStyle = new com.badlogic.gdx.scenes.scene2d.ui.Window.WindowStyle();
    windowStyle.titleFont = font;
    windowStyle.titleFontColor = com.badlogic.gdx.graphics.Color.WHITE;
    windowStyle.background = whiteDrawable.tint(new com.badlogic.gdx.graphics.Color(0.1f, 0.1f, 0.1f, 0.95f));
    skin.add("default", windowStyle);
        
        // ScrollPane style
        ScrollPane.ScrollPaneStyle scrollPaneStyle = new ScrollPane.ScrollPaneStyle();
        scrollPaneStyle.background = whiteDrawable.tint(new com.badlogic.gdx.graphics.Color(0.1f, 0.1f, 0.1f, 0.5f));
        scrollPaneStyle.vScroll = whiteDrawable.tint(new com.badlogic.gdx.graphics.Color(0.2f, 0.2f, 0.2f, 1f));
        scrollPaneStyle.vScrollKnob = whiteDrawable.tint(new com.badlogic.gdx.graphics.Color(0.5f, 0.5f, 0.5f, 1f));
        skin.add("default", scrollPaneStyle);
        
        // TextTooltip style with proper label style
        Label.LabelStyle tooltipLabelStyle = new Label.LabelStyle();
        tooltipLabelStyle.font = font;
        tooltipLabelStyle.fontColor = com.badlogic.gdx.graphics.Color.WHITE;
        tooltipLabelStyle.background = whiteDrawable.tint(new com.badlogic.gdx.graphics.Color(0.1f, 0.1f, 0.1f, 0.95f));
        skin.add("tooltip", tooltipLabelStyle);
        
        TextTooltip.TextTooltipStyle tooltipStyle = new TextTooltip.TextTooltipStyle();
        tooltipStyle.label = tooltipLabelStyle;
        tooltipStyle.background = whiteDrawable.tint(new com.badlogic.gdx.graphics.Color(0.1f, 0.1f, 0.1f, 0.95f));
        tooltipStyle.wrapWidth = 200f; // Wrap at 200 pixels
        skin.add("default", tooltipStyle);
        
        return skin;
    }
    
    /**
     * Helper method to create a tooltip with proper wrapping.
     */
    private TextTooltip createTooltip(String text, Skin skin) {
        TextTooltip tooltip = new TextTooltip(text, skin);
        tooltip.getActor().setWrap(true);
        tooltip.setInstant(false);
        return tooltip;
    }

    private void updateMonochromeRenderers() {
        if (particleRenderer != null) {
            particleRenderer.setMonochromeMode(monochromeModeEnabled, monochromeColor);
        }
        if (bondRenderer != null) {
            bondRenderer.setMonochromeMode(monochromeModeEnabled, monochromeColor);
        }
    }

    private void showMonochromeColorPicker(final Image swatch) {
        final Slider rSlider = new Slider(0f, 1f, 0.01f, false, skin);
        final Slider gSlider = new Slider(0f, 1f, 0.01f, false, skin);
        final Slider bSlider = new Slider(0f, 1f, 0.01f, false, skin);
        final Color workingColor = new Color(monochromeColor);
        rSlider.setValue(workingColor.r);
        gSlider.setValue(workingColor.g);
        bSlider.setValue(workingColor.b);

        final Image preview = new Image(skin.newDrawable("white", Color.WHITE));
        preview.setTouchable(Touchable.disabled);
        preview.setColor(workingColor);
        final Label hexLabel = new Label("Hex: " + formatColorHex(workingColor), skin);

        ChangeListener sliderListener = new ChangeListener() {
            @Override
            public void changed(ChangeEvent event, Actor actor) {
                workingColor.set(rSlider.getValue(), gSlider.getValue(), bSlider.getValue(), 1f);
                preview.setColor(workingColor);
                hexLabel.setText("Hex: " + formatColorHex(workingColor));
            }
        };
        rSlider.addListener(sliderListener);
        gSlider.addListener(sliderListener);
        bSlider.addListener(sliderListener);

        Dialog colorDialog = new Dialog("Pick Monochrome Color", skin) {
            @Override
            protected void result(Object object) {
                boolean accepted = Boolean.TRUE.equals(object);
                if (accepted) {
                    monochromeColor.set(workingColor);
                    updateMonochromeRenderers();
                    swatch.setColor(monochromeColor);
                }
            }
        };

        Table content = colorDialog.getContentTable();
        content.pad(12f);
        content.defaults().padBottom(8f).left();

        content.add(new Label("Preview", skin)).left();
        content.row();
        content.add(preview).size(100f, 40f);
        content.row();
        content.add(hexLabel).left();
        content.row();

        Table redRow = new Table();
        redRow.add(new Label("Red", skin)).width(60f).left();
        redRow.add(rSlider).width(200f).padLeft(10f);
        content.add(redRow).growX();
        content.row();

        Table greenRow = new Table();
        greenRow.add(new Label("Green", skin)).width(60f).left();
        greenRow.add(gSlider).width(200f).padLeft(10f);
        content.add(greenRow).growX();
        content.row();

        Table blueRow = new Table();
        blueRow.add(new Label("Blue", skin)).width(60f).left();
        blueRow.add(bSlider).width(200f).padLeft(10f);
        content.add(blueRow).growX();
        content.row();

        colorDialog.button("Cancel", false);
        colorDialog.button("Apply", true);
        colorDialog.getButtonTable().pad(10f);
        colorDialog.show(uiStage);
    }

    private String formatColorHex(Color color) {
        int r = MathUtils.clamp(Math.round(color.r * 255f), 0, 255);
        int g = MathUtils.clamp(Math.round(color.g * 255f), 0, 255);
        int b = MathUtils.clamp(Math.round(color.b * 255f), 0, 255);
        return String.format("#%02X%02X%02X", r, g, b);
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
            physicsWorld.destroyBond(bond, false, true);  // No violent reactions on restart
        }
        
        for (Particle particle : particlesToRemove) {
            physicsWorld.destroyParticle(particle);
        }
        
        // Update world boundaries based on current size multiplier
        updateWorldBoundaries();
        
        // Reset accumulator
        accumulator = 0f;
        
        // Reseed random for variety
        typeRandom.setSeed(System.currentTimeMillis());
        
        // Spawn new demo molecules with current spawn count
        spawnDemoMolecules();
        
        System.out.println("Simulation restarted with " + initialSpawnCount + " particles, world size: " + worldSizeMultiplier + "x");
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
        totalTime += delta; // Track total time for animations
        
        // Update camera for keyboard panning (arrow keys) and zoom (+/- keys)
        cameraController.update(delta);
        
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

        // Render moving energy fields as background
        movingEnergyFieldRenderer.render(physicsWorld.getMovingFields(), camera);
        
        bondRenderer.render(physicsWorld.getActiveBonds(), camera);
        particleRenderer.render(physicsWorld.getActiveParticles(), camera, totalTime);
        
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
        movingEnergyFieldRenderer.dispose();
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
        
        // Calculate distribution based on initialSpawnCount
        // Default was 33 total particles: 10 singles, 8 pairs, 6 triplets, 4 triangles, 3 branched(4-6), 2 large(8-10)
        // Scale proportionally
        int count = initialSpawnCount;
        int singles = Math.max(1, (int)(count * 0.30f)); // 30% singles
        int pairs = Math.max(1, (int)(count * 0.24f));   // 24% pairs
        int triplets = Math.max(1, (int)(count * 0.18f)); // 18% triplets
        int triangles = Math.max(1, (int)(count * 0.12f)); // 12% triangles
        int branched = Math.max(1, (int)(count * 0.09f)); // 9% branched
        int large = Math.max(1, (int)(count * 0.06f)); // 6% large
        
        // Singles
        for (int i = 0; i < singles; i++) {
            Vector2 pos = findFreePosition(occupiedPositions, minSpacing);
            spawnMolecule(1, pos.x, pos.y);
            occupiedPositions.add(pos);
        }
        
        // Pairs
        for (int i = 0; i < pairs; i++) {
            Vector2 pos = findFreePosition(occupiedPositions, minSpacing);
            spawnAngleMolecule(2, pos.x, pos.y);
            occupiedPositions.add(pos);
        }
        
        // Triplets - varied angles
        for (int i = 0; i < triplets; i++) {
            Vector2 pos = findFreePosition(occupiedPositions, minSpacing);
            spawnAngleMolecule(3, pos.x, pos.y);
            occupiedPositions.add(pos);
        }
        
        // Triangles
        for (int i = 0; i < triangles; i++) {
            Vector2 pos = findFreePosition(occupiedPositions, minSpacing);
            spawnTriangleMol(pos.x, pos.y);
            occupiedPositions.add(pos);
        }
        
        // Branched structures (4-6 nodes)
        for (int i = 0; i < branched; i++) {
            Vector2 pos = findFreePosition(occupiedPositions, minSpacing);
            spawnBranchedMol(MathUtils.random(4, 6), pos.x, pos.y);
            occupiedPositions.add(pos);
        }
        
        // Large irregular mols (8-10 nodes)
        for (int i = 0; i < large; i++) {
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
        float currentWidth = getCurrentWorldWidth();
        float currentHeight = getCurrentWorldHeight();
        float spawnRangeX = currentWidth * 0.4f;  // Use 80% of world width
        float spawnRangeY = currentHeight * 0.4f;  // Use 80% of world height
        
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
     * Create or update Box2D static bodies as walls around the world boundaries.
     * Supports dynamic world size based on worldSizeMultiplier.
     */
    private void updateWorldBoundaries() {
        float actualWidth = WORLD_WIDTH * worldSizeMultiplier;
        float actualHeight = WORLD_HEIGHT * worldSizeMultiplier;
        float halfWidth = actualWidth / 2f;
        float halfHeight = actualHeight / 2f;
        float wallThickness = 1f;
        
        // Update moving field world size
        physicsWorld.setWorldSize(actualWidth, actualHeight);
        
        com.badlogic.gdx.physics.box2d.World world = physicsWorld.getWorld();
        
        // Remove existing walls if they exist
        if (bottomWall != null) world.destroyBody(bottomWall);
        if (topWall != null) world.destroyBody(topWall);
        if (leftWall != null) world.destroyBody(leftWall);
        if (rightWall != null) world.destroyBody(rightWall);
        
        com.badlogic.gdx.physics.box2d.BodyDef bodyDef = new com.badlogic.gdx.physics.box2d.BodyDef();
        bodyDef.type = com.badlogic.gdx.physics.box2d.BodyDef.BodyType.StaticBody;
        
        // Bottom wall
        bodyDef.position.set(0, -halfHeight);
        bottomWall = world.createBody(bodyDef);
        com.badlogic.gdx.physics.box2d.PolygonShape bottomBox = new com.badlogic.gdx.physics.box2d.PolygonShape();
        bottomBox.setAsBox(halfWidth + wallThickness, wallThickness);
        bottomWall.createFixture(bottomBox, 0f);
        bottomBox.dispose();
        
        // Top wall
        bodyDef.position.set(0, halfHeight);
        topWall = world.createBody(bodyDef);
        com.badlogic.gdx.physics.box2d.PolygonShape topBox = new com.badlogic.gdx.physics.box2d.PolygonShape();
        topBox.setAsBox(halfWidth + wallThickness, wallThickness);
        topWall.createFixture(topBox, 0f);
        topBox.dispose();
        
        // Left wall
        bodyDef.position.set(-halfWidth, 0);
        leftWall = world.createBody(bodyDef);
        com.badlogic.gdx.physics.box2d.PolygonShape leftBox = new com.badlogic.gdx.physics.box2d.PolygonShape();
        leftBox.setAsBox(wallThickness, halfHeight + wallThickness);
        leftWall.createFixture(leftBox, 0f);
        leftBox.dispose();
        
        // Right wall
        bodyDef.position.set(halfWidth, 0);
        rightWall = world.createBody(bodyDef);
        com.badlogic.gdx.physics.box2d.PolygonShape rightBox = new com.badlogic.gdx.physics.box2d.PolygonShape();
        rightBox.setAsBox(wallThickness, halfHeight + wallThickness);
        rightWall.createFixture(rightBox, 0f);
        rightBox.dispose();
    }
    
    /**
     * Get current world width based on size multiplier.
     */
    private float getCurrentWorldWidth() {
        return WORLD_WIDTH * worldSizeMultiplier;
    }
    
    /**
     * Get current world height based on size multiplier.
     */
    private float getCurrentWorldHeight() {
        return WORLD_HEIGHT * worldSizeMultiplier;
    }
}
