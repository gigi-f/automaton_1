package com.automaton.core.render;

import com.automaton.core.physics.Particle;
import com.automaton.core.physics.PhysicsWorld;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Pixmap.Format;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Disposable;

/**
 * Renders particles using a batched sprite. A small circular texture is generated
 * at runtime to avoid asset dependencies.
 * Particles are colored by their type representing logic gates and states:
 * - TRUE (green), FALSE (gray) - logic states
 * - AND (red), OR (orange), NOT (purple), NAND (pink), NOR (cyan), XOR (yellow), XNOR (magenta) - logic gates
 */
public final class ParticleRenderer implements Disposable {
    private static final int TEXTURE_SIZE = 32;

    private final SpriteBatch batch;
    private final TextureRegion circleRegion;
    private final Color tmpColor = new Color();
    private PhysicsWorld physicsWorld; // For component size queries

    public ParticleRenderer() {
        this.batch = new SpriteBatch();
        Texture circleTexture = buildCircleTexture(TEXTURE_SIZE);
        this.circleRegion = new TextureRegion(circleTexture);
    }

    public void setPhysicsWorld(PhysicsWorld world) {
        this.physicsWorld = world;
    }

    public void render(Array<Particle> particles, OrthographicCamera camera) {
        render(particles, camera, 0f);
    }
    
    public void render(Array<Particle> particles, OrthographicCamera camera, float time) {
        batch.setProjectionMatrix(camera.combined);
        batch.begin();
        for (Particle particle : particles) {
            Vector2 pos = particle.getPosition();
            float radius = particle.getVisualRadius();
            float diameter = radius * 2f;
            float drawX = pos.x - radius;
            float drawY = pos.y - radius;
            
            // Check if particle is an organelle
            if (particle.isOrganelle()) {
                // Organelles: pulsing bright green glow
                Color typeColor = particle.getType().getColor();
                
                // Pulsing effect: brightness oscillates between 1.0 and 1.5
                float pulse = 1.0f + 0.5f * (0.5f + 0.5f * MathUtils.sin(time * 3f));
                
                tmpColor.set(typeColor.r * pulse, typeColor.g * pulse, 
                            typeColor.b * pulse, 1.0f);
                
                // Draw main particle
                batch.setColor(tmpColor);
                batch.draw(circleRegion, drawX, drawY, diameter, diameter);
                
                // Draw outer glow ring with transparency
                float glowRadius = radius * 1.4f;
                float glowDiameter = glowRadius * 2f;
                float glowX = pos.x - glowRadius;
                float glowY = pos.y - glowRadius;
                
                tmpColor.a = 0.3f * pulse * 0.5f; // Semi-transparent pulsing glow
                batch.setColor(tmpColor);
                batch.draw(circleRegion, glowX, glowY, glowDiameter, glowDiameter);
                
            } else if (!particle.isAlive()) {
                // Dead particles: grayscale with reduced opacity
                float gray = 0.4f;  // Dark gray
                tmpColor.set(gray, gray, gray, 0.5f); // 50% opacity
                batch.setColor(tmpColor);
                batch.draw(circleRegion, drawX, drawY, diameter, diameter);
                
            } else {
                // Living particles: color based on type, brightness based on energy
                Color typeColor = particle.getType().getColor();
                float energy = particle.getEnergy();
                float energyMultiplier = energy / 100f;  // 0-1 range
                
                // Lerp between dark (low energy) and full brightness (high energy)
                float minBrightness = 0.3f;  // Minimum brightness at 0 energy
                float brightness = minBrightness + (1f - minBrightness) * energyMultiplier;
                
                tmpColor.set(typeColor.r * brightness, typeColor.g * brightness, 
                            typeColor.b * brightness, typeColor.a);
                
                batch.setColor(tmpColor);
                batch.draw(circleRegion, drawX, drawY, diameter, diameter);
            }
            
            // Check for mitosis flash effect (applies to all particle types)
            if (physicsWorld != null && physicsWorld.isParticleMitosisFlashing(particle)) {
                float flashIntensity = physicsWorld.getParticleMitosisFlashIntensity(particle);
                
                // Draw bright cyan/white flash overlay
                float flashRadius = radius * (1.2f + 0.3f * flashIntensity); // Expands as it fades
                float flashDiameter = flashRadius * 2f;
                float flashX = pos.x - flashRadius;
                float flashY = pos.y - flashRadius;
                
                // Cyan-white color (0.3, 1.0, 1.0) mixed with white
                tmpColor.set(0.3f + 0.7f * flashIntensity, 1.0f, 1.0f, flashIntensity * 0.7f);
                batch.setColor(tmpColor);
                batch.draw(circleRegion, flashX, flashY, flashDiameter, flashDiameter);
            }
        }
        batch.setColor(Color.WHITE); // Reset
        batch.end();
    }

    /**
     * Get color for a mol based on its size.
     * Gradient: white (size 1) -> light blue -> dark blue (size 10+)
     */
    private void getColorForMolSize(int size, Color out) {
        // Clamp size to [1, 10]
        float t = MathUtils.clamp((size - 1) / 9f, 0f, 1f);
        
        // Interpolate from white (1,1,1) to dark blue (0.1, 0.1, 0.5)
        out.r = 1f - t * 0.9f;
        out.g = 1f - t * 0.9f;
        out.b = 1f - t * 0.5f;
        out.a = 1f;
    }

    @Override
    public void dispose() {
        batch.dispose();
        circleRegion.getTexture().dispose();
    }

    private Texture buildCircleTexture(int size) {
        Pixmap pixmap = new Pixmap(size, size, Format.RGBA8888);
        pixmap.setBlending(Pixmap.Blending.None);
        pixmap.setColor(1f, 1f, 1f, 1f);
        int radius = size / 2;
        for (int y = 0; y < size; y++) {
            for (int x = 0; x < size; x++) {
                int dx = x - radius;
                int dy = y - radius;
                if (dx * dx + dy * dy <= radius * radius) {
                    pixmap.drawPixel(x, y);
                }
            }
        }
        Texture texture = new Texture(pixmap);
        pixmap.dispose();
        return texture;
    }
}
