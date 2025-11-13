package com.automaton.core;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.InputAdapter;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;

/**
 * Handles camera controls for navigation:
 * - Pan: click and drag with left mouse button
 * - Zoom: scroll wheel
 * - Screen-to-world coordinate conversion
 */
public class CameraController extends InputAdapter {
    private static final float MIN_ZOOM = 0.1f;
    private static final float MAX_ZOOM = 10f;
    private static final float ZOOM_SPEED = 0.1f;

    private final OrthographicCamera camera;
    private final Vector3 tmpVec3 = new Vector3();
    private final Vector2 lastDragPos = new Vector2();
    private boolean isDragging = false;

    public CameraController(OrthographicCamera camera) {
        this.camera = camera;
    }

    @Override
    public boolean touchDown(int screenX, int screenY, int pointer, int button) {
        if (button == Input.Buttons.LEFT) {
            isDragging = true;
            lastDragPos.set(screenX, screenY);
            return true;
        }
        return false;
    }

    @Override
    public boolean touchUp(int screenX, int screenY, int pointer, int button) {
        if (button == Input.Buttons.LEFT) {
            isDragging = false;
            return true;
        }
        return false;
    }

    @Override
    public boolean touchDragged(int screenX, int screenY, int pointer) {
        if (isDragging) {
            // Calculate delta in screen space
            float deltaX = screenX - lastDragPos.x;
            float deltaY = screenY - lastDragPos.y;
            
            // Convert to world space (accounting for zoom)
            float worldDeltaX = deltaX * camera.zoom;
            float worldDeltaY = -deltaY * camera.zoom; // Y is inverted in screen space
            
            // Move camera in opposite direction of drag
            camera.position.x -= worldDeltaX;
            camera.position.y -= worldDeltaY;
            camera.update();
            
            lastDragPos.set(screenX, screenY);
            return true;
        }
        return false;
    }

    @Override
    public boolean scrolled(float amountX, float amountY) {
        // Zoom in/out with scroll wheel
        float zoomDelta = amountY * ZOOM_SPEED;
        camera.zoom = MathUtils.clamp(camera.zoom + zoomDelta, MIN_ZOOM, MAX_ZOOM);
        camera.update();
        return true;
    }

    /**
     * Convert screen coordinates to world coordinates.
     * @param screenX Screen X coordinate
     * @param screenY Screen Y coordinate
     * @return World position as Vector2
     */
    public Vector2 screenToWorld(int screenX, int screenY) {
        tmpVec3.set(screenX, screenY, 0);
        camera.unproject(tmpVec3);
        return new Vector2(tmpVec3.x, tmpVec3.y);
    }

    /**
     * Reset camera to default position and zoom.
     */
    public void reset() {
        camera.position.set(0f, 0f, 0f);
        camera.zoom = 1f;
        camera.update();
    }
}
