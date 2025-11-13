package com.automaton.desktop;

import com.automaton.core.AutomatonGame;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;

public class DesktopLauncher {
    public static void main(String[] args) {
        Lwjgl3ApplicationConfiguration config = new Lwjgl3ApplicationConfiguration();
        config.setTitle("Automaton");
        config.setWindowedMode(800, 600);
        new Lwjgl3Application(new AutomatonGame(), config);
    }
}
