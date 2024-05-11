package frc.team4481.lib.looper;

/**
 * Loop interface for a class which needs to be executed by a looper.
 *
 * Contains
 * - onStart() function which is executed at initialisation
 * - onLoop() function which is executed every update cycle
 * - onStop() function which is executed at termination.
 *
 * @author Team 254 - The Cheesy Poofs
 */
public interface Loop {
    void onStart(double timestamp);

    void onLoop(double timestamp);

    void onStop(double timestamp);
}

// Copyright (c) 2019 Team 254
