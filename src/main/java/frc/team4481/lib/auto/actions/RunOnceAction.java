package frc.team4481.lib.auto.actions;

/**
 * Template action for something that only needs to be done once and has no need for updates.
 *
 * @see Action
 *
 * @author Team 254 - The Cheesy Poofs
 */
public abstract class RunOnceAction implements Action {
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        runOnce();
    }

    public abstract void runOnce();
}

// Copyright (c) 2019 Team 254