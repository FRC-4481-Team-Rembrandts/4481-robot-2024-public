package frc.team4481.lib.auto.actions;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

/**
 * Simple wait action useful as a member of {@link SeriesAction} or {@link ParallelAction} or for early
 * autonomous mode testing
 *
 * @author Team 254 - The Cheesy Poofs
 */
public class WaitAction implements Action{
    private final double timeToWait;
    private double startTime;

    public WaitAction(double timeToWait /*ms>*/) {
        this.timeToWait = timeToWait;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= timeToWait;
    }

    @Override
    public void done() {

    }
}

// Copyright (c) 2019 Team 254
