package frc.team4481.lib.auto.mode;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.path.PathNotLoadedException;


/**
 * The AutoModeBase is the base class for each autonomous mode program.
 *
 * @author Team 254 - The Cheesy Poofs
 */
public abstract class AutoModeBase {
    protected double updateRate = 0.005;
    protected boolean active = false;

    /**
     * Abstract class for definition of the autonomous program
     *
     * @throws AutoModeEndedException on early termination
     */
    protected abstract void routine() throws AutoModeEndedException;

    protected abstract void initialize() throws AutoModeEndedException, PathNotLoadedException;

    /**
     * Runs the autonomous routine {@see routine()} and deals with exception on early termination.
     */
    public void run() {
        active = true;

        try {
            routine();
        } catch (AutoModeEndedException e) {
            DriverStation.reportError("AUTO MODE ENDED EARLY", false);
            return;
        }

        done();
    }

    public void init(){
        try{
            DataLogManager.log("Auto mode init");
            initialize();
        } catch (AutoModeEndedException | PathNotLoadedException e){
            DriverStation.reportError("Initialize failed", false);
        }
    }

    /**
     * Function to run on ending autonomous mode
     */
    public void done() {
        DataLogManager.log("Auto mode done");
    }

    /**
     * Sets mActive to false which is used for the action runner {@see runAction()}
     */
    public void stop() {
        active = false;
    }

    /**
     * @return true when autonomous mode is active
     * @throws AutoModeEndedException when autonomous mode ended
     */
    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!active) {
            throw new AutoModeEndedException();
        }

        return true;
    }

    /**
     * Runs actions inside the routine function {@see routine} for the selected autonomous mode instance.
     * Terminates the actions early if autonomous mode ended early.
     * The action runner function rate is controlled by sleeping the thread for a desired amount of time.
     *
     * @param action action item to run
     * @throws AutoModeEndedException if autonomous becomes deactivated during action run.
     */
    public void runAction(Action action) throws AutoModeEndedException {
        isActiveWithThrow();

        DataLogManager.log("Start action:" + action);
        double startTime = MathSharedStore.getTimestamp();
        action.start();

        while (isActiveWithThrow() && !action.isFinished()) {
            action.update();
            long waitTime = (long) (updateRate * 1000.0);

            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();
        DataLogManager.log("Stop action:" + action + ", Time elapsed: " + (MathSharedStore.getTimestamp() - startTime));
    }
}

// Copyright (c) 2019 Team 254