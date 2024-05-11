package frc.team4481.lib.auto.mode;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.team4481.lib.auto.BasicAutoRunnable;

import java.util.concurrent.atomic.AtomicReference;

/**
 * The AutoModeExecutor class runs, and (if necessary) stops a specified autonomous mode {@code AutoModeBase}.
 * To make sure the specified autonomous mode runs simultaneously with the subsystem looper a different thread
 * is created. This also allows the autonomous mode to be terminated if it is not completed by the time teleop
 * period starts.
 *
 * @author Team 254 - The Cheesy Poofs
 *
 * Modified by Team 4481 - Team Rembrandts
 */
public class AutoModeExecutor extends BasicAutoRunnable {
    private final AtomicReference<Thread> thread = new AtomicReference<>(null);

    /**
     * sets {@code AutoModeBase} as desired autonomous instance to execute
     *
     * Prepares a thread for the autonomous execution.
     *
     * @param autoMode autonomous mode instance to run
     */
    public void setAutoMode(AutoModeBase autoMode) {
        super.setAutoMode(autoMode);
        thread.set(new Thread(this));
    }

    /**
     * Starts a separate thread for autonomous execution
     */
    public void start() {
        if (thread.get() != null) {
            DataLogManager.log("Start auto runnable");
            thread.get().start();
        }
    }

    /**
     * Stops the execution of the autonomous mode and terminates the thread
     */
    public void stop() {
        if (getAutoMode() != null) {
            DataLogManager.log("Stop auto runnable");
            super.stop();
        }

        thread.set(null);
    }

    /**
     * Checks whether the thread is still active
     *
     * @return if the thread is active or not
     */
    public boolean isRunning() {
        if (thread.get() != null) {
            return  thread.get().isAlive();
        }
        return false;
    }
}

